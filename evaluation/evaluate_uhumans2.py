#!/usr/bin/env python3

"""
Evaluation node for uHumans2-style datasets.

Compares the live SLAM output of a mono-inertial vS-Graphs run against the
ground truth published on /tf by the dataset itself:

  - Camera platform: /tf world -> base_link_gt      (ground truth)
                      /vs_graphs/body_odom           (estimate, world -> imu)
  - Humans:           /tf world -> object_<id>       (ground truth, 0..N humans)
                      /dynamic_object_points_3d      (estimate, per-keyframe 3D points)

Camera and human estimates share the same SLAM-internal "world" frame, so a
single rigid (rotation + translation, no scale) alignment is computed from the
camera trajectory and applied to both before computing errors. Scale is not
applied (mono-inertial SLAM should already be metric); the recovered scale is
reported only as a diagnostic to sanity-check IMU scale correctness.

Human-to-ground-truth association is solved independently at every keyframe
via Hungarian assignment (minimum total distance) with a distance gate, since
estimated track ids are not guaranteed to match ground-truth human ids and the
number of humans visible at any instant may be 0, 1, or many.

Usage (after sourcing the workspace and while a run is playing back):
    python3 evaluate_uhumans2.py --ros-args \
        -p output_dir:=/path/to/results \
        -p body_odom_topic:=/vs_graphs/body_odom \
        -p human_points_topic:=/dynamic_object_points_3d

The node buffers samples as they arrive and finalizes automatically once no
new messages have been seen for `idle_timeout_s` seconds (i.e. the bag/run has
ended), writing three CSV files to `output_dir`:
  - camera_pose_errors.csv   (per-sample camera position/orientation error)
  - human_pose_errors.csv    (per-keyframe, per-track human position error)
  - human_track_summary.csv  (per estimated track_id: dominant matched GT id,
                               match purity, mean/RMSE position error)
Ctrl+C also triggers a graceful finalize before exit.
"""

import csv
import math
import os
import re
import threading
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from nav_msgs.msg import Odometry
from dynamic_keypoint_3d_lifter.msg import DynamicObjectPoints3D

OBJECT_FRAME_RE_TEMPLATE = r"^{prefix}(\d+)$"


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def rigid_align(src: np.ndarray, dst: np.ndarray):
    """Closed-form rigid (rotation + translation, no scale) alignment mapping
    `src` points onto `dst` points (Umeyama 1991, scale term computed only as
    a diagnostic and not applied). src, dst: (N,3) arrays of corresponding
    points, N >= 3.

    Returns (R (3,3), t (3,), scale_diagnostic (float)).
    """
    n = src.shape[0]
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_c = src - src_mean
    dst_c = dst - dst_mean

    h = src_c.T @ dst_c
    u, s, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    if d == 0:
        d = 1.0
    correction = np.diag([1.0, 1.0, d])
    r = vt.T @ correction @ u.T
    t = dst_mean - r @ src_mean

    var_src = float((src_c ** 2).sum()) / n
    scale_diag = float((s * np.array([1.0, 1.0, d])).sum() / (n * var_src)) if var_src > 0 else float("nan")
    return r, t, scale_diag


# The environment this was developed against has a broken scipy install
# (scipy.sparse.linalg._propack is missing symbols, which scipy.optimize and
# scipy.spatial both transitively import), so quaternion composition and the
# min-cost assignment below are implemented directly in numpy instead of
# depending on scipy. Both are covered by self-tests during development
# (Hungarian algorithm checked against brute force, quaternion <-> matrix
# round-trip and composition checked against a QR-based random rotation
# generator) - see conversation history for the verification runs.

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product q1 * q2, both in ROS [x, y, z, w] convention."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quat_from_matrix(m: np.ndarray) -> np.ndarray:
    """Rotation matrix (3,3) -> unit quaternion [x, y, z, w] (Shepperd's method)."""
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quat_angle_deg(q1: np.ndarray, q2: np.ndarray) -> float:
    """Geodesic angle in degrees between two unit quaternions (double-cover safe)."""
    dot = abs(float(np.dot(q1, q2)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def hungarian_min_cost_assignment(cost: np.ndarray):
    """Minimum-cost bipartite assignment (Kuhn-Munkres with potentials, O(n^2 m)).
    cost: (n_rows, n_cols). Returns (row_indices, col_indices) covering every row
    if n_rows <= n_cols, or every column otherwise - i.e. min(n_rows, n_cols) pairs,
    matching the semantics of scipy.optimize.linear_sum_assignment for a fully
    finite cost matrix. Self-contained to avoid this environment's broken scipy
    install (see module docstring above).
    """
    cost = np.asarray(cost, dtype=float)
    n_rows, n_cols = cost.shape
    if n_rows == 0 or n_cols == 0:
        return np.array([], dtype=int), np.array([], dtype=int)

    transposed = n_rows > n_cols
    work = cost.T if transposed else cost
    n, m = work.shape  # n <= m guaranteed here

    inf = float("inf")
    u = [0.0] * (n + 1)
    v = [0.0] * (m + 1)
    p = [0] * (m + 1)  # p[j] = row (1-indexed) currently matched to column j
    way = [0] * (m + 1)
    a = np.zeros((n + 1, m + 1))
    a[1:, 1:] = work

    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [inf] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = inf
            j1 = -1
            for j in range(1, m + 1):
                if not used[j]:
                    cur = a[i0][j] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1

    row_ind, col_ind = [], []
    for j in range(1, m + 1):
        if p[j] != 0:
            row_ind.append(p[j] - 1)
            col_ind.append(j - 1)
    row_ind = np.array(row_ind, dtype=int)
    col_ind = np.array(col_ind, dtype=int)
    order = np.argsort(row_ind)
    row_ind, col_ind = row_ind[order], col_ind[order]

    if transposed:
        row_ind, col_ind = col_ind, row_ind
    return row_ind, col_ind


class UHumans2Evaluator(Node):
    def __init__(self):
        super().__init__("uhumans2_evaluator")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("camera_gt_frame", "base_link_gt")
        self.declare_parameter("object_frame_prefix", "object_")
        self.declare_parameter("body_odom_topic", "/vs_graphs/body_odom")
        self.declare_parameter("human_points_topic", "/dynamic_object_points_3d")
        self.declare_parameter("output_dir", ".")
        self.declare_parameter("idle_timeout_s", 5.0)
        self.declare_parameter("tf_lookup_timeout_s", 0.05)
        self.declare_parameter("max_match_distance_m", 1.5)
        self.declare_parameter("tf_cache_time_s", 60.0)

        self.world_frame = self.get_parameter("world_frame").value
        self.camera_gt_frame = self.get_parameter("camera_gt_frame").value
        self.object_frame_prefix = self.get_parameter("object_frame_prefix").value
        self.body_odom_topic = self.get_parameter("body_odom_topic").value
        self.human_points_topic = self.get_parameter("human_points_topic").value
        self.output_dir = self.get_parameter("output_dir").value
        self.idle_timeout_s = float(self.get_parameter("idle_timeout_s").value)
        self.tf_lookup_timeout_s = float(self.get_parameter("tf_lookup_timeout_s").value)
        self.max_match_distance_m = float(self.get_parameter("max_match_distance_m").value)
        tf_cache_time_s = float(self.get_parameter("tf_cache_time_s").value)

        self._object_frame_re = re.compile(
            OBJECT_FRAME_RE_TEMPLATE.format(prefix=re.escape(self.object_frame_prefix)))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=tf_cache_time_s))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self._lock = threading.Lock()
        self.camera_samples = []
        self.human_samples = []
        self._finalized = False
        self.last_msg_time = self.get_clock().now()

        self.create_subscription(Odometry, self.body_odom_topic, self._odom_cb, 50)
        self.create_subscription(
            DynamicObjectPoints3D, self.human_points_topic, self._human_cb, 50)
        self.watchdog_timer = self.create_timer(1.0, self._watchdog_cb)

        self.get_logger().info(
            f"uhumans2_evaluator: world_frame='{self.world_frame}' "
            f"camera_gt_frame='{self.camera_gt_frame}' "
            f"body_odom_topic='{self.body_odom_topic}' "
            f"human_points_topic='{self.human_points_topic}' "
            f"output_dir='{self.output_dir}' idle_timeout_s={self.idle_timeout_s}")

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        self.last_msg_time = self.get_clock().now()
        stamp = msg.header.stamp
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.camera_gt_frame, Time.from_msg(stamp),
                timeout=Duration(seconds=self.tf_lookup_timeout_s))
        except TransformException as ex:
            self.get_logger().warn(
                f"No GT transform {self.world_frame}->{self.camera_gt_frame} at "
                f"t={stamp_to_sec(stamp):.3f}: {ex}", throttle_duration_sec=2.0)
            return

        est_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,
                             msg.pose.pose.position.z])
        est_quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        gt_pos = np.array([tf.transform.translation.x, tf.transform.translation.y,
                            tf.transform.translation.z])
        gt_quat = np.array([tf.transform.rotation.x, tf.transform.rotation.y,
                             tf.transform.rotation.z, tf.transform.rotation.w])

        with self._lock:
            self.camera_samples.append({
                "t": stamp_to_sec(stamp),
                "est_pos": est_pos,
                "est_quat": est_quat,
                "gt_pos": gt_pos,
                "gt_quat": gt_quat,
            })

    def _discover_object_frames(self):
        yaml_str = self.tf_buffer.all_frames_as_yaml()
        ids = []
        for line in yaml_str.splitlines():
            line = line.strip()
            if not line.endswith(":"):
                continue
            match = self._object_frame_re.match(line[:-1])
            if match:
                ids.append(match.group(1))
        return ids

    def _human_cb(self, msg: DynamicObjectPoints3D):
        self.last_msg_time = self.get_clock().now()
        if not msg.objects:
            return
        stamp = msg.header.stamp
        t = Time.from_msg(stamp)

        gt_snapshot = {}
        for object_id in self._discover_object_frames():
            frame = f"{self.object_frame_prefix}{object_id}"
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.world_frame, frame, t,
                    timeout=Duration(seconds=self.tf_lookup_timeout_s))
            except TransformException:
                continue
            gt_snapshot[object_id] = np.array([
                tf.transform.translation.x, tf.transform.translation.y,
                tf.transform.translation.z])

        new_samples = []
        for obj in msg.objects:
            if not obj.points_world:
                continue
            pts = np.array([[p.x, p.y, p.z] for p in obj.points_world])
            new_samples.append({
                "t": stamp_to_sec(stamp),
                "keyframe_id": msg.keyframe_id,
                "track_id": obj.track_id,
                "class_id": obj.class_id,
                "centroid_raw": pts.mean(axis=0),
                "n_points": pts.shape[0],
                "gt_snapshot": gt_snapshot,
            })

        with self._lock:
            self.human_samples.extend(new_samples)

    # ------------------------------------------------------------------
    # Finalization
    # ------------------------------------------------------------------
    def _watchdog_cb(self):
        if self._finalized:
            return
        now = self.get_clock().now()
        idle_s = (now - self.last_msg_time).nanoseconds * 1e-9
        have_data = len(self.camera_samples) > 0 or len(self.human_samples) > 0
        if have_data and idle_s > self.idle_timeout_s:
            self.get_logger().info(
                f"No new data for {idle_s:.1f}s (>{self.idle_timeout_s}s) - "
                "assuming the run has ended, finalizing evaluation.")
            self.finalize()

    def finalize(self):
        with self._lock:
            if self._finalized:
                return
            self._finalized = True
            camera_samples = list(self.camera_samples)
            human_samples = list(self.human_samples)

        os.makedirs(self.output_dir, exist_ok=True)

        r, t, scale_diag = self._write_camera_csv(camera_samples)
        self._write_human_csvs(human_samples, r, t)

        self.get_logger().info(
            f"Evaluation complete. camera_samples={len(camera_samples)} "
            f"human_samples={len(human_samples)} scale_diagnostic={scale_diag:.4f} "
            f"(should be close to 1.0 for correctly-scaled mono-inertial output). "
            f"CSVs written to '{self.output_dir}'.")

    def _write_camera_csv(self, camera_samples):
        path = os.path.join(self.output_dir, "camera_pose_errors.csv")
        if len(camera_samples) < 3:
            self.get_logger().warn(
                f"Only {len(camera_samples)} camera sample(s) with matching GT - "
                "need >=3 to compute alignment. Writing raw (unaligned) values only.")
            r, t, scale_diag = np.eye(3), np.zeros(3), float("nan")
        else:
            src = np.stack([s["est_pos"] for s in camera_samples])
            dst = np.stack([s["gt_pos"] for s in camera_samples])
            r, t, scale_diag = rigid_align(src, dst)

        q_r = quat_from_matrix(r)
        pos_errors = []
        ang_errors = []
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "gt_x", "gt_y", "gt_z", "gt_qx", "gt_qy", "gt_qz", "gt_qw",
                "est_x_raw", "est_y_raw", "est_z_raw",
                "est_x_aligned", "est_y_aligned", "est_z_aligned",
                "est_qx_aligned", "est_qy_aligned", "est_qz_aligned", "est_qw_aligned",
                "position_error_m", "orientation_error_deg",
            ])
            for s in camera_samples:
                aligned_pos = r @ s["est_pos"] + t
                aligned_quat = quat_multiply(q_r, s["est_quat"])
                pos_err = float(np.linalg.norm(aligned_pos - s["gt_pos"]))
                ang_err = quat_angle_deg(aligned_quat, s["gt_quat"])
                pos_errors.append(pos_err)
                ang_errors.append(ang_err)
                writer.writerow([
                    f"{s['t']:.9f}",
                    *[f"{v:.6f}" for v in s["gt_pos"]],
                    *[f"{v:.6f}" for v in s["gt_quat"]],
                    *[f"{v:.6f}" for v in s["est_pos"]],
                    *[f"{v:.6f}" for v in aligned_pos],
                    *[f"{v:.6f}" for v in aligned_quat],
                    f"{pos_err:.6f}", f"{ang_err:.6f}",
                ])

        if pos_errors:
            pos_errors_np = np.array(pos_errors)
            ang_errors_np = np.array(ang_errors)
            self.get_logger().info(
                f"Camera ATE (rigid-aligned, {len(pos_errors)} samples): "
                f"RMSE={np.sqrt((pos_errors_np ** 2).mean()):.4f} m "
                f"mean={pos_errors_np.mean():.4f} m median={np.median(pos_errors_np):.4f} m "
                f"max={pos_errors_np.max():.4f} m | "
                f"orientation RMSE={np.sqrt((ang_errors_np ** 2).mean()):.3f} deg")
        self.get_logger().info(f"Wrote {path}")
        return r, t, scale_diag

    def _write_human_csvs(self, human_samples, r, t):
        errors_path = os.path.join(self.output_dir, "human_pose_errors.csv")
        summary_path = os.path.join(self.output_dir, "human_track_summary.csv")

        by_stamp = defaultdict(list)
        for h in human_samples:
            aligned = r @ h["centroid_raw"] + t
            by_stamp[h["t"]].append({**h, "centroid_aligned": aligned})

        track_match_counts = defaultdict(lambda: defaultdict(int))
        track_errors = defaultdict(list)

        with open(errors_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "keyframe_id", "track_id", "class_id", "n_points",
                "est_x", "est_y", "est_z", "matched_gt_id",
                "gt_x", "gt_y", "gt_z", "position_error_m", "n_gt_visible",
            ])
            for stamp in sorted(by_stamp.keys()):
                entries = by_stamp[stamp]
                gt_snapshot = entries[0]["gt_snapshot"]
                gt_ids = list(gt_snapshot.keys())
                n_gt = len(gt_ids)

                matched = {}
                if n_gt > 0:
                    cost = np.zeros((len(entries), n_gt))
                    for i, e in enumerate(entries):
                        for j, gid in enumerate(gt_ids):
                            cost[i, j] = np.linalg.norm(
                                e["centroid_aligned"] - gt_snapshot[gid])
                    row_ind, col_ind = hungarian_min_cost_assignment(cost)
                    for row, col in zip(row_ind, col_ind):
                        if cost[row, col] <= self.max_match_distance_m:
                            matched[row] = (gt_ids[col], cost[row, col])

                for i, e in enumerate(entries):
                    if i in matched:
                        gid, err = matched[i]
                        gt_pos = gt_snapshot[gid]
                        track_match_counts[e["track_id"]][gid] += 1
                        track_errors[e["track_id"]].append(err)
                        writer.writerow([
                            f"{stamp:.9f}", e["keyframe_id"], e["track_id"], e["class_id"],
                            e["n_points"], *[f"{v:.6f}" for v in e["centroid_aligned"]],
                            gid, *[f"{v:.6f}" for v in gt_pos], f"{err:.6f}", n_gt,
                        ])
                    else:
                        writer.writerow([
                            f"{stamp:.9f}", e["keyframe_id"], e["track_id"], e["class_id"],
                            e["n_points"], *[f"{v:.6f}" for v in e["centroid_aligned"]],
                            -1, "", "", "", "", n_gt,
                        ])

        with open(summary_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "track_id", "dominant_gt_id", "n_frames_matched_dominant",
                "n_frames_matched_any", "match_purity", "mean_position_error_m",
                "rmse_position_error_m",
            ])
            for track_id in sorted(track_match_counts.keys()):
                gid_counts = track_match_counts[track_id]
                dominant_gid, dominant_count = max(gid_counts.items(), key=lambda kv: kv[1])
                n_any = sum(gid_counts.values())
                errs = np.array(track_errors[track_id])
                purity = dominant_count / n_any if n_any else float("nan")
                writer.writerow([
                    track_id, dominant_gid, dominant_count, n_any, f"{purity:.4f}",
                    f"{errs.mean():.6f}", f"{math.sqrt((errs ** 2).mean()):.6f}",
                ])

        n_matched_rows = sum(len(v) for v in track_errors.values())
        self.get_logger().info(
            f"Human evaluation: {len(human_samples)} raw estimate rows, "
            f"{len(track_match_counts)} distinct track_ids, "
            f"{n_matched_rows} matched to a GT human (max_match_distance_m="
            f"{self.max_match_distance_m}).")
        self.get_logger().info(f"Wrote {errors_path}")
        self.get_logger().info(f"Wrote {summary_path}")


def main():
    rclpy.init()
    node = UHumans2Evaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
