#!/usr/bin/env bash
set -euo pipefail

# Clone/update the companion repositories used by this workspace.
# Run this script from anywhere inside a clone of visual_sgraphs.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_SRC="$(cd "${SCRIPT_DIR}/.." && pwd)"

GIT_PROTOCOL="${GIT_PROTOCOL:-ssh}"

if [[ "${GIT_PROTOCOL}" != "ssh" && "${GIT_PROTOCOL}" != "https" ]]; then
  echo "GIT_PROTOCOL must be either 'ssh' or 'https'." >&2
  exit 1
fi

repo_url() {
  local owner="$1"
  local repo="$2"

  if [[ "${GIT_PROTOCOL}" == "https" ]]; then
    printf 'https://github.com/%s/%s.git' "${owner}" "${repo}"
  else
    printf 'git@github.com:%s/%s.git' "${owner}" "${repo}"
  fi
}

clone_or_update() {
  local path="$1"
  local owner="$2"
  local repo="$3"
  local branch="$4"
  local target="${WORKSPACE_SRC}/${path}"
  local url
  url="$(repo_url "${owner}" "${repo}")"

  if [[ -d "${target}/.git" ]]; then
    echo "Updating ${path} (${branch})"
    git -C "${target}" fetch origin "${branch}"
    git -C "${target}" checkout "${branch}"
    git -C "${target}" pull --ff-only origin "${branch}"
    return
  fi

  if [[ -e "${target}" ]]; then
    echo "Skipping ${path}: path exists but is not a git repository." >&2
    return
  fi

  echo "Cloning ${path} (${branch})"
  mkdir -p "$(dirname "${target}")"
  git clone --branch "${branch}" "${url}" "${target}"
}

echo "Workspace source directory: ${WORKSPACE_SRC}"
echo "Git protocol: ${GIT_PROTOCOL}"

clone_or_update "dynamic_keypoint_3d_lifter" "snt-arg" "dynamic_keypoint_3d_lifter" "master"
clone_or_update "dynamic_keypoint_tracker" "snt-arg" "dynamic_keypoint_tracker" "master"
clone_or_update "keyframe_depth_estimator" "snt-arg" "keyframe_depth_estimator" "master"
clone_or_update "keyframe_depth_validator" "snt-arg" "keyframe_depth_validator" "master"
clone_or_update "object_motion_estimator" "snt-arg" "object_motion_estimator" "master"
clone_or_update "realsense-ros" "IntelRealSense" "realsense-ros" "ros2-master"
clone_or_update "scene_segment_ros" "snt-arg" "scene_segment_ros" "dynamic"
clone_or_update "situational_graphs_msgs" "snt-arg" "situational_graphs_msgs" "main"
clone_or_update "voxblox_ros2_minimal" "snt-arg" "voxblox_ros2_minimal" "master"
clone_or_update "bin/dynamic_s_graphs" "snt-arg" "dynamic_s_graphs" "master"
clone_or_update "bin/object_tracker_3d_ros" "snt-arg" "object_tracker_3d_ros" "master"
clone_or_update "scene_segment_ros/src/eomt" "tue-mps" "eomt" "master"

echo "Done."
