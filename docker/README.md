# 🚀 Run vS-Graphs using Docker

This guide walks you through building and running `vS-Graphs` using **Docker**.

---

## 📁 Step I. Prepare Required Files

Before building the Docker image, ensure the following:

- All required **Semantic Segmentation Models** must be placed in the `seg_models/` directory.
- These models will be copied into the appropriate subdirectory of the `scene_segment_ros` package during the Docker build process.

---

## 🛠️ Step II. Build the Docker Image

To build the Docker image, run the following command in the root of the repository:

```bash
docker build \
  --build-arg ssh_prv_key="$(cat ~/.ssh/id_rsa)" \
  --build-arg ssh_pub_key="$(cat ~/.ssh/id_rsa.pub)" \
  -t vsgraphs .
```
