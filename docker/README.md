# Using Docker  

Note: Before building the image, make sure to have all the semantic segmentation models in the `seg_models` folder. These will be moved to the relevant location in the `scene_segment_ros` package.  

To build, use:  
```
docker build --build-arg ssh_prv_key="$(cat ~/.ssh/id_rsa)" --build-arg ssh_pub_key="$(cat ~/.ssh/id_rsa.pub)" -t vsgraphs .
```