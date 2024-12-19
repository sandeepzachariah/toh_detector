# TOH_DETECTOR
## Overview
Package that implements the coupled pointcloud-based and image-based
pipeline for detecting 'Tree-of-Heaven'.

## Install and Build
Clone the repository in the src folder of catkin_ws and build the package using catkin build or catkin make.
```bash
git clone https://github.com/sandeepzachariah/toh_detector.git
cd ../..
catkin build
```

## Usage
Run the rosbag files, tf_publisher and velodyne pointcloud converter.
```bash
roslaunch toh_detector launch_rosbag.launch
```

To run the pipeline:
```bash
roslaunch toh_detector pipeline.launch
```

## Testing on custom datasets
Add your dataset path in the `config/` folder and update the `scripts/run_rosbag_files.py` file.

## Code Structure
```bash
toh_detector
    ├── CMakeLists.txt
    ├── config
    │   ├── rosbag_files_pc.txt
    │   ├── rosbag_files_test_1.txt
    │   ├── rosbag_files_test_2.txt
    │   ├── rosbag_files_test_3.txt
    │   ├── rosbag_files_test_4.txt
    │   ├── rosbag_files_test_5.txt
    │   ├── rosbag_files_test_6.txt
    │   ├── rosbag_files_test_7.txt
    │   ├── rosbag_files.txt
    │   └── settings.yaml
    ├── data
    │   └── image.jpg
    ├── launch
    │   ├── image_to_pc.launch
    │   ├── launch_rosbag.launch
    │   ├── rosbag_play.launch
    │   └── tf_publish.launch
    ├── package.xml
    ├── README.md
    ├── scripts
    │   ├── data_association.py
    │   ├── run_rosbag_files.py
    │   ├── scratch_pad.ipynb
    │   └── tf_from_urdf.py
    ├── temp
    │   ├── image_all_pts.png
    │   ├── image.png
    │   └── pointcloud.txt
    └── urdf
        ├── mapping_system_pretty.urdf
        ├── mapping_system.urdf
        └── sample_urdf.urdf

```