# ORB-SLAM3-STEREO-FIXED

This repository is a modified version of [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)  


--- 

## Modification
- Succesfully tested in **Ubuntu 20.04** and **ROS2 Foxy**(with OpenCV 4.2.0)
- Update from C++11 to C++14
- Fixed unexpected <span style="color:red">error</span> when start **STEREO** mode with **Rectified** camera type  

## How to build
Clone the repository:
```
git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3
```

Install same required dependencies as original version. Then,  
Execute:
```
cd ORB_SLAM3
chmod +x build.sh
./build.sh
source colcon_ws/install/local_setup.bash
```
This will create **libORB_SLAM3.so**  at *lib* folder and the executables in *Examples* folder.

need to unzip Vocabulary/ORBvoc.txt first!!!

```
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/RealSense_D435i.yaml
```
```
./Examples/RGB-D/rgbd_realsense_D435i Vocabulary/ORBvoc.txt ./Examples/RGB-D/RealSense_D435i.yaml
```
```
ros2 run orbslam3 stereo-inertial Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/RealSense_D435i.yaml False False
```
```
ros2 run orbslam3 rgbd Vocabulary/ORBvoc.txt ./Examples/RGB-D/RealSense_D435i.yaml
```

