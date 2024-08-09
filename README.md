# Simulation Package
This package is based loosely off of the tutorial:
https://github.com/gazebosim/ros_gz_project_template.git

- [Launch Files](README.md#launch-files)
- [Useful Links](README.md#useful-links)
- [Installation](README.md#install-gazebo)
- [URDF SolidWorks Tool](README.md#urdf-solidworks-tool)
- [Heightmap Procedure](README.md#heightmap-procedure)
- [Troubleshooting](README.md#troubleshooting)

## Launch Files

The following is a list of the launch files and their purposes
- [aruco_marker.launch.py](./launch/aruco_marker.launch.py) : This file spawns aruco markers when run
- [camera_test.launch.py](./launch/camera_test.launch.py) : This file spawns cameras when run
- [deadblow_mallet.launch.py](./launch/deadblow_mallet.launch.py) : This file spawn in the orange autonomy mallet
- [lidar_test.launch.py](./launch/lidar_test.launch.py) : This file spawns in a lidar that publishes a pointcloud
- [sensor_test.launch.py](./launch/sensor_test.launch.py) : This file launches a world with all test sensors and bridges their outputs to ros
- [thermal_test.launch.py](./launch/thermal_test.launch.py) : This file launches a thermal camera when run --NOT YET FUNCTIONAL--
- [wanderer_II.launch.py](./launch/wanderer_II.launch.py) : This file spawns a urdf model of Wanderer II with fixed bougie joints
- [water_bottle.launch.py](./launch/water_bottle.launch.py) : This file spawns in a waterbottle for the autonomy mission. The color is configurable on launch
- [world.launch.py](./launch/world.launch.py) : This file launches a world set in it's contents. Used for testing, will be replaced with more concrete examples later.

## Useful Links

The following links are useful resources
- [Fortress Website](https://gazebosim.org/docs/fortress)
- [SDF Format Guide](http://sdformat.org/spec?ver=1.8&elem=sdf)
- [USGS Download Instructions](https://apps.nationalmap.gov/uget-instructions/index.html)
- [Known Gazebo Plugins](src/README.md)


## Install Gazebo

Run the following steps to uninstall Gazebo-11 and install Gazebo Fortress

```
sudo apt-get remove gazebo-11

sudo apt-get update
sudo apt-get install lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
sudo apt-get install ros-humble-ros-gz
sudo apt-get install ros-humble-joint-state-publisher
sudo apt-get install ros-humble-joint-state-publisher-gui
```

## Install Heightmap Tools

Run the following commands to install the tools necessary for getting heightmaps

### UGET
```
sudo apt-get install uget
```

Uget is used to download the .las file from the USGS website

### Cloud Compate

```
sudo apt-get install cloudcompare
```
 Cloud Compare is used to convert from a .las file to .png and .stl

 ### Meshlab

 ```
sudo apt-get install meshlab
```
Meshlab is used to determine the size of heightmaps, as well as fix any issues with scaling


## URDF SolidWorks Tool

The following steps will describe how to run the URDF SolidWorks Exporter Tool

1. Download the URDF tool from [https://wiki.ros.org/sw_urdf_exporter](https://wiki.ros.org/sw_urdf_exporter)
2. Run the installer
3. Open the assembly you want to export
4. Remove any unnecessary parts and/or details
5. Go into reference geometry
6. For every moving joint, add an axis about which the joint will move
7. For every seperate link (combined part), add a reference coordinate frame at this axis  
   a. Most joints rotate around the Z axis  
   b. X usually points towards the front of the rover  
   c. Give each axis and coordinate frame a descriptive name for later
8. Select the converter from the tool menu
9. Highlight all components that will be part of the base link
10. Select a reference frame and name the link, choose it's joint type
11. Add all children necessary and repeat the process using the axis and coordinate frames made before
12. Export the model and all the meshes
13. Modify the package to work with ROS2 and Gazebo Fortress

## Heightmap Procedure

The following steps will describe how to download and extract the necessary data for a heightmap

1. Follow the [USGS Download Tutorial](https://apps.nationalmap.gov/uget-instructions/index.html)
2. Open the .las file in Cloud Compare
3. Run the SOR (Statistical Outlier Removal) filter on the pointcloud
4. Choose which Scalar field to color it with, and which layer should be the active one
5. Under tools -> Scalar Fields, set the current scalar field to the RGB value
6. Under tools -> projection, rastorize the point cloud
7. Make sure the width/height is a power of 2
8. Export the entire thing as a mesh
9. Export the height into a .tif file
10. Export the RGB into another .tif file
11. Open the mesh in meshlab and record the dimensions using the bounding box
12. Convert the two .tif files into pngs using online converters
13. Use the RGB one as the texture file, the height one as the geometry file, and the size from meshlab as the size of the map


## Troubleshooting 

Below are some common Errors I have have encounterd

--------------------------------------------------------------------------------------------

[Err] [SystemPaths.cc:378] Unable to find file with URI [FILE_PATH]

This may be one of 2 things. Check if the file path is correct, if it isn’t fixed and rebuild. If the file path is correct run the following command:
```
printenv IGN_GAZEBO_RESOURCE_PATH
```
If the result is nothing, go to `hooks/PKG_NAME.sh.in` and `hooks/PKG_NAME.dsv.in` to check if the environment variable being set is `IGN_GAZEBO_RESOURCE_PATH`

If the result is wrong, go to `hooks/PKG_NAME.sh.in` and `hooks/PKG_NAME.dsv.in` to modify the paths to be the correct values

--------------------------------------------------------------------------------------------

[GUI] [Err] [Image.cc:140] Unknown image format[FILE_PATH]

This is probably because of the file type. Even though Gazebo-Classic supports the .tif (.tiff) format, Gazebo Fortress does not and instead wants you to use .png. There are .tif to .png converters available online that work.

--------------------------------------------------------------------------------------------

[GUI] [Err] [Ogre2Heightmap.cc:137] Heightmap final sampling must satisfy 2^n.

This is a really weird error, and I don’t like it. Basically, the heightmap must have a width / length that is a power of 2. If this isn’t the case Gazebo gets sad. To fix this, make sure the number of pixel width is equal to 2n + 1.

--------------------------------------------------------------------------------------------

File [FILE_PATH/__default__] resolved to path [FILE_PATH/__default__] but the path does not exist

This is probably because your material is messed up. Check to make sure you have both a diffuse and normal layer.

--------------------------------------------------------------------------------------------

Could not load mesh resource [file_path]

If this occurs in RViz, check the mesh file paths in your model. If they are wrong, that’s probably the error. Keep in mind, the urdf tool for SolidWorks is not smart enough to add escape characters to spaces if they are in the file name. This may make some parts that otherwise open correctly, break.
