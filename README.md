Capturing multi-view image datasets are relatively simple, all you need is a camera to capture videos/images and a structure from motion (SfM) framework to extract the camera poses for each image. However, capturing concentric views, which is ideal for 3D reconstruction models, can be troublesome with a camera. Furthermore, utilising SfM software produces camera poses with arbritrary values, which may not be ideal if the scene needs to be reconstructed in known units.

This repo offers a set of scripts that can utilise any ROS supported robot, with an associated MoveIt package and attached physical camera, to capture concentric views around an object. At the end of image capturing, a generated transform/COLMAP files ready for use in 3D reconstruction software. We experimented utilising our dataset on Neural Radiance Fields (NeRF) and 3D Gaussian Splatting (3DGS) models, which can be installed using [NeRFStudio](https://docs.nerf.studio/).

Our pipeline supports any robot that has a MoveIt package and valid URDF joints/links that accurately map the robot from its base to the camera's optical centre. The setup can support single or multiple robots with serial or parallel movements. We also support adding a turntable to your setup to facilitate capturing in relation to a rotating object. Our view capturing pipeline is highly customisable, allowing for image generation for a wide range of scenes and robotic configurations.

To run this framework, enter the following command: **python view_capture.py --config path\\to\\config\\file**

![Showcase of duel robot setup](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExdm5paXh0Z3lwb29pMGtyOWE1c2hzanFjZzV2emU4cGttazNleWEwdyZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/FE4JcF5CeUyELIEfiE/giphy-downsized-large.gif)

# How to Install

## 1) Install Python 3.8.10

Python 3.8.10 can be installed from the Python website: *https://www.python.org/downloads/release/python-3810/*

While our configuration will probably work on other versions, this is the version that we employed for controlling our robots

## 2) Install Requirements

The required Python packages can be installed via requirements.txt file. The command is as follows: *pip install -r /path/to/requirements.txt*

## 3) Install Colmap 

Install Colmap from: https://colmap.github.io/install.html

## Extras

MoveIt will need to be installed for your robot configuration and executed at run time. For information of how to set this up, please follow the MoveIt documentation: *http://moveit.ros.org/*

For details about how we implemented MoveIt for our UR5 setup, please visit *https://github.com/Lewis-Stuart-11/Dual-UR5-Config*

# Software Pipeline

Firstly, the user must provide information about the current scene to be captured. This includes the size of the object being captured, the position of that object in relation to the origin of the robot WCS, and the size of the radius to set the camera poses from the centre of the object. Using this information, a set of camera poses are generated using a simple sphere generation algorithm that are pointed towards the centre of the scene.  The sphere algorithm uses rings (horizontal 'cuts') and sectors (vertical 'cuts') to define a series of points along the surface of the sphere, the distance from each point to the centre is set as the capture radius parameter. These generated points are the camera poses that will be recorded in the transform file. If a turntable is not used, then these camera poses are added as the camera poses that the robots will traverse to. 

If a turntable is used, then the midpoint between each of the robots is calculated, which is the area of the scene that has the most overlap between all robots. Each camera pose is then rotated to a position on a fixed plane between the turntable and the midpoint. The angle between the new rotated pose and original pose is the angle that will be set as to the rotation of the turntable. Essentialy, rather than positioning the robots around the object in the scene, the turntable rotates the objects relative to the robots.

Each pose is added to a group based on the 'sector' that that pose was part of (in the sphere generation), and the pipeline iterates through each group during traversal. Each point in the group is then assigned to a robot. To calculate this, each robot is compared to that point based on the reach of the robot and the distance of the robot base to that point. The robot with best reachability is assigned to that point, and that point is then added to that robot's pose queue. Next, each point in each robot queue is iterated through in parallel, allowing multiple robots to traverse to points in parallel. If these points cannot be reached in parallel, then each robot will traverse to that point serially. Once a robot has reached a new point, then an image is captured from the camera attached to that robot, and the camera pose is recorded for that image. This software can also captured depth maps (if the cameras support this) and segment out the centre object (if the setup supports this). 

Once all points have been traversed, then the camera poses are then optimised using COLMAP. Firstly, a set of image pairs are calculated. Essentially, each image position is traversed, and if an image is adjacent to this image by x amount (the 'img\_pair\_range' argument), then these images are added as an image pair. Next, features are extracted from each of the image using COLMAP. These features are then matched based on the image pairs (features from images are only matched with features from other images that are an image pair). Then, each matched feature is triangulated and projected into 3D to produce a point cloud based on the recorded camera poses. Finally, bundle adjustment is then performed to refine each camera pose to become more accurate based on the pointcloud. This process of point triangulation and bundle adjustment is then repeated several times, each using the refined camera poses for triangulation. Finally, once the new camera poses have been calculated, the new camera poses are then scaled back to the original distances from the object centre and repositioned around the origin.

Each of the images and camera poses are saved into a new directory in the log path. This includes: images, transforms and COLMAP files. Different transforms are saved for the different types of images (e.g. RGB, RGB-D, Segmented RGB, etc..) and the original camera pose transforms are also saved. 

![Pipeline flowchart](https://i.imgur.com/0rjAGcZ.png)

# Functionality

Our framework is extremely customisable, offering many different arguments that can be altered to support a wide range of robot settings and scene types.

## Experiment Settings

These arguments are used for controlling where data is stored, loaded and visualsied

| Argument             | Default Value  | Description |
| :---                 |    :----:      |          ---: |
| config\_path          |    -           |  Path to config file containing a list of configuration arguments (**required**) |
| log\_path             |    -           |  Path to directory to store log information (images and transforms) after running an experiment |
| experiment_name      |    -           |  Name of this current experiment (**required**) |
| continue\_experiment  |    False       |  Continue a saved experiment with the same name as the ’experiment name’ argument. This means that already traversed positions will be skipped, and failed positions will be retried |
| replace\_stored\_experiment |  False | Set to remove all stored information about a previous experiment with the same name as the ’experiment name’ argument and restart the experiment. If not set, then a new experiment will be created with the same 'experiment_name' but incremented.

## Robot Settings

These arguments are used for controlling how the robots are controlled, which robots should traverse to which points and extra movement settings

| Argument             | Default Value  | Description |
| :---                 |    :----:      |          ---: |
| robot\_settings\_path |    -          | Path to JSON file containing all information required for running robots and managing cameras (**required**) |
| robot\_handler\_type |      moveit          | Class for controlling the robots (currently only supports MoveIt) |
| parallelise\_robots |    True         | Set to allow for parallel robot path planning i(if multiple robots are supported). Otherwise, serial path planning will be used  |
| avoid\_self\_capture\_paths | False | Sometimes, the robot will move to a position where its own joints are blocking the camera. If set to True, paths will be prioritsed that avoid this. |
| priorised\_robot |           -         | If multiple robots are supported, then prioritise one robot over the other when assigning new positions |
| planning\_time |  2.0  | The maximum number of seconds to take for each robot path planning attempt before failing |
| num\_planning\_attempts |  3  | The number of attempts to move to specific position before failing and beginning next position movement attempt |
| retry\_failed\_pos | False | Set to reattempt to move to previously failed positions after movement to all positions have been attempted. | 
| discard\_robot\_movement | False | Skips movement of the robot and assumes the move failed (use for debugging) |

## Scene Properties

These arguments are used for handling objects in a scene, including the main object to capture as well as extra objects in the scene to avoid

| Argument             | Default Value  | Description |
| :---                 |    :----:      |          ---: |
| scene\_handler\_type |    moveit   | Class for handling the scene (currently only supports MoveIt) |
| main\_obj\_position  | [0,0,0]        | The (x,y,z) position of the object relative to the centre of the WCS|
| main\_obj\_size      | [1.0, 1.0, 1.0]| The size of the main object (x,y,z) to capture. Used for avoiding collisions in the robot path planning|
| scene\_objs		| - | Path to a JSON file that loads different objects into the path planning environment.  Each specified object must declare a valid shape type (cube, sphere, mesh) and dimensions |
| scale\_factor | 1.0 | The value to scale the transform positions |

Scene objects can be added to MoveIt directly via their interface. However, we also offer a way to configure a scene automatically via a scene JSON file. This JSON file has the following format:
* *Name*: The unique name of the object to add
* *Type*: The type of object (either box, sphere or mesh)
* *Position*: Where in the scene to position that object
* *Size*: The size of the object 
* *Attach*: Whether to attach this object to the end-effector of the robots
* *Mesh\_file\_name*: the file path to the mesh object (typically .dae)

For context, this is the scene file that we used for our experiments:

```
[
	{
		"name": "floor",
		"type":"box",
		"position":[0.0,0,0.0],
		"size": [5,5,5],
		"attach": false
	},
	{
		"name": "left_light",
		"type":"mesh",
		"position":[0.0,1.35,0.0],
		"size": [1,1,1],
		"mesh_file_name": "/home/psxls7/catkin_ws/src/robotic_view_capture/pedestal_gazebo/meshes/light_left.dae",
		"attach": false
	},
	{
		"name": "right_light",
		"type":"mesh",
		"position":[0.0,-1.35,0.0],
		"size": [1,1,1],
		"mesh_file_name": "/home/psxls7/catkin_ws/src/robotic_view_capture/pedestal_gazebo/meshes/light_right.dae",
		"attach": false
	}
]

```

## Camera Handling

These arguments are used for managing the cameras, including capturing information such as depth and segmenting the background of images

| Argument             | Default Value  | Description |
| :---                 |    :----:      |          ---: |
| camera\_handler\_type | ros | The class for handling the attached cameras (currently supports ROS or Realsense cameras) |
| capture\_depth | False | Set to capture depth values from the camera depth topic (only set if a depth camera is being used)|
| crop\_width | - | The number of pixels to crop the image width. Default does not crop the image |
| crop\_height | - | The number of pixels to crop the image height. Default does not crop the image |
| segment\_method | - | Method used to segment the foreground from the background (currently supports segmentation via depth or background thresholding) |
| discard\_img\_capturing | False | If set, does not attempt to capture images or use any camera settings (use for debugging) |

## View Generation

These arguments are used for generating views around the object in the scene

| Argument             | Default Value  | Description |
| :---                 |    :----:      |          ---: |
| capture\_radius  | - | The distance from the center of the main object to all of the generated camera positions (**Required**) |
| rings | 7 | The number of rings used for generating the camera positions using a basic sphere algorithm |
| sectors | 14 | The number of sectors used for generating the camera positions using a basic sphere algorithm |
| aabb | [] | An axis-aligned bounding box, with (x,y,z) values between 0 and 1, that is used to filter points that are not positioned inside of this volume during view generation |
| visualise | False | Set to generate 3D scatter diagrams for traversed camera positions |
| save\_fig | False | Set to save the generated diagrams to a PNG image |

## Dataset Management

These arguments are used for determining how data is stored as a dataset, as well as what transforms should be generated

| Argument             | Default Value  | Description |
| :---                 |    :----:      |          ---: |
| calculate\_transforms\_from\_pos | False | Set to generate transforms based on the camera pose, rather than using the ROS transform (*set if using turntable*) |
| test\_incrementation | 8 | The number of training images per test image (0 will not assign any images as test images in the dataset) | 


## Refining Transform Settings

These arguments are used for refining the camera poses after image capturing using SfM

| Argument             | Default Value  | Description |
| :---                 |    :----:      |          ---: |
| sfm\_package | COLMAP | The SfM package to use for adjusting the transforms (currently onyl supports COLMAP) |
| refine\_transforms | True | If set, sfm techniques will be used to optimise the given transforms | 
| img\_pair\_path | - | The path to an img pair txt file that can be used in feature matching (only set if this file has been calculated before) |
| use\_relative\_img\_pairs | True | Set to match features of images relative to eachother |
| img\_pair\_range | 3 | The range of adjacent images to include for each image in relative image pair generation |
| undistort\_imgs | True | Undistorts the images after camera pose adjustment |
| use\_mask\_in\_sfm | True | Only extract features after applying the mask (recommended for turntable experiments with background) |
| extend\_reconstruction\_per\_robot | False | Rather than refining cameras captured from all robots together, refine camera poses from one robot then 'register' the images captured from other robots based on the images and camera poses from the first robot. Only use if relative robot positions are not calibrated correctly |

## Turntable Properties

These arguments are used for handling turntable movements

| Argument             | Default Value  | Description |
| :---                 |    :----:      |          ---: |
| use\_turntable | False | Set to use a turntable to rotate the object rather than moving the arm to positions around the object (drastically changes point and transform calculations) |
| turntable\_connection\_port | - | The port to connect the turntable |
| turntable\_handler | Zaber | The class used for handling turntable movements |


# Output Format

At the start of running the experiment, a new directory will be created in the location of your log directory with the name of the experiment. From there, all data captured/calculated during the running of the experiment will be contained to within this directory. 

Here is an example of the output of running an experiment.

## Images

```
+-- images 
|   +-- rgb 
|   +-- depth
|   +-- mask
|   +-- segmented  
|   +-- undistorted
|   +-- undistorted_segmented
```

This subdirectory contains all the different types of images that can be captured during an experiment:
* RGB: all images directly captured via the defined cameras in the setup. 
* Depth: depth masks captured using the cameras (if depth is supported and set). 
* Mask: segmentation masks calculated by the Segment Background classes. 
* Segmented: RGBA images where the masks have been applied directly to the RGB images
* Undistorted: all RGB images after going through the COLMAP undistort function
* Undistorted_segmented: the undistorted images after being segmented by the masks

All images are named after the points that they were captured from, including if they are part of the training or evaluation set (e.g. point 0 will have associated images titled 0000\_train.png if it is part of the training set, and 0000\_eval if it is part of the eval set)

## Transforms

```
+-- transforms
|   +-- original
|   +-- adjusted
|   +-- depth
|   +-- depth_mask
|   +-- mask
|   +-- segmented
|   +-- undistorted
```

This subdirectory contains all the different transforms that are derived during the experiment:
* Original: all transforms that were captured for each RGB images 
* Adjusted: transforms that were refined after bundle adjustment
* Depth: each transform also contains a depth map path
* Depth_mask: each transform also contains a depth map path as well as a mask path
* Mask: each transform also contains a mask path 
* Segmented: each transform has the image path to the segmented images (rather than RGB images)
* Undistorted: contains COLMAP point cloud data generated after image undistortion

Each transform subdirectory, apart from the undistorted transforms, are stored as JSON files:
* transforms.json: contains all transforms that were calculated during the experiment
* transforms_train.json: contains all transforms that are part of the training set
* transforms_test.json: contains all transforms that are part of the test set

The undistorted directory contains binary COLMAP point cloud data. The relative path to the transforms from the main directory is: ```transforms/sparse/0```. It is important to note that only undistorted or undistorted segmented images can be trained using these transforms. The relative image path is ```../../images/undistorted```

## Colmap

```
+-- colmap
|   +-- reconstructed
|   +-- adjusted
|   +-- dense
|   +-- colmap.db
|   +-- imageNamePairs.txt
```

The subdirectory contains all the different data captured during the COLMAP bundle adjustment process:
* Reconstructed: point cloud generated from the original transforms
* Adjusted: point cloud and camera poses after bundle adjustment 
* Dense: all data after performing the undistortion functionality
* Colmap.db: the sql database containing all information for running that COLMAP experiment 
* ImageNamePairs.txt: a txt of all image names stored in pairs for feature matching

This directory will not be made if transform refinement is not set for this experiment.

# Adding your own Robots

This framework has been designed to work with any robot configuration (either in a simulation or real life), as long as the following criteria are met:
* ROS- the robots supports ROS and are activated.
* MoveIt- a MoveIt package has been generated for the robot setup that has a valid controller. 
* URDF- the robots have a valid URDF file with transfroms mapping from the base of each robot to the camera lens.
* Robot Settings file- a robot settings file has been provided, that defines the parameters of the setup (e.g. the transforms to the camera, MoveIt controller name, camera configurations, etc...)

Configuring these different components are explained below.

## ROS, MoveIt and URDF
Firstly, ROS must be correctly installed such that the robots can be correctly managed. For more information, please visit: *https://www.ros.org/*. This framework was only tested on ROS Noetic.

Next, a valid MoveIt package must be generated which can perform valid path planning for each robot. The MoveIt package must have a valid kinematic chain from the base of each robot, to the end effector (camera lens). This framework supports parallel path planning if this is correctly configured in the conguration file. For more information, please visit: *http://moveit.ros.org/*.

To ensure that the robot is correctly mapped in MoveIt, a valid URDF file must be generated that documents each joint in the robot setup, including the base of each robot and camera lens. This information is important as it is how MoveIt can reposition each robot so that the camera is pointed towards the main scene. Furthermore, this is how the camera transforms are correctly calculated for each robot.

It is also crucial that the mapped robot joints are measured correctly. If the transforms cannot accurately map the camera lens in 3D space, then the bundle adjustment process at the end of the capturing process will fail. Hence, we recommend performing hand-to-eye calibration to accurately map the base of each robot to the camera optical centre.

The robot config files that we used for creating this pipeline can be installed from [this repo](https://github.com/Lewis-Stuart-11/Dual-UR5-Config)

## Robot Settings File
In order to run this pipeline on a ROS robot setup, a few parameters must be defined in a JSON file, that defines the key values required to facilitate valid image capturing. These provide information to the piepline about what robots are part of the scene, what cameras are avaliable and how these robots can move to new positions in the scene to facilitate image capturing.

Robot management is split into two classes:
* Controllers- the move group that controls a set of robots. If configured correctly in Moveit, these robots can be executed in parallel
* Robots- an individual robot in a move group
It is important that only robots that are part of a specific move group are added to that move group controller. It is recommended that multiple robots are added to a move group to support parallel image capturing.

The JSON file accepts a list of controllers, with each controller having a list of robots that define how to reposition the camera to the correct positions and the parameters of the camera. The parameters for a controller are as follows:
* move\_group: the name of the move group that contains a list of robots
* robots: the list of robots 

The parameters for a robot are as follows:
* end\_effector\_transform: the name of the end-effector joint on the robot (this is typicall the optical centre of the camera and will point towards the centre of the scene)
* base\_transform: base of the robot (root joint)
* camera_transform: the name of the camera joint that will be used for calculating the camera pose transform.
* reach: the maximum reach of the robot in the same units as positions in the URDF file. This is used for calculating what points should be assigned to each robot
* camera\_properties\_file\_path: path to the file containing the camera properties for that camera
* camera\_topic: name of the camera ROS topic (if using ROS camera topics)
* camera\_serial\_no: serial number for that camera (if required by specific camera API)
* camera\_fps: the FPS settings to use for that camera (if required by specific camera API)

For context, here is a JSON file that we utilised for capturing of our robot setup:
```
[
	{
		"move_group": "both_arms_manipulator",
		"robots":[
			{
				"end_effector_transform": "arm1_camera_controller",
				"base_transform": "arm1_base_link",
				"reach": 0.80, 
				"camera_transform": "arm1_camera_predicted_colour_lens",
				"camera_properties_file_path": "/camera_properties/cameras_robot1.json",
				"camera_serial_no": "827112071788",
				"camera_fps": 30
			},
			{
				"end_effector_transform": "arm2_camera_controller",
				"base_transform": "arm2_base_link",
				"reach": 0.80,
				"camera_transform": "arm2_camera_predicted_colour_lens",
				"camera_properties_file_path": "camera_properties/cameras_robot2.json",
				"camera_serial_no": "215222073199",
				"camera_fps": 6
			}
		]

	}
]
```

# Adding your own Camera

Currently, this pipeline supports ROS, GPhoto and Realsense cameras. Each of these are configured using individual classes inside of the camera_handler file. To add your own, create a new class that derives from the CameraHandler abstract class and add your own the methods for capturing new images. A DepthCameraHandler abstract class is also avaliable for cameras that support depth capturing. Each new class should be registered to the factory function

For the captured camera poses to be correctly refined using bundle adjustment, valid intrinsic camera paramters must be provided for each camera. These parameters are stored in a json file that defines each attribute of the camera. These paramters can be determined using either COLMAP or OpenCV's camera calibration. 

It is important to ensure that these intrinsic parameters are accurate, otherwise the bundle adjustment process will not converge correctly.

For context, here is a JSON file that we utilised for one of our Realsense D435i cameras:

```
{
    "k1": 0.12079316788730118,
    "k2": -0.26522267758028134,
    "k3": 0,
    "k4": 0,
    "p1": -0.0015222477960390676,
    "p2": 0.0005044829242852293,
    "w": 1920.0,
    "h": 1080.0,
    "fl_x": 1364.9868552946584,
    "fl_y": 1364.3790676406236,
    "cx": 973.5804734015358,
    "cy": 542.0366067547649,
    "camera_angle_x": 1.2258792464280706,
    "camera_angle_y": 0.7537339920842024,
}
```

# Adding your own Turntable

Currently, our pipeline supports Xaber turntables. To add your own, create a new class that derives from the TurntableHandler abstract class and add your own methods for rotating the turntable to specific positions. 

It is important to ensure that the turntable is correctly positioned in your MoveIt scene, ideally at coordinates (0,0,0), otherwise the bundle adjustment process will not converge correctly.

# Complimentary Material/ Citation

This repo contains materials that were used in the following paper:

[TO ADD CITATION]

In this paper, we utilised this repository to capture concentric views around a set of different plants. We then utilised NeRF and 3DGS to reconstruct a variety of different Wheat Plants and evaluated the accuracy of the reconstruction. We found that these models perform far more accurate reconstruction when compared to other standard models such as SfM.

We also offer the following repos that can be utilised alongside this view capturing software package:

1) [3D Plant View Synthesis](https://github.com/Lewis-Stuart-11/3D-Plant-View-Synthesis): Contains scripts for running data captured using the software in this repo

2) [UR5 Duel Config](https://github.com/Lewis-Stuart-11/Dual-UR5-Config):  MoveIt files for controlling a single/multiple UR5 arms. Can be executed using real robot arms or in simulation

3) [3DGS-to-PC](https://github.com/Lewis-Stuart-11/3DGS-to-PC): Converts a 3D Gaussian Splat to a Point Cloud. Useful for converting a captured dataset into a point cloud after gaussian splatting training

Finally, we captured an extensive dataset of different wheat plants using this repo [TO ADD LINK].

