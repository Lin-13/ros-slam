# Slam With Orbbec Pro

## Brief

Rum ORB-SLAM3 with Ros2.

you need run this Cmake Project in C++17.


Include three packages:

**slam** : use open source **ORB-SLAM3** to build the mono or RGBD system.

you can run it with:

```bash
ros2 run slam mono-slam
```

```bash
ros2 run slam ros-mono
```

```bash
ros2 run slam ros-rgbd
```

mono-slam read single frame in device using OpenCV itself, and others wait for ros2 topic bgr_image_topic , depth_image_topic(if need) and frame_info_topic(not significant).


**frame_publisher**: use **OrbbecSDK** or **OpenCV** to read RGBD frame or Mono frame to publish in some topic.

you can run with:

```bash
ros2 run frame_publisher publisher #Using OpenCV
```

```bash
ros2 run frame_publisher publisher_obsdk #Using OrbbecSDK
```

Similarly, you can only get depth frame by the **publisher_obsdk.** 

**img_py (TODO):**

read frame by python to get further appliance (Maybe NeRF)

you can use this package :

```bash
ros2 run img_py listener
```

```bash
ros2 run img_py talker
```

it need cv2 (torch for test) and other ros2 python package.

this package use Opencv-Pyhon to capture video frame. You can not run talker with frame_publisher package at same time.

you should change the source code :

```python
sys.path.append("~/anaconda3/envs/py_env/lib/python3.8/site-packages")
```

to make it run with torch or other package in Anaconda, if you need.

Ros2 version : **Foxy**

## Dependencies

**Ros**

> rclcpp
>
> std_msge
>
> cv_bridge *(you need to complie it in source code to solve error and replace the file in ros2)*
>
> sensor_msgs

ORB-SLAM3

Orbbec-SDK

Python

> cv2
>
> torch *(for test & TODO)*
>
> `<default package in ros2>`
