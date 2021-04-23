# AWS DeepRacer DeepDriver Project
The Deepdriver project is a modification of the [Follow the Leader(FTL) sample project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project). The obejct detection node was modified to detect stop signs and traffic lights. This allows the car to act based on basic road rules. Explore the DeepDriver project by cloning the [deepdriver project](https://github.com/jochem725/deepdriver).

DeepDriver uses nodes from the AWS DeepRacer core application and adds new nodes to process and act upon traffic sign and traffic light input. This application is built to work alongside the AWS DeepRacer core application so that we can run both of the applications simultaneously.

* **DeepRacer Core Packages used without modification**
    * camera_pkg
    * deepracer_navigation_pkg
    * deepracer_systems_pkg
    * device_info_pkg
    * i2c_pkg
    * inference_pkg
    * model_optimizer_pkg
    * sensor_fusion_pkg
    * servo_pkg
    * status_led_pkg
    * usb_monitor_pkg
* **DeepRacer Core Packages modified to support DeepDriver**
    * webserver_pkg
    * ctrl_pkg
    * deepracer_interfaces_pkg
* **DeepDriver functionality specific packages**
    * object_detection_pkg
    * deepdriver_navigation_pkg
    * traffic_sign_pkg

## Hardware Setup

The deepdriver project is built to work on **AWS DeepRacer** with a single camera attached to it. Optionally, you can also connect an **Intel Neural Compute Stick 2** to the USB slot at the rear end of the car as depicted to improve the inference performance.

## Main Components

There are seven packages (ROS Nodes) that are of importance for the DeepDriver project.
 
1. [Object Detection Package](https://github.com/jochem725/deepdriver/tree/main/deepdriver_ws/object_detection_pkg) - Package responsible to detect traffic signs and traffic lights from the camera sensor images. It publishes a message with the images and the detected bounding boxes that can be used for further post-processing in different ROS nodes.

2. [Traffic Sign Package](https://github.com/jochem725/deepdriver/tree/main/deepdriver_ws/traffic_sign_pkg) - Package responsible for processing the output from the object detection node. It post-processes the detected objects from the object detection node in order to provide instructions the car can use in the navigation node.

3. [DeepDriver Navigation Package](https://github.com/jochem725/deepdriver/tree/main/deepdriver_ws/deepdriver_navigation_pkg) - Package responsible for collecting the from traffic sign postprocessing and mapping it to the servo message with throttle and steering angle values.

4. [DeepDriver Launcher Package](https://github.com/jochem725/deepdriver/tree/main/deepdriver_ws/deepdriver_launcher) - The DeepRacer Interfaces ROS package is a foundational package that creates the custom service and message types that are used in the core AWS DeepRacer application, but has been modified to support DeepDriver.

5. [Control Package](https://github.com/jochem725/deepdriver/tree/main/deepdriver_ws/ctrl_pkg) - Package extended from AWS DeepRacer core application and responsible for creating main node with services exposed to be used by webserver backend API calls. This manages the mode of the car: manual, autonomous, calibration or deepdriver.

6. [DeepRacer Interfaces Package](https://github.com/jochem725/deepdriver/tree/main/deepdriver_ws/deepracer_interfaces_pkg) - The DeepRacer Interfaces ROS package is a foundational package that creates the custom service and message types that are used in the DeepDriver project.

7. [Webserver Package](https://github.com/jochem725/deepdriver/tree/main/deepdriver_ws/webserver_pkg) - Package extended from AWS DeepRacer core application and responsible for creating a collection of FLASK APIs that are called from the front end. These APIs call the required ROS services and return the result to the front end required for DeepDriver to interact with the device console.


## DeepDriver mode:
DeepDriver introduces a new mode (deepdriver mode) of operation in the AWS DeepRacer device apart from the existing modes of operation(autonomous mode, calibration mode and manual mode). More details about the existing modes of operation in the AWS DeepRacer device is found [here](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/modes-of-operation.md).

In DeepDriver mode the DeepRacer devices takes the camera image input from the front facing camera connected to the car and runs it through the machine learning model to identify stop signs and traffic lights. OpenCV is used to post-process detected traffic lights using the bounding box and HSV color segmentation. Each step involves a pipeline of a series of ROS messages published/subscribed at various nodes.

### Inference (Decision)

The inference step is handled by the Object Detection ROS package that creates the object_detection_node responsible for collecting sensor data (camera images) from sensor_fusion_pkg and running object detection. The Traffic Sign ROS package is responsible for translating the detected objects into actions the car can use to act. The traffic_sign_node translates the detected object into a TrafficMsg object which contains the detected traffic signs and traffic lights. Specifically for traffic lights it contains the detected color (red, yellor or green) which can be used to start/stop the car based on the rules of the road.

**Possible improvements:**
- Add speed sign detection
- Improve the robustness of the traffic light color detection
- Lane detection
- Replace the object detection model with a model trained specifically for traffic

### Action (Navigation)
The DeepDriver Navigation ROS package creates the deepdriver_navigation_node which decides the action / controller message based in the input from the traffic_sign_node. 

Currently the car only drives forward, or stops. 

**Possible improvements:**
- Using speed sign input to change the speed of the car
- Using signs for navigation similar to [the DeepRacer Offroad sample project](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project)
- Only drive if the car is on a road

## Summary

The Deepdriver project is a modification of the [Follow the Leader(FTL) sample project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project), which leverages most of the concepts used in the AWS DeepRacer application.
You can learn more about the AWS DeepRacer core application [here](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).
* AWS DeepRacer device modes of operation: [https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/modes-of-operation.md](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/modes-of-operation.md).
