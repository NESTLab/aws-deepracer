# AWS DeepRacer (NESTLab fork)
## Overview
This repository provides instructions and packages to setup a new AWS DeepRacer such that you can use it to run ROS 2 nodes and &mdash; optionally &mdash; ARGoS controllers. The target user for this repo is anyone with access to NESTLab resources, such as an AWS DeepRacer robot, the NESTLab WiFi (can be changed to a different network) and so on.

Specific changes have been implemented to the code base originally provided by [aws-deepracer](https://github.com/aws-deepracer/aws-deepracer). **Please follow the instructions provided here to properly setup the AWS DeepRacer for ROS 2 and ARGoS support before attempting the instructions provided in the [original README](original_readme.md).** The instructions here will allow you to run your ROS 2 nodes without needing `root` access (unlike the original code).

## Setup requirements
- A new AWS DeepRacer device - *if required, you can do a factory reset according to [these steps](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-ubuntu-update.html#deepracer-ubuntu-update-preparation)*
- Access to the [NESTLab wiki](https://www.nestlab.net/wiki/nestlab)
- Access to the NESTLab local WiFi network - *this can be substituted with another WiFi network you have access to*
- A Ubuntu machine - *the instructions here have not been tested with other OS so proceed at your own risk*

## Setup instructions
To setup the AWS DeepRacer, there are 3 main steps: [Preliminary setup](#preliminary-setup), [ROS 2 setup](#ros-2-setup), and [ARGoS setup](#argos-setup). You *must* complete at least the first 2 steps to run any ROS 2 nodes on the vehicle. The third step is optional if you do not need ARGoS support.

### Preliminary setup
If the vehicle has an IP address set up already and you can log in with SSH, you can skip this subsection and proceed to the [ROS 2 setup](#ros-2-setup). Otherwise, if the vehicle is new or if it has been reset, proceed with the following instructions. These self-sufficient instructions are extracted from this [guide](/media/aws_deepracer_and_sensor_guide.pdf).

1. Before turning on the vehicle, ensure that all sensors are plugged in to the USB ports (more details in the provided guide). **At the very least, a camera should be plugged into a USB port on the compute module and be kept plugged in at all times.** This is to prevent the pre-installed `deepracer-core` service from logging large amounts of missing sensor errors, eventually using up the device storage.
2. Turn on the compute module. Neither the motor battery nor the compute battery need to be charged; you can just use the wall adapter to power the compute module directly.
3. Let the compute module load for around 3 minutes, or until the power indicator light turns blue. Then, connect a micro USB cable from your machine to the vehicle.
4. On your machine, ensure that it's not connected to the internet. Then go to the `deepracer.aws` URL on your web browser. A warning page about an unsafe connection may show up, but you can ignore and proceed (there should be a button for you to accept the risk and proceed).
5. A new page will show up displaying the device console, asking for a password. If this is a new vehicle, the password is found underneath the vehicle. Otherwise, you can find the password from the [AWS DeepRacers page from the NESTLab Wiki](https://www.nestlab.net/wiki/robots/awsracers).
6. Once logged in, connect to the NESTLab WiFi. You can find the network credentials on the [General Lab Setup and Passwords page from the NESTLab Wiki](https://www.nestlab.net/wiki/robots/laptopsetup). *Typically, the NESTLab WiFi network doesn't have internet access, but occassionally it may be plugged into the school's wall Ethernet port to obtain internet access. If it is, see the info panel below.*
>:information_source: If you're connected to a network with internet access, at some point a page may pop up and inform you that a new software update is available. If that happens, update the vehicle. Otherwise simply proceed (it sometimes takes awhile to show up).
7. If the WiFi connection is successful, you should be provided with an IP address of the vehicle. Make sure to record that IP address somewhere to facilitate SSH access; this is the IP address that you will use to work with the vehicle.
8. Click on the button with the vehicle's IP address to get to the vehicle control page. You will be prompted for the password again, which can be found underneath the vehicle (if it is a new vehicle). *Alternatively you can visit the vehicle control page by visiting the vehicle IP address in another browser window.*
9. Once logged in, click on `Settings` on the sidebar, then reset the device console password. You should reset it to the password provided on the [AWS DeepRacers page from the NESTLab Wiki](https://www.nestlab.net/wiki/robots/awsracers). Once successful, you will be logged out after 30 seconds.

<p align="center">
<img src="/media/vehicle_control_settings.png" width="607" height="542" >
</p>

10. Log back in with the new password, then proceed to the `Settings` page and enable SSH access. Set the password to be the same password as the device console password.
>:warning: *Do not proceed with the next subsection until you perform the software update.* You can do this by first changing the vehicle's network to one with internet access (see Step 6.). If the notification to update doesn't appear after around 10 minutes after connecting to the internet, manually update the vehicle by following the instructions [here](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-troubleshooting-manual-update-device.html). The software version &mdash; after update &mdash; should at least 2.0.383.0 (would have started out as 2.0.113.0).

### ROS 2 setup
For this subsection, you need to connect to the vehicle via SSH.

1. Once connected, clone this repository with its submodules on the vehicle in the `deepracer_nav2_ws` directory:
    ```
    mkdir -p /home/deepracer/deepracer_nav2_ws
    cd /home/deepracer/deepracer_nav2_ws
    git clone --recurse-submodules https://github.com/NESTLab/aws-deepracer.git
    ```
2. Run the initialization script:
    ```
    cd /home/deepracer/deepracer_nav2_ws/aws-deepracer
    ./initialize_deepracer.sh
    ```
>:information_source: After running the script, the `deepracer-core` service is disabled, so you will not be able to access the vehicle control page any longer. To access it again, see the [instructions below](#calibrating-the-vehicle-ie-how-to-reactivate-the-vehicle-control-page).
3. Reboot the vehicle:
    ```
    sudo reboot
    ```

### ARGoS setup
Skip this subsection if you intend to only use the AWS DeepRacers with ROS 2 without ARGoS support. If you do, before proceeding, ensure that the [argos3-deepracer plugin](https://github.com/NESTLab/argos3-deepracer) is installed on your machine.

Connect to the vehicle via SSH, then run the `initialize_argos.sh` script. This will install ARGoS and the `argos3-deepracer` plugin on the vehicle. **Do not run this script on your machine.**
```
cd /home/deepracer/deepracer_nav2_ws/aws-deepracer
./initialize_argos.sh
```

## Usage instructions

### Build ROS 2 packages
In the [ROS 2 setup section](#ros-2-setup), the packages have been built once already. If you want to rebuild the packages, simply go to the `aws-deepracer` directory and do `colcon build`:
```
cd /home/deepracer/deepracer_nav2_ws/aws-deepracer
colcon build
```

### Calibrating the vehicle (*i.e.*, how to reactivate the AWS console/vehicle control page?)
To calibrate the vehicle and/or access other vehicle settings, you will need to do it through a web browser using the vehicle's IP address (like in the [Preliminary setup](#preliminary-setup)). Because the service that enables the browser interface will have been disabled in the [ROS 2 setup](#ros-2-setup), you will need to start it on the vehicle:
```
sudo systemctl start deepracer-core.service
```

Then, connect a micro-USB cable between your machine and the vehicle's compute module. Type in the vehicle's IP address in your web browser and the vehicle control page should show.

Once you're done, stop the service on the vehicle:
```
sudo systemctl stop deepracer-core.service
```

### Running ROS 2 nodes for autonomous navigation (w/o deep RL)
#### Recommended method
Convenience scripts have been installed in which only the core driver nodes are run, _i.e.,_ IMU, drivetrain, camera, and LIDAR (only those with Evo addons). All you need to do is to first execute the following.
```
# Execute command to run all the driver nodes
start-ros-service <DESIRED-ROS-DOMAIN-ID> <EVO-OR-BASE-BOOL> # this runs the start-ros service
```
This takes care of the vehicle's core components, which means you only need to run remaining ROS nodes required for the robot to actuate or sense. For example, running a launch file that contains _only_ the `amcl` and `teleop` nodes will provide localization and tele-operation capabilities.

Once you're done (or if you want to reset the drivers), you can stop the service.
```
stop-ros-service
```

#### Legacy method
You can follow the instructions provided by the AWS in the `deepracer_*` folders (of this repository), but run the nodes *without* `root` access. A Hello World launch file that you can start off with is the `deepracer.launch.py` launch file:
```
ros2 launch deepracer_bringup deepracer.launch.py
```

You can run other ROS 2 nodes that you may need (*e.g.*, `amcl`, `teb_local_planner`, etc.) or your own nodes as you would any ROS 2 node and they should work as expected.

Technically, you should be able to run nodes that use deep reinforcement learning models to drive your vehicle. However, if you use the nodes provided by AWS they may not work out-of-the-box without `root` access.

### Running ROS 2 nodes for autonomous navigation (w/ deep RL)
If you need to implement deep reinforcement learning models using the AWS-provided nodes (*i.e.*, via the vehicle control page) to drive your vehicle, you can just restart the `deepracer-core` service.
```
sudo systemctl enable deepracer-core.service # for launching at startup
sudo systemctl start deepracer-core.service
```
This service runs everything as `root`, which means that any additional ROS 2 nodes launched by non-`root` users *will not be able to see the topics by the nodes launched by the service*. The only way to make it work is to then run all ROS 2 nodes as `root`, which is a [bad idea&trade;](https://answers.ros.org/question/11300/running-ros-as-a-root-user/). The service actually runs code from the [`aws-deepracer-launcher`](https://github.com/aws-deepracer/aws-deepracer-launcher) repository, so you can find more information there.

### Running the AWS DeepRacer using ARGoS controllers
Here we use the example controller `deepracer_hello_world` provided in the [`argos3-deepracer`](https://github.com/NESTLab/argos3-deepracer) repository, where it has a corresponding configuration file `hello_world_config.argos` and the controller is given a `drhw` ID in the file. Then, to run the controller simply execute the following.
```
# Execute command to run all the driver nodes (the ARGoS controller requires the ROS drivers)
start-ros-service <DESIRED-ROS-DOMAIN-ID> <EVO-OR-BASE-BOOL> # this runs the start-ros service

# Run the controller
cd ${HOME}/argos3-deepracer
build/testing/deepracer_hello_world -c src/testing/hello_world_config.argos -i drhw
```
When you're done with the controller, you can kill the ROS nodes.
```
stop-ros-service
```

>:information_source: A useful tool to debug the ROS nodes is through `systemctl` and `journalctl` commands (since the nodes are started using a service). For example, you can check if the service is active by doing `systemctl status --user start-ros` or `journalctl --user-unit start-ros`.

## Notes
- After running `initialize_deepracer.sh`, all but one LED indicator light will be turned off on the compute module. Any instructions from the official AWS channels regarding those LEDs are no longer applicable, so just keep that in mind. The lights have been turned off because the `deepracer-core` service has been disabled, for reasons listed in the [Usage instructions](#usage-instructions).
- When following the instructions on the official AWS DeepRacer repositories, *avoid running the code as `root`*. The instructions above have been created to ensure that no `root` access is needed ever to run the ROS 2 nodes. If you have permission issues running the robot's sensor/actuator ROS 2 nodes (`servo_node`, `imu_node`, `camera_node` etc.) without `root` access, please create an issue.
- While there are ROS 2 packages catered for the AWS DeepRacer provided in the `/opt/aws/deepracer` directory, some of them are outdated so it's easier to clone the appropriate versions and build them from source. This allows the user (you) to maintain the code base as desired.

## Troubleshooting
- IMU node doesn't start properly: do `i2cdetect -y -r 1` to see if the address 68 is detected. If not, restart the vehicle.