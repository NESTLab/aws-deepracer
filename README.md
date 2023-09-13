# AWS DeepRacer (NESTLab Fork)
## Overview
This repository provides instructions and packages to setup a new AWS DeepRacer such that you can use it to run ROS 2 nodes and &mdash; optionally &mdash; ARGoS controllers. The target user for this repo is anyone with access to NESTLab resources, such as an AWS DeepRacer robot, the NESTLab WiFi (can be changed to a different network) and so on.

Specific changes have been implemented to the code base originally provided by [aws-deepracer](https://github.com/aws-deepracer/aws-deepracer). **Please follow the instructions provided here to properly setup the AWS DeepRacer for ROS 2 and ARGoS support before attempting the instructions provided in the [original README](original_readme.md).** The instructions here will allow you to run your ROS 2 nodes without needing `root` access (unlike the original code).

## Setup Requirements
- A new AWS DeepRacer device - *if required, you can do a factory reset according to [these steps](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-ubuntu-update.html#deepracer-ubuntu-update-preparation)*
- Access to the [NESTLab wiki](https://www.nestlab.net/wiki/nestlab)
- Access to the NESTLab local WiFi network - *this can be substituted with another WiFi network you have access to*
- A Ubuntu machine - *the instructions here have not been tested with other OS so proceed at your own risk*

## Instructions
To setup the AWS DeepRacer, there are 3 main steps: [Preliminary Setup](#preliminary-setup), [ROS 2 Setup](#ros-2-setup), and [ARGoS setup](#argos-setup). You *must* complete at least the first 2 steps to run any ROS 2 nodes on the vehicle. The third step is optional if you do not need ARGoS support.

### Preliminary Setup
If the vehicle has an IP address set up already and you can log in with SSH, you can skip this subsection and proceed to the [ROS 2 Setup](#ros-2-setup). Otherwise, if the vehicle is new or if it had to be factory reset, proceed with the following instructions. The steps here are self-sufficient, but for additional details see this [PDF](/media/aws_deepracer_and_sensor_guide.pdf) where the instructions are extracted from.

1. Before turning on the vehicle, ensure that all sensors are plugged in to the USB ports (according to the provided PDF). **At least one camera should be plugged into one of the USB port and be kept plugged in at all times.** This is to prevent a pre-installed service on the DeepRacer from logging large amounts of missing sensor errors, eventually using up the device storage.
2. Turn on the compute module. Neither the vehicle nor the compute battery need to be charged and plugged in; you can just use the wall adapter to power the compute module directly.
3. Let the compute module load for around 3 minutes, or until the power indicator light turns blue. Then, connect a micro USB cable to the DeepRacer.
4. On your machine, ensure that it's not connected to the internet. Then go to the `deepracer.aws` URL on your web browser. A page warning of unsafe connection may show up, but you can ignore and proceed (there should be a button for you to accept the risk and proceed).
5. A new page will show up displaying the device console, asking for a password. If this is a new DeepRacer, the password is found underneath the vehicle. Otherwise, you can find the password from the [AWS DeepRacers page from the NESTLab Wiki](https://www.nestlab.net/wiki/robots/awsracers).
6. Once logged in, connect to the NESTLab WiFi. You can find the network credentials on the [General Lab Setup and Passwords page from the NESTLab Wiki](https://www.nestlab.net/wiki/robots/laptopsetup). *Typically, the NESTLab WiFi network doesn't have internet access, but occassionally it may be plugged into the school's wall Ethernet port to obtain internet access. If it is, see the warning panel below.*
>:warning: If you're connected to a network with internet access, at some point a page may pop up and inform you that a new software update is available. If that happens, update the vehicle. Otherwise simply proceed (it sometimes takes awhile to show up).
7. If the WiFi connection is successful, you should be provided with an IP address of the vehicle. Make sure to record that IP address somewhere to facilitate SSH access; this is the IP address that you will use to work with the vehicle moving forward.
8. Click on the button with the vehicle's IP address to get to the vehicle control. You will be prompted for the device console password again, which can be found underneath the vehicle. *Alternatively you can visit the vehicle control page by visiting the vehicle IP address in another browser window.*
9. Once logged in, click on `Settings` on the sidebar, then reset the device console password. You should reset it to the password provided on the [AWS DeepRacers page from the NESTLab Wiki](https://www.nestlab.net/wiki/robots/awsracers). Once successful, you will be logged out after 30 seconds.
10. Log back in with the new password, then proceed to the `Settings` page and enable SSH access. Set the password to be the same password as the device console password.
>:warning: *Do not proceed with the next subsection until you perform the software update.* You can do this by first changing the vehicle's network to one with internet access (see Step 6.). If the notification to update doesn't appear after around 10 minutes after connecting to the internet, manually update the vehicle by following the instructions [here](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-troubleshooting-manual-update-device.html). The software version &mdash; after update &mdash; should at least 2.0.383.0 (would have started out as 2.0.113.0).

### ROS 2 Setup
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
3. Manually add the `ROS_DOMAIN_ID` to `.bashrc`. This ID should match at most the last 2 digits of the vehicle's IP address (leave any leading 0s out).
    ```bash
    # in the vehicle's .bashrc
    export ROS_DOMAIN_ID=<HOST-ID> # e.g., if IP=192.168.1.103, then replace <HOST-ID> with 3; if 192.168.1.23, then replace with 23
    ```
4. Reboot:
    ```
    sudo reboot
    ```

### ARGoS Setup
You can skip this subsection if you intend to only use the AWS DeepRacers with ROS 2 without ARGoS support. Before proceeding, ensure that the [argos3-deepracer plugin is installed (WORK-IN-PROGRESS)](https://github.com/NESTLab/argos3-deepracer) on your machine.

Connect to the vehicle via SSH, then run the `initialize_argos.sh` script. This will install ARGoS and the `argos3-deepracer` plugin on the vehicle. **Do not run this script on your machine.**
```
cd /home/deepracer/
./initialize_argos.sh
```

## Notes
- When following the instructions on the official AWS DeepRacer repositories, *avoid running the code as `root`*. The instructions above have been created to ensure that no `root` access is needed ever to run the ROS 2 nodes. If you have permission issues running the ROS 2 nodes (`servo_node`, `imu_node`, `camera_node` etc.) without `root` access, please open an issue.
- It is known that the AWS DeepRacers come with some ROS 2 packages in the `/opt/aws/deepracer` directory already. However, some of them are outdated so it's easier to clone the latest appropriate versions and build them from source.
