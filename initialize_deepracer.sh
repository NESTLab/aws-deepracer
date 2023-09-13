#!/bin/bash

# Setup color output macro
loginfo() {
    GREEN="\033[0;32m"
    GREENB="\033[1;32m"
    YELLOW="\033[0;33m"
    YELLOWB="\033[1;33m"
    RESET="\033[0m"

    echo -e "${!1}APPTAINER BUILD:    ${2}${RESET}"
}

### Terminate on errors ###
set -e


### Modify timeout for password requirement ###
loginfo "YELLOWB" "Updating password timeout the 'deepracer-core' service..."
pushd /etc/sudoers.d
echo "Defaults timestamp_timeout=60" | sudo tee -a deepracer


### Stop and disable the `deepracer-core` service ###
loginfo "YELLOWB" "Disabling the 'deepracer-core' service..."
sudo systemctl stop deepracer-core.service
sudo systemctl disable deepracer-core.service


### Update the system ###
loginfo "YELLOWB" "Updating and upgrading packages..."
sudo apt update -y

sudo apt-mark hold rsyslog # the `config` package by Intel (`dpkg -l` description: "deepcam configuration install package.") causes conflicts with the `rsyslog` package. This is because the `config` packages also writes to `/etc/logrotate.d/rsyslog`.

for (( i=0; i<3; i++ )) # (need to upgrade and fix broken multiple times to ensure that dependencies are all up-to-date)
do
    sudo apt upgrade -qy
    yes "" | sudo apt --fix-broken install # yes allows for acceptance of default values
done

sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-imu-tools -qy # dependencies


### Install the AWS DeepRacer ROS 2 packages ###
loginfo "YELLOWB" "Installing ROS 2 packages for the AWS DeepRacer..."
cd /home/deepracer/deepracer_nav2_ws/aws-deepracer/deepracer_nodes && ./install_dependencies.sh

# Install the IMU I2C driver
cd /home/deepracer/deepracer_nav2_ws/aws-deepracer/deepracer_nodes/BMI160-i2c
pip install .

# Install the IMU package
source /opt/ros/foxy/setup.bash
cd /home/deepracer/deepracer_nav2_ws/aws-deepracer/deepracer_nodes/larsll-deepracer-imu-pkg
sudo rosdep init
rosdep update
rosdep install -i --from-path . --rosdistro foxy -y

# Install the ROS 2 packages
cd /home/deepracer/deepracer_nav2_ws/aws-deepracer/deepracer_nodes
rosws update
cd ~/deepracer_nav2_ws/aws-deepracer/ && \
colcon build --packages-select deepracer_interfaces_pkg deepracer_bringup cmdvel_to_servo_pkg enable_deepracer_nav_pkg rf2o_laser_odometry rplidar_ros camera_pkg servo_pkg imu_pkg


### Set up userspace access to camera ###
loginfo "YELLOWB" "Setting up access to camera..."
sudo usermod -aG video deepracer # the `video` group already has access, so just add the `deepracer` user


### Set up userspace access to servo ###
loginfo "YELLOWB" "Setting up access to servo..."

# Create new group and add users
sudo groupadd pwmgroup
sudo usermod -aG pwmgroup deepracer
sudo usermod -aG pwmgroup root

# Create shell script to modify permissions
cat <<EOF | sudo tee /usr/local/bin/udev-pwm-permissions.sh
#!/bin/bash

GPIO_PIN=436 # provided by pegatron as found https://github.com/aws-deepracer/aws-deepracer-servo-pkg/blob/f232684f84313dd3f2c740836493ccd049be6686/servo_pkg/include/servo_pkg/servo_mgr.hpp#L47

# Workaround to 'manually' create the \`pwm*\` directories in /sys/class/pwm/pwmchip0; there are 5 channels in total
for (( i=0; i<5; i++ ))
do
    echo \${i} > /sys/class/pwm/pwmchip0/export
done

# Workaround to 'manually' create the gpio pin directory in /sys/class/gpio/
echo \${GPIO_PIN} > /sys/class/gpio/export

# Modify group
chown -R :pwmgroup /sys/class/pwm/pwmchip0/ # additional front slash indicates modification of the contents of pwmchip0
chown -R :pwmgroup /sys/class/gpio/export
chown -R :pwmgroup /sys/class/gpio/gpio\${GPIO_PIN}/

# Modify permissions for owner and group
chmod -R 770 /sys/class/pwm/pwmchip0/
chmod -R 220 /sys/class/gpio/export
chmod -R 770 /sys/class/gpio/gpio\${GPIO_PIN}/
EOF
sudo chmod +x /usr/local/bin/udev-pwm-permissions.sh

# Create udev rule to run shell script
cat <<EOF | sudo tee /etc/udev/rules.d/99-custom-pwm.rules
SUBSYSTEM=="pwm", KERNEL=="pwmchip0", ACTION=="add", PROGRAM="/usr/local/bin/udev-pwm-permissions.sh"
EOF


### Set up userspace access to IMU ###
loginfo "YELLOWB" "Setting up access to IMU..."
sudo usermod -aG i2c deepracer # the i2c group already has access, so just add the `deepracer` user


### Adding source files in .bashrc ###
loginfo "YELLOWB" "Adding source files to .bashrc..."

echo "" >> ~/.bashrc
echo "# ROS source files" >> ~/.bashrc
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source /home/deepracer/deepracer_nav2_ws/aws-deepracer/install/setup.bash" >> ~/.bashrc


### Completion ###
loginfo "GREENB" "Initialization complete!"