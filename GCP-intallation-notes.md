```bash
gcloud compute instances create ros-desktop \
  --zone=europe-west4-a \
  --machine-type=n2-standard-8 \
  --image=ubuntu-pro-2004-focal-v20260317 \
  --image-project=ubuntu-os-pro-cloud \
  --boot-disk-size=200GB
```

```bash

gcloud compute ssh ros-desktop --zone=europe-west4-a
```

Within this ssh connection to install the chrome remote desktop

```bash
sudo apt update && sudo apt upgrade -y

#sudo apt install --assume-yes ubuntu-desktop

```

Now we need to hack a bit for the chrome remote desktop.

```bash
# Create the user manually before CRD tries to
sudo adduser --system --group --force-badname _crd_network

# Verify it was created
id _crd_network

# Download the remote desktop software
wget https://dl.google.com/linux/direct/chrome-remote-desktop_current_amd64.deb

sudo apt-get install --assume-yes ./chrome-remote-desktop_current_amd64.deb
# this will fail, so ally the fix:
sudo sed -i '108s/.*/    id "$CRD_NETWORK_USER" \&\& true || adduser --system "$CRD_NETWORK_USER"/' /var/lib/dpkg/info/chrome-remote-desktop.postinst
sudo dpkg --configure -a

# Now lets continue

sudo DEBIAN_FRONTEND=noninteractive \
  apt install --assume-yes xfce4 desktop-base dbus-x11 xscreensaver
#  apt install --assume-yes xfce4 ubuntu-desktop dbus-x11 xscreensaver

sudo bash -c 'echo "exec /etc/X11/Xsession /usr/bin/xfce4-session" > /etc/chrome-remote-desktop-session'

sudo systemctl disable lightdm.service
```

# Install ROS 1

```bash
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/refs/heads/master/install_ros_noetic.sh
chmod 755 ./install\_ros_noetic.sh
bash ./install\_ros\_noetic.sh

sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
ros-noetic-rosserial-python ros-noetic-rosserial-client \
ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

Now restart the machiine

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
