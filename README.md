# Technical-workshop-ROS
# ROS 2 Humble Setup on Ubuntu 22.04 in VirtualBox with Gazebo and Git

## Step 1: Install VirtualBox

1. Go to: [https://www.virtualbox.org/wiki/Downloads](https://www.virtualbox.org/wiki/Downloads)
2. Choose the version for your host operating system (Windows/Linux/macOS).
3. Download and run the installer.
4. Follow the default installation steps.
5. After installation, launch Oracle VM VirtualBox.

---

## Step 2: Download Ubuntu 22.04 ISO

1. Go to: [https://releases.ubuntu.com/jammy/](https://releases.ubuntu.com/jammy/)
2. Download the Ubuntu 22.04 Desktop ISO (file size around 3.5 GB).
   - Example filename: `ubuntu-22.04.4-desktop-amd64.iso`

---

## Step 3: Create a New Virtual Machine in VirtualBox

1. Open VirtualBox, click on "New".
2. Enter the following details:
   - Name: Ubuntu 22.04  
   - Type: Linux  
   - Version: Ubuntu (64-bit)
3. Click Next.

### Configure Hardware

- **Memory (RAM):** Allocate at least 8000 MB (8 GB).
- **Hard Disk:**
  - Choose "Create a virtual hard disk now"
  - Choose "VDI (VirtualBox Disk Image)"
  - Select "Dynamically allocated"
  - Set size to at least 25 GB

Click **Create**.

---

## Step 4: Install Ubuntu 22.04 in VirtualBox

1. Select your new VM and click **Start**.
2. When prompted, select the Ubuntu ISO you downloaded.
3. Follow the installation steps:
   - Choose "Try or Install Ubuntu"
   - Select language and keyboard layout
   - Choose "Normal Installation"
   - Enable updates during install
   - Select "Erase disk and install Ubuntu" (affects only the virtual disk)
4. Set your username and password.
5. Wait for the installation to complete.
6. Restart the VM and remove the ISO when prompted.

---

## Step 5: Update Ubuntu and Install Git

After logging in to Ubuntu, open a terminal and run:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install git -y
```

To verify Git installation:

```bash
git --version
```

---

## Step 6: Install ROS 2 Humble on Ubuntu 22.04

### 1. Setup Locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### 2. Add ROS 2 Repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
```
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo apt install /tmp/ros2-apt-source.deb
```
### 3. Install ROS 2 Humble Desktop

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

### 4. Source the ROS 2 Environment

To automatically source the ROS 2 setup script:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

To source it manually for the current terminal session:

```bash
source /opt/ros/humble/setup.bash
```

### 5. Install ROS Tools (Optional)

```bash
sudo apt install python3-colcon-common-extensions -y
```

---

## Step 7: Install Gazebo Classic (Version 11)

Gazebo Classic 11 is compatible with ROS 2 Humble and available in Ubuntu 22.04.

### Install Gazebo

```bash
sudo apt update
sudo apt install gazebo11 libgazebo11-dev -y
```

### Test Gazebo Installation

Run:

```bash
gazebo
```

If Gazebo opens with the default world, installation is successful.

### Source Gazebo Environment in `.bashrc`

To ensure Gazebo is available in every session:

```bash
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
source ~/.bashrc
```

### Install Gazebo ROS Packages

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control -y
```

---

## Step 8: Clone the ROS Workshop Repository and Build

### Create a Directory and Clone the Repository

```bash
mkdir ros-technical-workshop
cd ros-technical-workshop
git clone --recurse-submodules https://github.com/Srivenkateshwar09/Technical-workshop-ROS.git src
```

### Build the Code

```bash
cd ..
colcon build
```
To source the model of turtlebot
```
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```
To run the turtlbot simulation
```
ros2 launch workshop turtlebot_launch.py
```

To run the node 
```
cd src/workshop/src
python3 run controller.py
```

---


