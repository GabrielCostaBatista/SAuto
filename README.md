# Table of Contents

- [1. Installing Multipass and ROS Noetic on macOS](#1-installing-multipass-and-ros-noetic-on-macos)
- [2. Setting Up Foxglove Studio with ROS Noetic](#2-setting-up-foxglove-studio-with-ros-noetic)
  - [2.1. Install the Foxglove Bridge in ROS VM](#21-install-the-foxglove-bridge-in-ros-vm)
  - [2.2. Launch the Foxglove Bridge](#22-launch-the-foxglove-bridge)
  - [2.3. Install Foxglove Studio on macOS](#23-install-foxglove-studio-on-macos)
  - [2.4. Connect Foxglove Studio to Your ROS VM](#24-connect-foxglove-studio-to-your-ros-vm)
  - [2.5. Troubleshooting](#25-troubleshooting)

---


# Installing Multipass and ROS Noetic on macOS

This guide explains how to install Multipass on macOS and set up ROS Noetic inside an Ubuntu 20.04 Multipass VM, following the ROS installation tutorial for Ubuntu 20.04 from https://varhowto.com/install-ros-noetic-ubuntu-20-04.

---

## 1. Install Multipass on macOS

Multipass allows you to run Ubuntu VMs easily on macOS.

### Steps:

1. Download and install Multipass from the official site: https://multipass.run/

2. Verify installation by opening Terminal and running:  
   `multipass version`

3. Launch an Ubuntu 20.04 VM (name it `ros1-vm`):  
   `multipass launch 20.04 --name ros1-vm --memory 4G --disk 40G`

4. Access the VM shell:  
   `multipass shell ros1-vm`

---

## 2. Install ROS Noetic inside the Ubuntu 20.04 VM

Follow the steps below inside the VM shell (`ros1-vm`):

### Step 1:  Set up ROS Noetic repo for Ubuntu 20.04 
`echo "deb http://packages.ros.org/ros/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros-focal.list`

### Step 2: Set up your keys  
`sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`

### Step 3: Update package index  
`sudo apt update`

### Step 4: Install ROS Noetic Desktop Full  
`sudo apt install ros-noetic-desktop-full -y`

### Step 5: Environment setup  
Add the ROS environment variables to your bash session:  
`source /opt/ros/noetic/setup.bash`  
`echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`
`source ~/.bashrc`

### Step 6: Verify Noetic installation
`roscd`
This should take you to `/opt/ros/noetic`.
`roscore`
If no errors appear everything should be ok.


---

## 3. Optional: Mount a Shared Folder Between macOS and the VM

You might want to edit code directly in your PC instead of working inside the VM (to use your usual code editor for example). To share files between your macOS host and the Ubuntu VM:

1. Create a folder, e.g., `~/ros_shared`, or clone this repository on macOS (we will assume that you cloned the repository) (remember that the repository folder will appear inside the folder where you are when cloning it):  
   `mkdir -p ~/ros_shared` or `git clone https://github.com/GabrielCostaBatista/SAuto.git`

2. Mount it into the VM:  
   `multipass mount SAuto ros1-vm:/home/ubuntu/SAuto`

3. Check if the shared folder is inside the VM:
   `multipass shell ros1-vm` 
   `ls`


---

## 4. Additional Tips

- Use `multipass list` to check running instances and their IP addresses.  
- Use `multipass exec ros1-vm -- <command>` to run commands inside the VM without opening a shell.  
- To stop and delete the VM:  
  `multipass stop ros1-vm`  
  `multipass delete ros1-vm`  
  `multipass purge`

---

You now have a working ROS Noetic environment inside an Ubuntu 20.04 VM on your macOS machine using Multipass!


---

# 2. Setting Up Foxglove Studio with ROS Noetic

Foxglove Studio is a modern visualization tool for ROS data, similar to RViz, but with a web and desktop interface and advanced features.

## 2.1. Install the Foxglove Bridge in ROS VM

Inside your Multipass Ubuntu VM, run:  
`sudo apt update`  
`sudo apt install ros-noetic-foxglove-bridge`

## 2.2. Launch the Foxglove Bridge

Start the bridge node so Foxglove Studio can connect:  
`roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765`  
This opens a WebSocket server on port 8765.

## 2.3. Install Foxglove Studio on macOS

- Download and install Foxglove Studio for macOS from https://foxglove.dev/download  
- Or use the web version at https://studio.foxglove.dev

## 2.4. Connect Foxglove Studio to Your ROS VM

1. Find your VM's IP address by running on your host:  
   `multipass list`  
2. Open Foxglove Studio (desktop or web).  
3. Click "Open connection" and select "Foxglove WebSocket".  
4. Enter the WebSocket URL:  
   `ws://<VM-IP>:8765` (replace `<VM-IP>` with your VM's IP)   
5. Connect and start visualizing your ROS topics.

## 2.5. Troubleshooting

- If you cannot connect, ensure the bridge is running and port 8765 is accessible.  
- Multipass uses NAT networking; if connection fails from host, try the web app inside the VM or set up port forwarding.  
- Foxglove Studio requires creating a free account and organization on first use.  
- Use the shared folder (if set up) to save and share layouts or data between host and VM.

---

You now have Foxglove Studio set up and connected to your ROS Noetic environment running in a Multipass VM on macOS!

