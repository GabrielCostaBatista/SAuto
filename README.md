# ROS Noetic on macOS with Multipass & Foxglove Setup Guide

This guide covers setting up ROS Noetic on macOS using Multipass virtualization and configuring Foxglove.

## Table of Contents

- [1. Installing Multipass and ROS Noetic on macOS](#1-installing-multipass-and-ros-noetic-on-macos)
  - [1.1. Install Multipass on macOS](#11-install-multipass-on-macos)
  - [1.2. Install ROS Noetic inside the Ubuntu 20.04 VM](#12-install-ros-noetic-inside-the-ubuntu-2004-vm)
  - [1.3. Optional (Deprecated): Mount a Shared Folder Between macOS and the VM](#13-optional-deprecated-mount-a-shared-folder-between-macos-and-the-vm)
  - [1.4. Fixing catkin_make errors by using VSCode remote instead of shared folder](#14-fixing-catkin_make-errors-by-using-vscode-remote-instead-of-shared-folder)
  - [1.5. Additional Tips](#15-additional-tips)
- [2. Setting Up Foxglove Studio with ROS Noetic (VM to MacOS)](#2-setting-up-foxglove-studio-with-ros-noetic-vm-to-macos)
  - [2.1. Install the Foxglove Bridge in ROS VM](#21-install-the-foxglove-bridge-in-ros-vm)
  - [2.2. Launch the Foxglove Bridge](#22-launch-the-foxglove-bridge)
  - [2.3. Install Foxglove Studio on macOS](#23-install-foxglove-studio-on-macos)
  - [2.4. Connect Foxglove Studio to Your ROS VM](#24-connect-foxglove-studio-to-your-ros-vm)
  - [2.5. Troubleshooting](#25-troubleshooting)
- [3. Setting Up Video Stream from Alphabot2 to MacOS, using Foxglove](#3-setting-up-video-stream-from-alphabot2-to-macos-using-foxglove)
  - [3.1. Install the Foxglove Bridge in Alphabot](#31-install-the-foxglove-bridge-in-alphabot)
  - [3.2. Start camera (Streaming images example specific)](#32-start-camera-streaming-images-example-specific)
  - [3.3. Install Foxglove Studio on macOS](#33-install-foxglove-studio-on-macos)
  - [3.4. Stream images in Foxglove](#34-stream-images-in-foxglove)

---

# 1. Installing Multipass and ROS Noetic on macOS

This guide explains how to install Multipass on macOS and set up ROS Noetic inside an Ubuntu 20.04 Multipass VM, following the ROS installation tutorial for Ubuntu 20.04 from https://varhowto.com/install-ros-noetic-ubuntu-20-04.

## 1.1. Install Multipass on macOS

Multipass allows you to run Ubuntu VMs easily on macOS.

### Steps:

1. Download and install Multipass from the official site: https://multipass.run/

2. Verify installation by opening Terminal and running:  
   ```bash
   multipass version
   ```

3. Launch an Ubuntu 20.04 VM (name it ros1-vm):  
   ```bash
   multipass launch 20.04 --name ros1-vm --memory 4G --disk 40G
   ```

4. Access the VM shell:  
   ```bash
   multipass shell ros1-vm
   ```

## 1.2. Install ROS Noetic inside the Ubuntu 20.04 VM

Follow the steps below inside the VM shell (ros1-vm):

### Step 1:  Set up ROS Noetic repo for Ubuntu 20.04 
```bash
echo "deb http://packages.ros.org/ros/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros-focal.list
```

### Step 2: Set up your keys  
```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### Step 3: Update package index  
```bash
sudo apt update
```

### Step 4: Install ROS Noetic Desktop Full  
```bash
sudo apt install ros-noetic-desktop-full -y
```

### Step 5: Environment setup  
Add the ROS environment variables to your bash session:  
```bash
source /opt/ros/noetic/setup.bash  
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 6: Verify Noetic installation
```bash
roscd
```
This should take you to `/opt/ros/noetic`.
```bash
roscore
```
If no errors appear everything should be ok.

## 1.3. Optional (Deprecated): Mount a Shared Folder Between macOS and the VM

You might want to edit code directly in your PC instead of working inside the VM (to use your usual code editor for example). To share files between your macOS host and the Ubuntu VM:

1. Create a folder, e.g., `~/ros_shared`, or clone this repository on macOS (we will assume that you cloned the repository) (remember that the repository folder will appear inside the folder where you are when cloning it):  
   ```bash
   mkdir -p ~/ros_shared
   ```
   or 
   ```bash
   git clone https://github.com/GabrielCostaBatista/SAuto.git
   ```

2. Mount it into the VM:  
   ```bash
   multipass mount SAuto ros1-vm:/home/ubuntu/SAuto
   ```

3. Check if the shared folder is inside the VM:
   ```bash
   multipass shell ros1-vm 
   ls
   ```

## 1.4. Fixing catkin_make errors by using VSCode remote instead of shared folder

### 1. Unmount the Shared Folder `/home/ubuntu/SAuto`
1. If you previously mounted a shared folder at `/home/ubuntu/SAuto`, unmount it from your macOS terminal (not inside the VM) with:

```bash
multipass umount ros1-vm:/home/ubuntu/SAuto
```

Replace `ros1-vm` with your VM's name if it's different.


### 2. Set Up GitHub SSH Access Inside the VM

**a. Generate a new SSH key pair (inside the VM):**

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
```

- When prompted for a file location, press Enter to accept the default.
- Optionally set a passphrase, or press Enter to leave it empty.

**b. Add your SSH key to the SSH agent:**

```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```

**c. Copy your public key to your clipboard:**

```bash
cat ~/.ssh/id_ed25519.pub
```

- Copy the entire output.

**d. Add the public key to your GitHub account:**
- Go to GitHub → Settings → SSH and GPG keys → New SSH key.
- Paste your copied key and give it a descriptive title (e.g., "Multipass VM").
- Save the key.

**e. Test the SSH connection to GitHub:**

```bash
ssh -T git@github.com
```

- On first connection, type `yes` to confirm.  
- You should see a welcome message if successful.


### 3. Clone the Repository Using SSH

Now you can clone the repository securely without username/password prompts:

```bash
git clone git@github.com:GabrielCostaBatista/SAuto.git
```


### 4. Setup VSCode Remote-SSH

VSCode Remote-SSH allows you to use your macOS VSCode to edit files directly on the Multipass VM, providing a seamless development experience without the permission issues of shared folders.

**a. Set up SSH keys for Multipass VM access:**
- In MacOS, check if you already have an SSH key:
  ```bash
  ls -la ~/.ssh
  ```
  Look for files named `id_ed25519` and `id_ed25519.pub` (or `id_rsa` and `id_rsa.pub`). If these exist, you can skip the key generation step.

- If you don't have an SSH key, generate one:
  ```bash
  ssh-keygen -t ed25519
  ```
  Press Enter to accept the default location.
  
- Get your VM's IP address:
  ```bash
  multipass list
  ```
  Note the IP address of your `ros1-vm` VM (we'll refer to it as `<VM-IP>`)
  
- For Multipass VMs, use this special method to add your SSH key:
  ```bash
  cat ~/.ssh/id_ed25519.pub | multipass exec ros1-vm -- bash -c "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
  ```
  If you're using RSA keys instead of ED25519, use `~/.ssh/id_rsa.pub` in the command above.

- Test the connection:
  ```bash
  ssh ubuntu@<VM-IP>
  ```
  You should now be able to connect without a password prompt.

**b. Install the Remote-SSH extension in VSCode:**
- Open VSCode on your macOS
- Go to Extensions (or press `Cmd+Shift+X`)
- Search for "Remote - SSH" by Microsoft
- Click Install

**c. Configure SSH connection in VSCode:**
- In VSCode, press `F1` or `Cmd+Shift+P` to open the command palette
- Type "Remote-SSH: Add New SSH Host" and select it
- Enter the SSH connection command:
  ```
  ssh ubuntu@<VM-IP>
  ```
  (Replace `<VM-IP>` with your VM's actual IP address)
- Choose a config file to update (usually the first option is fine)

**d. Connect to your VM:**
- In VSCode, click on the green button in the bottom-left corner
- Select "Connect to Host..." from the menu
- Choose your VM from the list
- VSCode will connect using your SSH key

**e. When connected:**
- VSCode will open a new window connected to your VM
- Go to "File > Open Folder" to navigate to your project folder (e.g., `/home/ubuntu/SAuto`)
- You now have full access to your ROS project with all VSCode features

**f. Create a persistent bookmark:**
- After successfully connecting once, your VM will appear in the Remote Explorer sidebar
- (Optional) You can change the name that appears on the sidebar by altering the name after Host in `~/.ssh/config` file.

Now you can develop ROS applications using your familiar macOS VSCode environment while the code runs natively in the Ubuntu VM, avoiding permission issues with shared folders while getting the full development experience.

## 1.5. Additional Tips

- Use `multipass list` to check running instances and their IP addresses.  
- Use `multipass exec ros1-vm -- <command>` to run commands inside the VM without opening a shell.  
- To stop and delete the VM:  
  ```bash
  multipass stop ros1-vm  
  multipass delete ros1-vm  
  multipass purge
  ```

> 🎉 You now have a working ROS Noetic environment inside an Ubuntu 20.04 VM on your macOS machine using Multipass!

---

# 2. Setting Up Foxglove Studio with ROS Noetic (VM to MacOS)

Foxglove Studio is a modern visualization tool for ROS data, similar to RViz, but with a web and desktop interface and advanced features.

## 2.1. Install the Foxglove Bridge in ROS VM

Inside your Multipass Ubuntu VM, run:  
```bash
sudo apt update  
sudo apt install ros-noetic-foxglove-bridge
```

## 2.2. Launch the Foxglove Bridge

Start the bridge node so Foxglove Studio can connect:  
```bash
roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765
```
This opens a WebSocket server on port 8765.

> **Note**: The `--screen` option in roslaunch is used to display the standard output (stdout) and standard error (stderr) from all nodes directly in the terminal where you run the command, rather than logging this output to files. The `port:=(number)` argument allows you to set the port (the default is 8765).

## 2.3. Install Foxglove Studio on macOS

- Download and install Foxglove Studio for macOS from https://foxglove.dev/download  
- Or use the web version at https://studio.foxglove.dev

## 2.4. Connect Foxglove Studio to Your ROS VM

1. Find your VM's IP address by running on your host:  
   ```bash
   multipass list
   ```
2. Open Foxglove Studio (desktop or web).  
3. Click "Open connection" and select "Foxglove WebSocket".  
4. Enter the WebSocket URL:  
   `ws://<VM-IP>:8765` (replace `<VM-IP>` with your VM's IP, remember that you can find it by running `multipass list` outside your VM).
5. Connect and start visualizing your ROS topics.

## 2.5. Troubleshooting

- If you cannot connect, ensure the bridge is running and port 8765 is accessible.  
- Multipass uses NAT networking; if connection fails from host, try the web app inside the VM or set up port forwarding.  
- Foxglove Studio requires creating a free account and organization on first use.  
- Use the shared folder (if set up) to save and share layouts or data between host and VM.

---

# 3. Setting Up Video Stream from Alphabot2 to MacOS, using Foxglove

## 3.1. Install the Foxglove Bridge in Alphabot

Access the Alphabot using ssh:
```bash
ssh alphabot2@192.168.28.54
```

If you haven't previously, install Foxglove bridge using:
```bash
sudo apt install ros-noetic-foxglove-bridge
```

Then, launch the foxglove_bridge using:
```bash
roslaunch foxglove_bridge foxglove_bridge.launch
```

The robot is now streaming the topics running inside it (usually in port 8765).

## 3.2. Start camera (Streaming images example specific)

The robot is streaming the topics but the camera hasn't been started yet so let's do it using commands:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch raspicam_node camerav2_1280x960.launch
```

Now, the robot is streaming the images and we only need to collect them on the MacOS side

## 3.3. Install Foxglove Studio on macOS

- Download and install Foxglove Studio for macOS from https://foxglove.dev/download  
- Or use the web version at https://studio.foxglove.dev

## 3.4. Stream images in Foxglove

Open Foxglove studio and click on `Open connection...`. Then, input `ws://192.168.28.54:8765` into the WebSocket URL box and click `Open`.

You should now be able to see the images being streamed on the up-right corner.

---

> 📷 You can now stream images to MacOS from the Alphabot using Foxglove. You can follow the same process to access any topic you launched on Alphabot in MacOS.
