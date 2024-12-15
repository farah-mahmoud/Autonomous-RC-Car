# Setup
We will be developing our code using ROS on Ubuntu for Gazebo.


## Ubuntu
Download Ubuntu 20.04 LTS (Focal Fossa) from [here](https://releases.ubuntu.com/20.04/).  
Install the desktop image.  
Follow this [video](https://youtu.be/-iSAyiicyQY?si=fcQH9JyLW-xjKPko) to install Ubuntu alongside Windows as a dual boot.  
Allocate at least 30GB of space or more for Ubuntu.  
Make yourself familiar with Ubuntu and install VSCode and Discord from the Ubuntu software store.

Start a terminal and run:

	sudo apt update
	sudo apt upgrade
	
---

# ROS2 Setups
## ROS2 Foxy
Follow this [guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) to install ROS 2 Foxy.

## Gazebo Simulation
to install gazebo:

	sudo apt install ros-foxy-gazebo-ros-pkgs
	sudo apt install ros-foxy-gazebo-ros-pkgs

## rviz2
to install rviz2:

	sudo apt install ros-foxy-rviz2

## Check Installations
To check if Gazebo is installed correctly:

	gazebo
 
To check if RViz2 is installed correctly:

	rviz2
 
If both open without any errors, then you've successfully installed Gazebo and RViz2.
---

# VSCode
Run the below commands to install helpful extensions:

	code --install-extension aaron-bond.better-comments
	code --install-extension akamud.vscode-theme-onedark
	code --install-extension christian-kohler.path-intellisense
	code --install-extension cschlosser.doxdocgen
	code --install-extension DavidAnson.vscode-markdownlint
	code --install-extension eamodio.gitlens
	code --install-extension hbenl.vscode-test-explorer
	code --install-extension IronGeek.vscode-env
	code --install-extension jeff-hykin.better-cpp-syntax
	code --install-extension matepek.vscode-catch2-test-adapter
	code --install-extension ms-azuretools.vscode-docker
	code --install-extension ms-python.python
	code --install-extension ms-vscode-remote.remote-containers
	code --install-extension ms-vscode-remote.remote-ssh
	code --install-extension ms-vscode-remote.remote-ssh-edit
	code --install-extension ms-vscode-remote.remote-wsl
	code --install-extension ms-vscode-remote.vscode-remote-extensionpack
	code --install-extension ms-vscode.cmake-tools
	code --install-extension ms-vscode.cpptools
	code --install-extension ms-vscode.test-adapter-converter
	code --install-extension ms-vscode.vs-keybindings
	code --install-extension PKief.material-icon-theme
	code --install-extension redhat.vscode-xml
	code --install-extension streetsidesoftware.code-spell-checker
	code --install-extension twxs.cmake
	code --install-extension usernamehw.errorlens
	code --install-extension yzhang.markdown-all-in-one

# Terminator
	sudo apt install terminator
