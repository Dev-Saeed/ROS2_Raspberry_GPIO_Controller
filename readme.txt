# raspberry_gpio ROS 2 Package

This package provides a ROS 2 node for controlling GPIO pins on a Raspberry Pi using the WiringPi library. 
Below are instructions to set up WiringPi and use this package on any compatible device.

## Prerequisites

Ensure your system has the necessary tools and dependencies installed:

1. **ROS 2 Installed**:
   - Follow the official ROS 2 installation guide: https://docs.ros.org/en/rolling/Installation.html

2. **WiringPi Library Installed**:
   - WiringPi is a GPIO library for Raspberry Pi that must be installed and configured.

    ## WiringPi Library Install and Setup:

      git clone https://github.com/WiringPi/WiringPi.git
      cd WiringPi
      sudo ./build
      mv debian-template/wiringpi-3.0-1.deb .
      sudo apt install ./wiringpi-3.0-1.deb
      sudo cp /home/your_username/WiringPi/wiringPi/wiringPi.h /usr/include/

3. Set GPIO Permissions
      
      sudo usermod -aG dialout $USER
      sudo reboot

   if it didnt work, try:
      sudo groupadd gpio
      sudo usermod -aG gpio $USER
      sudo chown root:gpio /dev/gpiomem
      sudo chmod 660 /dev/gpiomem
      sudo usermod -aG dialout $USER
      sudo reboot

4. Now you can use the nodes and the launch files
        
