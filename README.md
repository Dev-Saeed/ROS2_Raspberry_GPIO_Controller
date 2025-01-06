# Raspberry GPIO Control for ROS 2

This package provides ROS 2 nodes for controlling GPIO pins on a Raspberry Pi using the WiringPi library. It includes a flexible node to manage multiple GPIO pins, supporting different modes and configurations, all specified via a YAML configuration file.

---

## Prerequisites

### 1. Install ROS 2
Ensure you have a ROS 2 distribution installed on your Raspberry Pi. You can follow the [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) for your desired distribution.

### 2. Install WiringPi
WiringPi is required to interface with the GPIO pins on the Raspberry Pi. Follow these steps to install WiringPi:

1. Clone the WiringPi repository:
   ```bash
   git clone https://github.com/WiringPi/WiringPi.git
   cd WiringPi
   ```

2. Build and install WiringPi:
   ```bash
   sudo ./build
   ```

3. Move the library file to the system library directory
   ```bash
   sudo cp /home/your_username/WiringPi/wiringPi/wiringPi.h /usr/include/
   ```
   replace your_username

---

## Installation

### 1. Clone the Repository
Clone this package into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Dev-Saeed/ROS2_Raspberry_GPIO_Controller raspberry_gpio
```

### 2. Build the Package
Navigate to your ROS 2 workspace and build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select raspberry_gpio
source install/setup.bash
```

### 3. Configure Permissions
To access GPIO hardware, ensure your user has the appropriate permissions:

1. Add your user to the `dialout` group:
   ```bash
   sudo usermod -aG dialout $USER
   ```

2. Reboot your system to apply the changes:
   ```bash
   sudo reboot
   ```

---

## Usage

### Multi GPIO Node

The multi GPIO node allows you to control multiple GPIO pins simultaneously. Each pin can be configured as an input, output, PWM, or clock pin. Configuration is provided via a YAML file.

#### Configuration File

Edit the `multi_gpio_params.yaml` file in the `config/` directory to configure your GPIO pins:
```yaml
multi_gpio_control_node:
  ros__parameters:
    pins:   [21 , 20 , 26]     # List of GPIO pins
    modes:  [0  , 0  , 1 ]     # Modes: 0 = OUTPUT, 1 = INPUT, 2 = PWM, 3 = GPIO_CLOCK
    pulls:  [0  , 1  , 0 ]     # Pull: 0 = PUD_DOWN, 1 = PUD_UP, 2 = PUD_OFF
    pub_rate: 500              # Publish rate in milliseconds for input pins
```
When the node is launched with this configuration, you will see messages like:
```bash
[WARN] [multi_gpio_control_node]: Pin 21 configured as OUTPUT
[WARN] [multi_gpio_control_node]: Pin 20 configured as OUTPUT
[WARN] [multi_gpio_control_node]: Pin 26 configured as INPUT with Pull-Down
[WARN] [multi_gpio_control_node]: Input state publishing every 500ms
``` 
This setup allows you to verify the configuration of each pin directly in the shell logs.

#### Expected ROS 2 Topics

After launching the node with the above configuration, the `ros2 topic list` command will show the following topics (along with standard ROS 2 topics):

```bash
/gpio/input_state_pin26
/gpio/output_control_pin20
/gpio/output_control_pin21
/parameter_events
/rosout
```

**Explanation of Topics**:
- `/gpio/input_state_pin26`: Publishes the state of pin 26 (configured as an input) every 500ms.
- `/gpio/output_control_pin20`: Allows control of pin 20 (configured as an output).
- `/gpio/output_control_pin21`: Allows control of pin 21 (configured as an output).

  This setup ensures each pin has a dedicated topic for its role, making it easy to interact with and monitor GPIO functionality in ROS 2.
---

### Launch the Multi GPIO Node

To launch the multi GPIO node with your configuration file:
```bash
ros2 launch raspberry_gpio multi_gpio_launch.py
```

#### Testing Input and Output

1. **Input Pins**:
   - Subscribe to the input state topics:
     ```bash
     ros2 topic echo /gpio/input_state_pin<NUM>
     ```
     Replace `<NUM>` with the pin number (e.g., `21`).

2. **Output Pins**:
   - Publish to the output control topics:
     ```bash
     ros2 topic pub --once /gpio/output_control_pin<NUM> std_msgs/msg/Bool "{data: true}"
     ```
     Replace `<NUM>` with the pin number (e.g., `20`).

---

## Troubleshooting

1. **Permissions**:
   Ensure your user is in the `dialout` group to access `/dev/gpiomem`:
   ```bash
   sudo usermod -aG dialout $USER
   sudo reboot
   ```
   if it didnt work, try:
   ```bash
   sudo groupadd gpio
   sudo usermod -aG gpio $USER
   sudo chown root:gpio /dev/gpiomem
   sudo chmod 660 /dev/gpiomem
   sudo usermod -aG dialout $USER
   sudo reboot
   ```
2. **WiringPi Issues**:
   If you encounter issues with WiringPi, ensure it is correctly installed by running:
   ```bash
   gpio readall
   ```

3. **Log Messages**:
   Check the ROS 2 logs for error messages:
   ```bash
   ros2 launch raspberry_gpio multi_gpio_launch.py
   ```

---

## Contributing

Contributions are welcome! Feel free to fork this repository, create new features, or fix bugs. Submit a pull request for review.

---

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
```
