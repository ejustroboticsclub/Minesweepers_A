### Alerts
- Always Check for Undervoltage and Temprature in your RPi if Undervoltage detected you should use another power source
  - check it with:
    ```
    ./volt.sh
    ./temp.sh
    
### Dependencies
- **tf & tf2**
```
sudo apt-get install ros-${ROS_DISTRO}-turtle-tf2 ros-${ROS_DISTRO}-tf2-tools ros-${ROS_DISTRO}-tf
```
- **wiringPi**
  - Install it with:
    ```bash
    git clone https://github.com/TheNextLVL/wiringPi.git
    cd wiringPi
    ./build
    ```

- **Enable I2C on Raspberry Pi**
  1. **Add the user to the I2C group:**
     ```bash
     sudo usermod -aG i2c pi
     ```

  2. **Change permission of I2C-1 (or I2C-0):**
     ```bash
     sudo chmod 666 /dev/i2c-1
     ```

  3. **Change the rule:**
     ```bash
     sudo nano /etc/udev/rules.d/99-i2c.rules
     ```
     - Add the following line:
       ```bash
       SUBSYSTEM=="i2c-dev", MODE="0666"
       ```
     - Save the file and exit.

  4. **Reload the udev rules:**
     ```bash
     sudo udevadm control --reload-rules
     sudo udevadm trigger
     ```

  5. **Verify permissions:**
     ```bash
     ls -l /dev/i2c-*
     ```
     - It should show:
       ```plaintext
       crw-rw-rw- 1 root root 89, 1 Apr 21  2022 /dev/i2c-1
       ```
