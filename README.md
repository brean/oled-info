# Show information on the 0.96 inch OLED Display
Mainly used for raspberry pi that is used in robotics or to display information for a small server.

Can be set up using Docker or venv.

The shown information are either provided from local sources or by the [sys_info](https://github.com/Fleet-Control/sys-info/) ROS2 nodes

Docker ROS 2 humble/jazzy environment for your Rasbperry PI to show some information that are interesting in robotics.
You can configure it to show different values, the default is:

 - WiFi IP-Address (using `psutil`)
 - Temperature (using the python bindings for `vcgencmd` for [raspberry pi](https://www.raspberrypi.com/documentation/computers/os.html))
 - Battery (read from the ROS 2-topic `battery_state`)
 - WiFi signal strength (by simply parsing `/proc/net/wireless`)
 - An overview as icons/emojis showing if any of the following problems occured:
    - ⚡ for undervoltage
    - ☹️ for CPU throtteling
    - **F** for CPU **F**requency cap
    - T for CPU soft **T**emperature limit
  
   followed by the same icons to show if any of these problems happending right now (If everything is fine you should see 4 dashes followed by a vertical line follwed by 4 additional dashes `----|----` - based on [`vcgencmd`](https://www.raspberrypi.com/documentation/computers/os.html#get_throttled)).
 - The systems ROS_DOMAIN_ID next to the overview (you should configure the docker-compose file to forward this to your docker container)

![image](https://github.com/user-attachments/assets/c6c1c69e-9de7-42a8-a9c6-76e0a11fe625)

## Installation
Just copy the docker-compose.yml to your robot, write your own or checkout the whole repository then run `docker compose up -d` and wait until the docker images are downloaded, 1-2 minutes later the information should show up on the connected screen.
If the docker deamon is setup to auto-start (for example via systemctl/systemd) the docker container will automatically start every time you start the raspberry pi.

## Troubleshooting
If you get the error message `sys_info_extended.py: error: I2C device not found: /dev/i2c-1` you need to activate the I2C-interface on the raspberry pi with `raspi-config`
