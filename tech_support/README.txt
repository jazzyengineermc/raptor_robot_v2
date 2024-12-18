Tech Support
    after installation is complete you can drive raptor around in gazebo simulator  :)
    the wheelchair picture is what I turned into 'Raptor'. I first removed all existing electronics and brakes from the motors
    made brackets to hold quadrature encoders driven via a belt drive on the back of the motor giving me a whopping 42k ticks per wheel rev. Mega resolution!!
    arduino nano and 2 IBT-2 motor driver boards drive it using 18v cordless drill batteries.
    The main robot base is running on a Raspberry Pi 4b 8g model with a m.2 250g ssd via USB3 port

Ubuntu 22.04 -- unless you know what your doing, trying to be beginner friendly

()Installing info

cd <arduino development directory>
tar -xvf <path to>ROSArduinoBridge.tar 

use Arduino IDE to edit pid values for your motors and pin numbers to hardware wiring
-note- I used an Arduino nano and included jpg for easy pinout reference

cd ~/dev_ws/src
tar -xvf <path to>diffdrive_arduino.tar

cd ~/dev_ws && colcon build --symlink-install

use minicom -b 57600 /dev/ttyUSB0  # or your arduino device path
e reports back wheel encoder counts
o <n> <n> sets a PWM pulse to the wheels # <n> should be 0 - 255

mkdir ~/bin
echo "export PATH=$PATH:/home/${whoami}/bin" >> ~/.bashrc
source ~/.bashrc
cp ~/dev_ws/src/raptor_robot_v2/tech_support/robot-installer.sh ~/bin
chmod 755 ~/bin/robot-installer.sh

from now on you can run:
    "robot-installer.sh robot" to get the latest robot code"
    "robot-installer.sh ros2" to install ros-humble and all packages needed

optional addons:
    cd ~/dev_ws/src && git clone https://github.com/jazzyengineermc/raptor_v2_ui.git  # for digital eyes :)
    camera gear for openCV related development # RealSense d435i camera is what I use and webcam
    audio gear for speach-to-speech commands # I use stereo mic from webcam and usb soundcard w/ speakers
    robot arm for object manipulation # I don't have one... yet ;)
