## Raptor Robot V2

This is my attempt at a ros2 slam robot with voice commands and an AI backend.

I plan to use https://github.com/hippo5329/linorobot2_hardware as an esp32 based start, using 7960's to control powered wheelchair motors. The rest should be "Standard ROS2 Packages" and this git is just the config of those packages.

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has direcctories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).
