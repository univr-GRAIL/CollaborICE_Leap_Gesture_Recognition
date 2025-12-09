esecuzione:

powershell da amministratore
usbipd list
(controlla id della usb da usare)
usbipd bind --busid 4-4
usbipd attach --wsl --busid 4-4

in ubuntu:
dentro colcon_ws
colcon build --symlink-install
. install/setup.bash

per lanciare app: (nome package e poi nome eseguibile)
ros2 run leap_gesture_recognition leap_gesture_recognition
ros2 run leap_motion leap_motion


visualizza che stampa il topic
ros2 topic echo /leapmotion/joints


se libreria si spacca
cd colcon_ws/
rm -rf build/ install/ log/

installare librerie:
https://github.com/ultraleap/leapc-python-bindings/tree/main

