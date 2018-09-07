# raspberry-pi-camera
A simple Raspberry Pi camera with video recording option. Cloned from gplhedge on 9/7/2018.

# Required hardware
1. Raspberry Pi 2 Model B
2. Raspberry Pi Camera from OmniVision technologies.
3. Two push button switches.
4. Two LEDs
5. 330ohm current limiting resistors for LEDs.
6. Connecting wires.

# Connections
## LED connections
- Connect LED0( recording status LED) between GPIO 4 and GND via 330ohm resistor
- Connect LED1( camera status LED) between GPIO 18 and GND via 330ohm resistor

## Push button switch connections
- Connect PB0(camera control switch) between GPIO 14 and GND
- Connect PB1(recording and capture switch) between GPIO 15 and GND

## Camera connections
- Connect Pi camera such that the blue strip on the connector is facing the ethernet port
- You need to enable the camera in raspi-config
Refer to [camera setup guide](https://www.raspberrypi.org/documentation/usage/camera/README.md) for detailed info on this.

# Video recording and image capturing
- Push PB0 to turn ON camera. The button press toggles the ON/OFF state of the camera. The LED1 is ON when camera is ON
- Press PB1 to capture image. The LED0 blinks 3 times to confirm the image capture.
- Long press PB1 to start recording video. LED0 turns on and stays ON during video recording. Press PB1 again to stop recording the video.

**NOTE:** You cannot capture image when video recording is in progress. Press hold PB1 until LED0 turns on and then release to start
recording the video.

# Where the recorded stuffs are stored?
- Videos are stored under /data/videos and images are stored under /data/images

**NOTE** You need to create /data directory or run the script with  superuser permission so that the script can create those directories.
You can change these locations by editing the script. Also, the image and video resolutions can be changed by modifying the global variables
that are there in the beginning of this rasp_cam.py.

