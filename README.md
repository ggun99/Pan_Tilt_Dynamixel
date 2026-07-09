# Pan_Tilt_Dynamixel

This is pan-tilt controller with PID control.

We used 2 * Dynamixel XM430-W350-T, D435.

With the camera, it can be follow your face or someone's.
It follows the first detected face. 



## Pre-Installation


Before use this code, you have to install opencv-contrib-python, DynamixelSDK. 

```bash
pip install opencv-contrib-python
```
Becareful installed version, because there are some differences in latest version. We used opencv-contrib-python 4.10.0.84 to utilize this code. And also, becareful about conflict between the opencv-python, opencv-python-headless. I searched that it's better to use just one python package.

- [DynamixelSDK Download](https://docs.robotis.com/docs/software/dynamixel_sdk/download)

We just download Option1 source code. 

```bash
$ sudo apt update
$ sudo apt install python3 python3-pip python3-serial
$ cd DynamixelSDK/python
$ pip install .
```

Before using the dynamixels, you have to connect board that you use and dynamixels. 

And give them different ID. We used DynamixelWizard2.0 to change the ID. Default was 1, so you have to change one dynamixel ID to 2. In our case, we used pan ID : 2, tilt ID : 1. If it's different, then you can change the ID value in the code(in pid_dynamixel.py, PAN_ID = 2, TILT_ID = 1). 

And that's all! Enjoy!! 

