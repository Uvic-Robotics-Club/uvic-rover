


cameras + phidget + front worm gears

jetson setup
- double check usb rules permission
- download PWM.h arduino library
- install VNC server 
    - mostly arduino ports
- time server linux chrony time
    - im sorry forgot how to do it


ros 

worm gear ros flow
joystick python script -> ros topic output pwm and dir [int,bool] -> arduino

phidget
joystick python script -> python library calls




joystick mapping 




orin side (joe)
1. cameras 
2. phidget motors
3. worm gear + arduino
- front motor controllers take pwm and dir
- back motor take rc
- encoders

4. arm
- id: angle arduino file
    - wrap in ros
- sam's script : input: 1x6, output: 6 angles [0,0,0,0,0,0] corny: gripper same code as last year 
- map to joystick (help sam)
5. gps (day 2 traversal)/ day 1 2nd task search rescue/ science

macbook side
prereq: connnect two controllers 
one for drive
one for the arm*
gui for cameras

gui for gps 





```
$ rqt
```


# gstreamer options


# On the source computer (the one with the USB camera):
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=station port=5000
# On the receiving computer:
gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink

