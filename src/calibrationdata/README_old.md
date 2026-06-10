ros2 run usb_cam usb_cam_node_exe   --ros-args   -r __ns:=/webcam   -p video_device:=/dev/video2   -p image_width:=640   -p image_height:=480   -p pixel_format:=yuyv   -p framerate:=30.0   -p camera_name:=webcam

ros2 run camera_calibration cameracalibrator   --size 7x5   --square 0.029   --no-service-check   --ros-args   -r image:=/webcam/image_raw   -r camera:=/webcam

ls /tmp/calibrationdata.tar.gz
mkdir -p ~/ros2/calibration/webcam

cp /tmp/calibrationdata.tar.gz ~/ros2/calibration/webcam/

cd ~/ros2/calibration/webcam
tar -xzf calibrationdata.tar.gz

ls
mv ost.yaml webcam.yaml
mkdir -p ~/.ros/camera_info
cp ~/ros2/calibration/webcam/webcam.yaml ~/.ros/camera_info/webcam.yaml
ros2 topic echo /webcam/camera_info --once

ros2 run usb_cam usb_cam_node_exe   --ros-args   -r __ns:=/webcam   -p video_device:=/dev/video2   -p image_width:=640   -p image_height:=480   -p pixel_format:=yuyv   -p framerate:=30.0   -p camera_name:=webcam

[image]

width
640

height
480

[narrow_stereo]

camera matrix
511.981181 0.000000 314.328072
0.000000 515.414836 237.447377
0.000000 0.000000 1.000000

distortion
0.248413 -0.219012 0.008959 -0.008151 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
550.386658 0.000000 309.007341 0.000000
0.000000 554.766785 240.948064 0.000000
0.000000 0.000000 1.000000 0.000000

