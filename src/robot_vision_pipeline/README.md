source ~/venvs/ros_env/bin/activate

QT_QPA_PLATFORM=xcb ros2 run rqt_image_view rqt_image_view




 ros2 run usb_cam usb_cam_node_exe --ros-args   -p video_device:=/dev/video2   -p image_width:=640   -p image_height:=480   -p pixel_format:=yuyv   -r image_raw:=/astra/rgb/image_raw