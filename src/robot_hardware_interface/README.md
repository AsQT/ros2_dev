    # robot_hardware_interface

    ## 1. Vai trò package
    Hardware interface TCP/RS485 cho robot, gồm node standalone `robot_hw_node`, plugin ros2_control `tcp_system_hardware`, service điều khiển trục và message flag trạng thái.

    ## 2. Vị trí trong hệ thống
    Tầng sát phần cứng. Nhận lệnh từ GUI/controller_manager, publish joint state/status và cung cấp service connect/servo/home/jog/run/stop.

    ## 3. Thành phần chính
    - `src/robot_hw_node.cpp`: node hardware standalone qua TCP client.
- `src/tcp_system_hardware.cpp`: plugin ros2_control lifecycle.
- `src/tcp_client.cpp`: framing/protocol TCP.
- `src/rs485_hw_node.cpp`, `rs485_system_hardware.cpp`: biến thể RS485 trong source.
- `srv/*.srv`: service điều khiển axis/all.
- `msg/AxisFlag.msg`, `FlagStatus.msg`: trạng thái servo/limit/emergency.
- `scripts/hardware_gui.py`, `test_servo_all_flags.py`: tool test.
- `config/params.yaml`, `plugin.xml`, `launch/hardware_interface.launch.py`.

    ## 4. Node / executable
    | Node / executable | Nguồn | Vai trò |
    |---|---|---|
    | Executable `robot_hw_node`; script `hardware_gui.py`; plugin library `tcp_system_hardware` cho ros2_control. | package source/launch | Runtime của package hoặc node được launch |

    ## 5. Topic / Service / Action
    | Interface | Type | Vai trò |
    |---|---|---|
    | /robot_hw/servo_all | service/topic | hardware IO |
| /robot_hw/home | service/topic | hardware IO |
| /robot_hw/jog | service/topic | hardware IO |
| /robot_hw/run_axis | service/topic | hardware IO |
| /robot_hw/stop_axis | service/topic | hardware IO |
| /robot_hw/stop_all | service/topic | hardware IO |
| /robot_hw/flags | service/topic | hardware IO |
| /joint_states | service/topic | hardware IO |

    Ghi chú interface: Publish `/joint_states`, `/robot_hw/connected`, `/hardware/connected` legacy, `/robot_hw/status_text`, `/robot_hw/flags`, `/hardware/flags`; subscribe `/robot_hw/servo_axis_cmd`, `/robot_hw/run_axis_cmd`, `/robot_hw/jog_cmd`, `/robot_hw/run_all_cmd`, controller trajectory; services `/robot_hw/connect`, `/robot_hw/disconnect`, `/robot_hw/poll_now`, `/robot_hw/servo_all`, `/robot_hw/servo_on_axis`, `/robot_hw/servo_on_all`, `/robot_hw/jog`, `/robot_hw/home`, `/robot_hw/run_axis`, `/robot_hw/stop_axis`, `/robot_hw/stop_all`.

    ## 6. File launch liên quan
    hardware_interface.launch.py

    ## 7. File cấu hình liên quan
    config/params.yaml, plugin.xml

    ## 8. Cách build riêng package
    ```bash
    cd ~/ros2_dev
    colcon build --packages-select robot_hardware_interface
    source install/setup.bash
    ```

    ## 9. Cách chạy nhanh
    ```bash
    source ~/ros2_dev/install/setup.bash
    ros2 launch robot_hardware_interface hardware_interface.launch.py
    ```
    Nếu package không có launch/runtime node, dùng nó bằng cách phụ thuộc interface hoặc include file từ package khác.

    ## 10. Ghi chú kỹ thuật / giới hạn hiện tại
    - Source of truth là source code hiện tại trong `robot_hardware_interface`.
    - Tài liệu này không thay thế kiểm tra runtime bằng `ros2 node list`, `ros2 topic list`, `ros2 service list`, `ros2 action list` sau khi launch.
    - Không có phụ thuộc MoveIt/action đặc biệt được xác định từ source.
