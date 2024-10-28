#Windowsでの実行手順
ROS2をセットアップ(https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)

```
(x64 Native Tools Command Prompt for VS2019)
call C:\dev\ros2_humble\local_setup.bat

build前にCOMポートは適宜書き換える(デバイスマネージャー参照)
distance直下で
colcon build --symlink-install
call install/setup.bat
ros2 run distance_measure sensor_node
```