## Run Program On Go2W with the CMU Navigation System

### Start Hardware
1. Connect the bettery with the NUC and Jetson Orin.
2. Login the **jjiao** account with the password.
3. Check whether the network connection of the NUC is **Anymal_Shared_Network**.
4. Turn on the Go2W and swith its mode as **walking mode**.
5. Run ```./start_vnc.sh``` on both the NUC and Jetson, you can use the Remmina to remote control them.
<div align="center">
  <a href="">
    <img align="center" src="image/anymal_d_hardware.jpeg" width="50%" alt="anymal_d_hardware">
  </a> 
</div>

### Start VNC
1. (NUC/Jetson) settings -> sharing -> scene sharing (turn on)
2. (User PC) open Remmina Remote Destkop -> create a VNC setting -> click connect
3. **Note**: if you cannot connect to the NUC, please check the wifi setting of NUC, whether **Anymal_Shared_Network** is connected
<div align="center">
  <a href="">
    <img align="center" src="image/anymal_d_vnc_setting.png" width="40%" alt="anymal_d_vnc_setting">
  </a> 
</div>

### Start Sensor
1. Setup sensor on the NUC (start Livox, publish TF): 
   ```
   cd ~/robohike_ws/src/RPL-RoboHike/config_launch_go2w
   ./run_nuc_go2w_sensor_setup.sh
   
   ```

2. Setup sensor on the Jetson (start Zed camera): 
   ```
   cd ~/robohike_ws/src/RPL-RoboHike/config_launch_go2w
   ./run_orin_go2w_sensor_setup.sh
   ```

3. Open RVIZ on the NUC to check whether the zed camera already setup

   ```
   rviz -d config_launch_anymal/rviz_cfg/zed.rviz
   ```

4. Check whether sensor data is ready:

   ``````
   rostopic hz /livox/imu
   ``````

<!-- TODO -->
### RUN the CMU Navigtion System on Anymal

1. Setup cmu navigation:
   ```
   cd ~ && ./run_nuc_anymal_real_system.sh
   roslaunch config_launch_anymal/launch/cmu_exploration/anymal_real_system.launch
   ```
   A rviz will open and visualize message
2. Use the [Game Joystick](image/joystick_esm9013_description.png) and click the ```start``` button, you can see message in the terminal
   ```
   [Navigation Control] Will republish velocity to /motion_reference/command_twist message
   ```
   This means that your joystick is taking the control
3. Provide a waypoint and Press ```auto mode``` to let the robot autonomouslymove
4. [Emergency] Use the ANYMal joystick to avoid any danger, and recover ```auto mode``` by pressing the ```start``` button with the game joystick.
