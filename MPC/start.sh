# HELP:
# list screen: screen -ls
# restore screen: screen -r name
# detach: Ctrl-a + Ctrl-d
cd ~
screen -m -d -S rosbot bash -c 'ros2 launch rosbot_description rosbot_pro_nolidar_rosbot2.launch.py'
#screen -m -d -S rosbot bash -c 'ros2 launch rosbot_description rosbot_pro.launch.py'
echo "[INFO] ROSbot launch started!"
sleep 10
screen -m -d -S vnav bash -c 'python3 MPC/vnav.py'
echo "[INFO] Vectornav running!"
sleep 5
screen -m -d -S rtk bash -c 'python3 MPC/rtk2.py'
echo "[INFO] RTK running!"