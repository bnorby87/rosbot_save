# HELP:
# list screen: screen -ls
# restore screen: screen -r name
# detach: Ctrl-a + Ctrl-d
cd ~
#screen -m -d -S rosbot bash -c 'ros2 launch rosbot_description rosbot_pro_nolidar_rosbot2.launch.py'
screen -m -d -S rosbot bash -c 'ros2 launch rosbot_description rosbot_pro.launch.py'
echo "[INFO] ROSbot launch started!"
sleep 10
#screen -m -d -S mavsdk bash -c 'mavproxy.py --master=/dev/ttyPX4,57600 --out=udp:localhost:14550'
screen -m -d -S mavsdk bash -c 'mavproxy.py --master=/dev/ttyPX4,115200 --out=udpbcast:192.168.50.35:14550 --out=udpbcast:192.168.50.35:15550'
echo "[INFO] MAVProxy started!"
#sleep 10
#screen -m -d -S mavsdk bash -c '.local/lib/python3.8/site-packages/mavsdk/bin/mavsdk_server -p 50051 udp://:14550'
#echo "[INFO] MAVSDK started!"
#sleep 2
#screen -m -d -S px bash -c 'python3 demo/PX.py'
#echo "[INFO] Pixhawk started!"
#sleep 5
#screen -m -d -S rtk bash -c 'python3 demo/rtk.py'
#echo "[INFO] RTK running!"