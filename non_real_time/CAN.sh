#sudo ip link set can0 down
#sudo ip link set can0 type can bitrate 500000
#sudo ip link set can0 up


sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
#sudo ip link set can0 type can bitrate 250000 #restart-ms 100
sudo ip link set can0 up
