First， modify IPV4 according to the radar configuration instructions.

Then,install vlan：
sudo apt-get install vlan 
sudo modprobe 8021q  

Run 11.sh after each startup

Running instructions：

rosrun ars548_process ars548_process_node 

roslaunch ars548_process vis_ars548.launch 
