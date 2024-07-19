sudo modprobe 8021q  # 加载到内核




sudo ip link add link enp3s0 name enp3s0.19 type vlan id 19

sudo ip link

sudo ip -d link show enp3s0.19

sudo ip addr add 10.13.1.16/24 brd 10.13.1.255 dev enp3s0.19

sudo ip link set dev enp3s0.19 up

sudo route add -net 224.0.2.2 netmask 255.255.255.255 enp3s0.19

route -n


