#!/usr/bin/env python3
import rospy
import socket
#from parse.msg import UdpMsg
from std_msgs.msg import Header


class RadarConn:
    def __init__(self, tunnel_id=0,
                 host_ip="192.168.66.166",
                 radar_ip="192.168.66.113",
                 multicast_group="224.0.2.2",
                 multicast_port=42102):
        self.tunnel_id = tunnel_id
        self.host_ip = host_ip
        self.ip = radar_ip
        self.group = multicast_group
        self.port = multicast_port
        self.buffer_size = 36000

        self.socket_inst = None

        self.pub = None

        self.init_socket()

        # default node name: udp_node0
        self.node_name = "udp_node%d" % self.tunnel_id
        # default pub name: udp_pub0
        self.pub_name = "udp_pub%d" % self.tunnel_id

        rospy.init_node(self.node_name)
        #self.pub = rospy.Publisher(self.pub_name, UdpMsg, queue_size=5)

    def init_socket(self):
        if self.host_ip is None:
            return False
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 10)
        sock.bind(("", self.port))
        sock.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP,
                        socket.inet_aton(self.group) + socket.inet_aton(self.host_ip))
        self.socket_inst = sock
        return True

    def publish(self):
        while not rospy.is_shutdown():
            try:
                data, addr = self.socket_inst.recvfrom(self.buffer_size)
                print(addr[0])
                if addr[0] != self.ip:
                    continue
                msg = UdpMsg()
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
                msg.ip = str(addr[0])
                msg.data = data
                self.pub.publish(msg)
            except socket.error as socket_error:
                rospy.logerr(socket_error)
            except Exception as exception_err:
                rospy.logerr(exception_err)
        rospy.loginfo('Closing a connection to port ' + str(self.port))
        self.socket_inst.close()

    def startup(self):
        try:
            self.publish()
        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    radar_connection = RadarConn(tunnel_id=0)
    radar_connection.startup()
