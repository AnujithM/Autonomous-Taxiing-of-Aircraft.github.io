#!/usr/bin/python
print("0")
import pickle
print("1")
import socket
import rospy
from std_msgs.msg import Int32MultiArray
Ip = "192.168.0.35"
port = 20000
print("inside test")
if __name__=="__main__":
    control_pub = rospy.Publisher('control_info', Int32MultiArray, queue_size=10)
    rospy.init_node('control_publisher', anonymous=False)
    control = Int32MultiArray()
    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    UDPServerSocket.bind((Ip, port))
    data_, addr = UDPServerSocket.recvfrom(1024)  # Communication from lane detection file
    data_ = pickle.loads(data_)
    print(data_)
    # print(data_.decode('utf-8'))
    message1 = [1, 2, 3]
    message1 = pickle.dumps(message1)
    UDPServerSocket.sendto(message1, addr)
    # UDPServerSocket.sendto("Hi Client".encode('utf-8'), addr)
    while True:
        print("Inside True")
        data_,addr = UDPServerSocket.recvfrom(1024)  #Communication from lane detection file
        control.data = pickle.loads(data_)
        control_pub.publish(control)
        # with open("control", "wb") as f:
        #     f.write(data_)
        # with open("control", "rb") as f:
        #     b = pickle.load(f)
        # print(b[1])
        # data_ = pickle.loads(data_)
        # print(data_.decode('utf-8'))
        # print(data_)

