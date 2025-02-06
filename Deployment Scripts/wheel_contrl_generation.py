# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
print('1')
import pickle
import socket
import hid


for device in hid.enumerate():
    print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x}{device['product_string']}")

print(2)
UDP_IP = "192.168.0.35"
UDP_PORT = 20000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(3)
serverAddressPort = (UDP_IP, UDP_PORT)
message1 = "HI Server"
message1 = pickle.dumps(message1)
sock.sendto(message1, serverAddressPort)
print(4)
#sock.sendto("HI Server".encode("utf-8"), serverAddressPort)
data, addr = sock.recvfrom(1024)
#print(data.decode("utf-8"), "was recieved")
#print(addr)
data = pickle.loads(data)
print(data)
print(addr)
xbox_controller = [0x045e, 0x028e]
wheel_controller = [0x0079, 0x189c]
gamepad = hid.device()
gamepad.open(*xbox_controller)
gamepad.set_nonblocking(True)

while True:
    report = gamepad.read(64)
    
    if report:
        print(report)
        control = pickle.dumps(report)
        sock.sendto(control, serverAddressPort)
        
    