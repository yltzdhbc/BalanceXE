# coding=utf-8

# 使用python2.7运行

import socket
import time
import struct


class ROBOT(object):
    UDP_IP = "192.168.1.201"
    UDP_PORT = 2222
    sock = 0
    wheel_base = 0.134  # m
    PI = 3.1415926


    def __init__(self):
        # 创建一个socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def sendCommand(self, OSCmessage, param):
        base = bytearray(OSCmessage)  # OSC message
        param = bytearray(struct.pack(">f", param))
        message = base + param
        self.sock.sendto(message, (self.UDP_IP, self.UDP_PORT))
        time.sleep(0.05)

    # Mode: 0 普通模式 , 1 专家模式
    def mode(self, value=0.0):
        print 'BROBOT Mode:', value
        if (value != 1):
            value = 0.0
        self.sendCommand(b'/1/toggle1/\x00\x00,f\x00\x00', value)

    # 油门指令 范围 [-1.0 to 1.0] 正数位为向前
    def throttle(self, value=0.0):
        print "ROBOT Throttle:", value
        value = (value + 1.0) / 2.0  # 将 values 转换到 0.0-1.0
        self.sendCommand(b'/1/fader1\x00\x00\x00,f\x00\x00', value)

    # 转向指令 范围 [-1.0 to 1.0] 正数为向右转
    def steering(self, value=0.0):
        print "ROBOT Steering:", value
        value = (value + 1.0) / 2.0  # 将 values 转换到 0.0-1.0
        self.sendCommand(b'/1/fader2\x00\x00\x00,f\x00\x00', value)

    # 舵机1命令. 值为 0 or 1 (默认)
    def servo1(self, value=0.0):
        print "ROBOT Servo1:", value
        # if (value!=1):value=0.0
        self.sendCommand(b'/1/push1\x00\x00,f\x00\x00', value)

    # 舵机2命令. 值为 [0.0 to 1.0]
    def servo2(self, value=0.0):
        print "ROBOT Servo2:", value
        self.sendCommand(b'/1/xy1\x00,f,f\x00\x00', value)

    # 精确移动指令 速度，电机1脉冲数，电机2的脉冲数 speed, steps1, steps2
    def move(self, speed, steps1, steps2):
        print "ROBOT MOVE", speed, steps1, steps2
        base = bytearray(b'/1/move\x00\x00\x00')
        param1 = bytearray(struct.pack("h", speed))
        param2 = bytearray(struct.pack("h", steps1))
        param3 = bytearray(struct.pack("h", steps2))
        message = base + param1 + param2 + param3
        self.sock.sendto(message, (self.UDP_IP, self.UDP_PORT))

    # 直线指令 速度 方向 距离
    def line(self, speed, direction, distance):
        print "ROBOT line:", speed, direction, distance
        setps = 11428 * distance
        if direction == 0:
            self.move(speed, setps, setps)
        elif direction == 1:
            self.move(speed, -setps, -setps)

    def turn(self, speed, direction, angle):
        print "ROBOT turn:", speed, direction, angle
        setps = (4720 - 40) / 360 * angle
        if direction == 0:
            self.move(speed, -setps, setps)
        elif direction == 1:
            self.move(speed, setps, -setps)

    def Circular_Arc(self, speed, direction, angle, radius):
        print "ROBOT Arc:", speed, direction, angle, radius
        if direction == 0:
            left_setps = angle * self.PI * \
                         (radius - self.wheel_base / 2) / 180 * 11428
            right_setps = angle * self.PI * \
                          (radius + self.wheel_base / 2) / 180 * 11428
        elif direction == 1:
            left_setps = angle * self.PI * \
                         (radius + self.wheel_base / 2) / 180 * 11428
            right_setps = angle * self.PI * \
                          (radius - self.wheel_base / 2) / 180 * 11428
        self.move(speed, int(left_setps), int(right_setps))
