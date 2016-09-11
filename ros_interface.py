#! /usr/bin/python

import serial
import socket
import rospy
from std_msgs.msg import Float32
from math import atan2, pi, sqrt
rospy.init_node("disc_interface")


pub_alt = rospy.Publisher('altitude', Float32, queue_size=10)
pub_gxy = rospy.Publisher('gyro_xy', Float32, queue_size=10)


class atan2_pub:
    def __init__(self):
        self.pub = rospy.Publisher('z_rotation', Float32, queue_size=10)

    def add_value(self, values):
        angle = atan2(values[1], values[0])
        self.pub.publish(angle/pi*180)


class bump_detection:
    def __init__(self, c1_threshold=0.1, c2_threshold=0.6, t1=0.5, t2=0.2):
        self.t1 = t1 # minimal time within C1
        self.t2 = t2  # max time between leaving C1 and entering C2
        self.c1_threshold = c1_threshold  # maximal deviation from 1g
        self.c2_threshold = c2_threshold  # miniminal deviation from g to trigger
        self.c1_entry = rospy.Time(0)
        self.c1_exit = rospy.Time(0)

    def add_value(self, v):
        dg = abs(v-1)
        # print dg
        # check if we are in C1:
        if dg < self.c1_threshold:
            # entering C1
            if self.c1_entry.to_sec() == 0:
                # rospy.loginfo("entering C1")
                self.c1_entry = rospy.Time.now()
        else:
            if self.c1_entry.to_sec() > 0:  # we have been in the corridor
                now = rospy.Time.now()
                if (now - self.c1_entry).to_sec() > self.t1:
                    # print "loaded"
                    #rospy.loginfo("long enough in C1")
                    self.c1_exit = now
            self.c1_entry = rospy.Time(0)

        if dg > self.c2_threshold:
            now = rospy.Time.now()
            if self.c1_exit.to_sec() > 0 and (now-self.c1_exit).to_sec() < self.t2:
                print ("TRIGGER %.3f %f" % ( (now-self.c1_exit).to_sec(), dg) )
                self.c1_exit = rospy.Time(0)


class running_mean:
    def __init__(self):
        self.N = 10
        self.sum = self.N
        self.l = [1]*self.N  # some initialization so that I don't have to care about startup
        self.pos = 0

    def add_value(self, v):
        if self.pos == self.N-1:
            self.pos = 0
        else:
            self.pos += 1

        self.sum -= self.l[self.pos]
        self.l[self.pos] = v
        self.sum += v
        return self.get_mean()

    def get_mean(self):
        return self.sum/self.N


class multi_pub:
    def __init__(self, prefix):
        self.prefix = prefix
        self.pub_1 = rospy.Publisher(self.prefix + '_x', Float32, queue_size=10)
        self.pub_2 = rospy.Publisher(self.prefix + '_y', Float32, queue_size=10)
        self.pub_3 = rospy.Publisher(self.prefix + '_z', Float32, queue_size=10)
        self.pub_full = rospy.Publisher(self.prefix, Float32, queue_size=10)
        self.pub_mean = rospy.Publisher(self.prefix + '_mean', Float32, queue_size=10)

        self.run_mean = running_mean()

    def add_value(self, spl):
        self.pub_1.publish(spl[0])
        self.pub_2.publish(spl[1])
        self.pub_3.publish(spl[2])
        if len(spl) > 3:
            self.pub_full.publish(spl[3])
            m = self.run_mean.add_value(spl[3])
            self.pub_mean.publish(m)


acc_mean = running_mean()
acc_bd = bump_detection()

gyro_bd = bump_detection(30, 600, 0.4, 0.2)

mp_g = multi_pub("gyro")
mp_a = multi_pub("accel")
atan2_pub_ = atan2_pub()



# selector for Serial Interface or UDP via WIFI
udp=True


if udp:
  UDP_IP = "192.168.43.162" # TODO: read local ip
  UDP_PORT = 8888
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.bind((UDP_IP, UDP_PORT))
else:
  ser = serial.Serial('/dev/ttyACM0', 115200)

while not rospy.is_shutdown():
  
    if udp:
      data, addr = sock.recvfrom(100) 
      sp = data.strip().split()
    else:
      sp = ser.readline().strip().split()
      
    print sp
    if sp[0] == 'Alt':
        pub_alt.publish(Float32(float(sp[1])))
        continue
    if sp[0] == 'Acc': # ax, ay, az, a_norm
        values = [float(s) for s in sp[1:]]
        mp_a.add_value(values)
        atan2_pub_.add_value(values)

    if sp[0] == 'G':  # gx, gy, gz, g_norm
        values = [float(s) for s in sp[1:]]
        mp_g.add_value(values)

        gyro_xy = sqrt(values[0]*values[0]+values[1]*values[1])
        pub_gxy.publish(gyro_xy)
        #gyro_bd.add_value(gyro_xy)

 
