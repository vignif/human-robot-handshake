#!/usr/bin/env python
## 03_july_18
import roslib
import rospy
import std_msgs
import numpy as np
import csv
import random
from std_msgs.msg import Float32MultiArray
from qb_interface import msg 
from std_msgs.msg import Empty
from std_msgs.msg import String
import curses
from datetime import datetime

##include for emergency break
import sys, termios, atexit
from select import select
import time

tstart = datetime.now()


## input char from keyboard

# save the terminal settings
fd = sys.stdin.fileno()
new_term = termios.tcgetattr(fd)
old_term = termios.tcgetattr(fd)

# new terminal setting unbuffered
new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)

# switch to normal terminal
def set_normal_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, old_term)

# switch to unbuffered terminal
def set_curses_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, new_term)

def putch(ch):
    sys.stdout.write(ch)

def getch():
    return sys.stdin.read(1)

def getche():
    ch = getch()
    putch(ch)
    return ch

def kbhit():
    dr,dw,de = select([sys.stdin], [], [], 0)
    return dr <> []

def emergency(pos):
    if getch() == 'x':
#         print "EXIT"
        rospy.sleep(0.1)
        pos.publish([0])
        rospy.signal_shutdown("Emergency")
        rospy.on_shutdown(h)
        
def h():
    pass
#     print "Emergency shutdown!"


def main():
    pub = rospy.Publisher("qb_class/hand_ref", msg.handRef, queue_size= 1000)
    rospy.init_node("steps", anonymous=True)
    rospy.set_param('/stiffness',1.0)
    #Class for managing subscription and file with relevant data
    Mg=manage_cb()
    
    # 22 different values
#     steps=range(6000,17000,250)
#     
#     #initial seed to make a pseudorandom shuffle
#     SEED = 448 
#     random.seed(SEED)
#     random.shuffle(steps)
#for one step experiment uncomment the next row
    steps=[0, 5000, 8500, 9000, 10000, 11000, 12000]
    rate=rospy.Rate(100) #100 Hz
    index=0
    ln=len(steps)
    ln=float(ln)
    print "Let's find q0!"
    for i in steps:
        print "Experiment status: " + str(round(index/ln,2)*100) + "%" + "\t \t closure: " + str(i)

        for a in range(0,200,1):
            pub.publish([i])
            Mg.sent_pos(i)
            ## write a row in the file at each iteration
            Mg.save() 
            rate.sleep()
            ## if a key is hitted, check if its the safe key 'x'
            ## send a safe position '0' and stop the experiment
            if kbhit():
                emergency(pub)
                halt=True
                break
            if rospy.is_shutdown():
                break
        if rospy.is_shutdown():
                break
        index+=1
                
            
    rospy.sleep(1)
    pub.publish([0])
    tend = datetime.now()
    print "q0: " + str(i) 
    print "handsize code: " +   str(steps.index(i)-1)
#     print tend-tstart
    print "END"
        

 
class manage_cb:
    def __init__(self):
        
        self.sub_current = rospy.Subscriber("qb_class/hand_measurement", msg.handPos , self.current_cb)
        self.sub_sensors = rospy.Subscriber("sensors_FSR", Float32MultiArray, self.sens_cb)
        self.sub_realpos = rospy.Subscriber("qb_class/hand_measurement", msg.handPos , self.realpos_cb)
        self.FSR=[0,0,0,0]
        self.current=0
        self.realpos=0
    
    def realpos_cb(self,handPos):
        self.realpos=handPos.closure[0]
        return self.realpos
    
    def current_cb(self,handPos):
        self.current=handPos.closure[2]
        return self.current
    
    def sens_cb(self,msg):
        self.FSR=np.array(msg.data)
        self.FSR=np.around(self.FSR, decimals=2)
        return self.FSR
    
    def sent_pos(self,pos):
        self.sentpos=pos
        return self.sentpos
    
    def save(self):
        ##File structure
        ## [FSR1, FSR2, FSR3, FSR4, current, realpos, sentpos]
                
        tosave = np.append(self.FSR, np.around(self.current , decimals=2))
        tosave = np.append(tosave, np.around(self.realpos , decimals=2))
        tosave = np.append(tosave, self.sentpos)
        dir="/home/francesco/ros_ws_clean/ctrl/"
        name="find_q0"
       
        with open(dir + name + ".csv" , 'a') as f:
            writer = csv.writer(f)
            writer.writerow(tosave)         
        #print tosave

 
        
if __name__=="__main__":
    try:
        atexit.register(set_normal_term)
        set_curses_term()
        main()
    except rospy.ROSInterruptException:
        pass
