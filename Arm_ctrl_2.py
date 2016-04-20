#!/usr/bin/env python
'''
Created on November 20, 2016

'''
import threading
import serial
import struct
from cStringIO import StringIO
import time
import rospy
from std_msgs.msg import String

def _OnLineReceived(line):
    print line
class SerialDataGateway(object):
    '''
    Helper class for receiving lines from a serial port
    '''

    def __init__(self, port="/dev/ttyACM0", baudrate=115200, lineHandler=_OnLineReceived):
        '''
        Initializes the receiver class.
        port: The serial port to listen to.
        receivedLineHandler: The function to call when a line was received.
        '''
        self._Port = port
        self._Baudrate = baudrate
        self.ReceivedLineHandler = lineHandler
        self._KeepRunning = False

    def Start(self):
        try:
            self._Serial = serial.Serial(port=self._Port, baudrate=self._Baudrate, timeout=1)
            time.sleep(1)   # Wait Arduino reset! This is fucking important!
        except:
            rospy.loginfo("SERIAL PORT Start Error")
            raise
        self._KeepRunning = True
        self._ReceiverThread = threading.Thread(target=self._Listen)
        self._ReceiverThread.setDaemon(True)
        self._ReceiverThread.start()

    def Stop(self):
        rospy.loginfo("Stopping serial gateway")
        self._KeepRunning = False
        time.sleep(.1)
        try:
            self._Serial.close()
        except:
            rospy.loginfo("SERIAL PORT Stop Error")
            raise

    def _Listen(self):
        stringIO = StringIO()
        while self._KeepRunning:
            try:
                data = self._Serial.read()
            except:
                rospy.loginfo("SERIAL PORT Listen Error")
                raise
            if data == '\r':
                pass
            if data == '\n':
                self.ReceivedLineHandler(stringIO.getvalue())
                stringIO.close()
                stringIO = StringIO()
            else:
                stringIO.write(data)

    def Write(self, data):
        #AttributeError: 'SerialDataGateway' object has no attribute '_Serial'
        try:
            self._Serial.write(data)
        except AttributeError:
            rospy.loginfo("SERIAL PORT Write Error")
            raise

class Arm(object):

    def __init__(self,freq,omega):
        rospy.init_node('Arm_ctrl')
        rospy.Subscriber("Arm_cmd", String, self.callback)
        self.serial = SerialDataGateway("/dev/ttyACM0", 115200)
        self.serial.Start()
        self.freq = freq
        self.omega = omega
        self.iniPosition = [90.0,90.0,90.0,20.0,90.0,40.0]
        self.position = self.iniPosition
        self.trajectory = [[],[],[],[],[],[]]
        self.state = 'idle'
        self.working = False
        rospy.loginfo('Eli arm start')
    def callback(self,data):
        cmd = data.data
        rospy.loginfo('Received:%s'%cmd)
        if cmd == 'poke':
            self.trajectory = self.poke()
            rospy.loginfo('Start poke')
            self.state = 'poke'
        elif cmd == 'grab':
            self.trajectory = self.grab()
            rospy.loginfo('Start grab')
            self.state = 'grab'
        elif cmd == 'putback':
            self.trajectory = self.putback()
            rospy.loginfo('Start putback')
            self.state = 'putback'
        elif cmd == 'initialize':
            self.trajectory = self.initialize()
            rospy.loginfo('Initialize')
            self.state = 'initialize'
        else:
            self.state = 'idle'
        self.working = True
    def calTrajectory(self,start,end):
        diff = [abs(i-j) for i,j in zip(start,end)]
        maxTheta = max(diff)
        duration = maxTheta/self.omega
        n = duration*self.freq
        n = n if n>0 else 1    # In case of ZeroDivision
        trajectory = []
        for i in range(len(start)):   # range 6 (axis)
            single_axis_traj = self.linearInterpolation(start[i],end[i],n)
            trajectory.append(single_axis_traj)
        return trajectory
    def linearInterpolation(self,start,end,n):
        diff = end - start
        delta = diff/n
        theta = start
        trajectory = []
        for i in range(int(n)):
            theta += delta
            trajectory.append(theta)
        return trajectory
    def calMultiPointTrajectory(self,*point):
        trajectory = [[],[],[],[],[],[]]
        for i in range(len(point)-1):
            portion = self.calTrajectory(point[i],point[i+1])
            trajectory = [i+j for i,j in zip(trajectory,portion)]
        return trajectory

    def motionGen(self,motion,mode,angle=90.0):
        motionDic = {'grabPosition':[90.0,48.0,172.0,58.0,90.0,40.0],\
                  'stretch':[90.0,80.0,158.0,43.0,90.0,40.0],\
                  'put':[90.0,80.0,158.0,43.0,90.0,40.0],\
                  'initialPosition':[90.0,90.0,90.0,20.0,90.0,40.0]
        }
        jointPosition = motionDic[motion]
        if 0 in mode:   # Initial state
            jointPosition[4] = 90.0
            jointPosition[5] = 40.0
        if 1 in mode:  # Catch
            jointPosition[5] = 120.0
        if 2 in mode:   # Release
            jointPosition[5] = 40.0
        if 3 in mode:    # Rotate claw
            jointPosition[4] = 180.0
        if 4 in mode:  # Rotate claw back
            jointPosition[4] = 90.0
        if 5 in mode:
            jointPosition[0] = angle
        return jointPosition


    def poke(self,startPoint=None):
        startPosition = startPoint if startPoint else self.position
        p1 = self.motionGen('grabPosition',[1,5],120)
        p2 = self.motionGen('stretch',[1,5],120)
        return self.calMultiPointTrajectory(startPosition,p1,p2)

    def grab(self,startPoint=None):
        startPosition = startPoint if startPoint else self.position
        p1 = self.motionGen('grabPosition',[3])
        p2 = self.motionGen('stretch',[3])
        p3 = self.motionGen('stretch',[1,3])
        p4 = self.motionGen('grabPosition',[1,3])
        p5 = self.motionGen('initialPosition',[1,4])
        return self.calMultiPointTrajectory(startPosition,p1,p2,p3,p4,p5)

    def putback(self,startPoint=None):
        startPosition = startPoint if startPoint else self.position
        p1 = self.motionGen('grabPosition',[1,3])
        p2 = self.motionGen('stretch',[1,3])
        p3 = self.motionGen('stretch',[2,3])
        p4 = self.motionGen('grabPosition',[2,3])
        p5 = self.motionGen('initialPosition',[2,4])
        return self.calMultiPointTrajectory(startPosition,p1,p2,p3,p4,p5)

    def initialize(self):
        return self.calMultiPointTrajectory(self.position,self.iniPosition)

    def publishArmCmd(self):
        transposed_trajectory = zip(*self.trajectory)
        for jointPosition in transposed_trajectory:
            self.position = jointPosition  # Record the instant position
            time.sleep(1.0/self.freq)
            serialData = self.cvt2SerialData(jointPosition)
            self.serial.Write(serialData)
        self.working = False    # Job done
        rospy.loginfo('Job done')

    def cvt2SerialData(self,data):  # Data refer to [Jj1,j2,j3,j4,j5,j6]
        data = map(int,data)
        output = []
        output.append(0xc9) # start byte
        output += data     # fill in joint angle
        output += [55,66]  # fucking blue cat's head and neck DOF
        check_sum = 0
        for number in output[1:]:    # check sum
            check_sum ^= int(number)
        output.append(check_sum) # end byte
        output.append(0xca) # end byte
        #rospy.loginfo(output)    # for testing
        string = ''
        for i in output:
            string += struct.pack('!B',i)  # convert to binary char
        return string

    def start(self):
        if self.working:
            self.publishArmCmd()
        else:
            self.state = 'idle'
    def showInfo(slef):
        print 'state:%s %s'%(self.state,self.position)


if __name__ == '__main__':
    eliArm = Arm(freq=30.0,omega=50.0)
    try:
        while not rospy.is_shutdown():
            eliArm.start()
    finally:
        eliArm.serial.Stop()
