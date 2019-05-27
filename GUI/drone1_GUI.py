# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'drone_GUI.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!



import roslib
import rospy
from PyQt4 import QtCore, QtGui
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
from math import radians

from ardrone_autonomy.msg import Navdata  # for receiving navdata feedback
from nav_msgs.msg import Odometry

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8


    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

#x=0
#y=0


class Ui_Form(object):
    def setupUi(self, Form):
	# self.Form = For
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(835, 565)
        self.Forward = QtGui.QPushButton(Form)
        self.Forward.setGeometry(QtCore.QRect(100, 20, 99, 27))
        self.Forward.setObjectName(_fromUtf8("Forward"))
        self.Right = QtGui.QPushButton(Form)
        self.Right.setGeometry(QtCore.QRect(200, 50, 99, 27))
        self.Right.setObjectName(_fromUtf8("Right"))
        self.Left = QtGui.QPushButton(Form)
        self.Left.setGeometry(QtCore.QRect(0, 50, 99, 27))
        self.Left.setObjectName(_fromUtf8("Left"))
        self.Backward = QtGui.QPushButton(Form)
        self.Backward.setGeometry(QtCore.QRect(100, 80, 99, 27))
        self.Backward.setObjectName(_fromUtf8("Backward"))
        self.Up = QtGui.QPushButton(Form)
        self.Up.setGeometry(QtCore.QRect(100, 120, 99, 27))
        self.Up.setObjectName(_fromUtf8("Up"))
        self.Down = QtGui.QPushButton(Form)
        self.Down.setGeometry(QtCore.QRect(100, 170, 99, 27))
        self.Down.setObjectName(_fromUtf8("Down"))
        self.TurnLeft = QtGui.QPushButton(Form)
        self.TurnLeft.setGeometry(QtCore.QRect(0, 140, 99, 27))
        self.TurnLeft.setObjectName(_fromUtf8("TurnLeft"))
        self.TurnRight = QtGui.QPushButton(Form)
        self.TurnRight.setGeometry(QtCore.QRect(200, 140, 99, 27))
        self.TurnRight.setObjectName(_fromUtf8("TurnRight"))
        self.TakeOff = QtGui.QPushButton(Form)
        self.TakeOff.setGeometry(QtCore.QRect(0, 190, 99, 27))
	self.TakeOff.setAutoRepeat(False)
        self.TakeOff.setObjectName(_fromUtf8("TakeOff"))
        self.Land = QtGui.QPushButton(Form)
        self.Land.setGeometry(QtCore.QRect(200, 190, 99, 27))
	self.Land.setAutoRepeat(False)
        self.Land.setObjectName(_fromUtf8("Land"))
        self.Stop = QtGui.QPushButton(Form)
        self.Stop.setGeometry(QtCore.QRect(100, 220, 99, 27))
        self.Stop.setObjectName(_fromUtf8("Stop"))
        self.label = QtGui.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(10, 260, 121, 17))
        self.label.setObjectName(_fromUtf8("label"))
        self.Straight = QtGui.QPushButton(Form)
        self.Straight.setGeometry(QtCore.QRect(120, 300, 99, 27))
        self.Straight.setObjectName(_fromUtf8("Straight"))
        self.Square = QtGui.QPushButton(Form)
        self.Square.setGeometry(QtCore.QRect(10, 300, 99, 27))
        self.Square.setObjectName(_fromUtf8("Square"))
        self.Circle = QtGui.QPushButton(Form)
        self.Circle.setGeometry(QtCore.QRect(10, 340, 99, 27))
        self.Circle.setObjectName(_fromUtf8("Circle"))
        self.Rectangle = QtGui.QPushButton(Form)
        self.Rectangle.setGeometry(QtCore.QRect(120, 340, 99, 27))
        self.Rectangle.setObjectName(_fromUtf8("Rectangle"))
        self.Hexagon = QtGui.QPushButton(Form)
        self.Hexagon.setGeometry(QtCore.QRect(120, 380, 99, 27))
        self.Hexagon.setObjectName(_fromUtf8("Hexagon"))
        self.Triangle = QtGui.QPushButton(Form)
        self.Triangle.setGeometry(QtCore.QRect(10, 380, 99, 27))
        self.Triangle.setObjectName(_fromUtf8("Triangle"))
        self.Vx = QtGui.QLabel(Form)
        self.Vx.setGeometry(QtCore.QRect(430, 100, 91, 17))
        self.Vx.setObjectName(_fromUtf8("Vx"))
        self.Vx_D = QtGui.QLabel(Form)
        self.Vx_D.setGeometry(QtCore.QRect(560, 100, 68, 17))
        self.Vx_D.setObjectName(_fromUtf8("Vx_D"))
        self.Vy = QtGui.QLabel(Form)
        self.Vy.setGeometry(QtCore.QRect(430, 130, 91, 17))
        self.Vy.setObjectName(_fromUtf8("Vy"))
        self.Vy_D = QtGui.QLabel(Form)
        self.Vy_D.setGeometry(QtCore.QRect(560, 130, 68, 17))
        self.Vy_D.setObjectName(_fromUtf8("Vy_D"))
        self.Vz = QtGui.QLabel(Form)
        self.Vz.setGeometry(QtCore.QRect(430, 160, 91, 17))
        self.Vz.setObjectName(_fromUtf8("Vz"))
        self.Vz_D = QtGui.QLabel(Form)
        self.Vz_D.setGeometry(QtCore.QRect(560, 160, 68, 17))
        self.Vz_D.setObjectName(_fromUtf8("Vz_D"))
        self.Altitude = QtGui.QLabel(Form)
        self.Altitude.setGeometry(QtCore.QRect(430, 190, 111, 17))
        self.Altitude.setObjectName(_fromUtf8("Altitude"))
        self.Alt_D = QtGui.QLabel(Form)
        self.Alt_D.setGeometry(QtCore.QRect(560, 190, 68, 17))
        self.Alt_D.setObjectName(_fromUtf8("Alt_D"))
        self.Battery = QtGui.QLabel(Form)
        self.Battery.setGeometry(QtCore.QRect(430, 220, 68, 17))
        self.Battery.setObjectName(_fromUtf8("Battery"))
        self.Bat_D = QtGui.QLabel(Form)
        self.Bat_D.setGeometry(QtCore.QRect(560, 220, 68, 17))
        self.Bat_D.setObjectName(_fromUtf8("Bat_D"))
        self.rotX = QtGui.QLabel(Form)
        self.rotX.setGeometry(QtCore.QRect(430, 250, 101, 17))
        self.rotX.setObjectName(_fromUtf8("rotX"))
        self.rotY = QtGui.QLabel(Form)
        self.rotY.setGeometry(QtCore.QRect(430, 280, 101, 17))
        self.rotY.setObjectName(_fromUtf8("rotY"))
        self.rotZ = QtGui.QLabel(Form)
        self.rotZ.setGeometry(QtCore.QRect(430, 310, 101, 17))
        self.rotZ.setObjectName(_fromUtf8("rotZ"))
        self.rotX_D = QtGui.QLabel(Form)
        self.rotX_D.setGeometry(QtCore.QRect(560, 250, 68, 17))
        self.rotX_D.setObjectName(_fromUtf8("rotX_D"))
        self.rotY_D = QtGui.QLabel(Form)
        self.rotY_D.setGeometry(QtCore.QRect(560, 280, 68, 17))
        self.rotY_D.setObjectName(_fromUtf8("rotY_D"))
        self.rotZ_D = QtGui.QLabel(Form)
        self.rotZ_D.setGeometry(QtCore.QRect(560, 310, 68, 17))
        self.rotZ_D.setObjectName(_fromUtf8("rotZ_D"))

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))
        self.Forward.setText(_translate("Form", "Forward", None))
        self.Right.setText(_translate("Form", "Right", None))
        self.Left.setText(_translate("Form", "Left", None))
        self.Backward.setText(_translate("Form", "Backward", None))
        self.Up.setText(_translate("Form", "Up", None))
        self.Down.setText(_translate("Form", "Down", None))
        self.TurnLeft.setText(_translate("Form", "TurnLeft", None))
        self.TurnRight.setText(_translate("Form", "TurnRight", None))
        self.TakeOff.setText(_translate("Form", "Takeoff", None))
        self.Land.setText(_translate("Form", "Land", None))
        self.Stop.setText(_translate("Form", "Stop", None))
        self.label.setText(_translate("Form", "Trajectories", None))
        self.Straight.setText(_translate("Form", "Straight", None))
        self.Square.setText(_translate("Form", "Square", None))
        self.Circle.setText(_translate("Form", "Circle", None))
        self.Rectangle.setText(_translate("Form", "Rectangle", None))
        self.Hexagon.setText(_translate("Form", "Hexagon", None))
        self.Triangle.setText(_translate("Form", "Triangle", None))
        self.Vx.setText(_translate("Form", "Vx  (mm/s)", None))
        self.Vx_D.setText(_translate("Form", "Vx_D", None))
        self.Vy.setText(_translate("Form", "Vy  (mm/s)", None))
        self.Vy_D.setText(_translate("Form", "Vy_D", None))
        self.Vz.setText(_translate("Form", "Vz  (mm/s)", None))
        self.Vz_D.setText(_translate("Form", "Vz_D", None))
        self.Altitude.setText(_translate("Form", "Altitude (cm)", None))
        self.Alt_D.setText(_translate("Form", "Alt_D", None))
        self.Battery.setText(_translate("Form", "Battery %", None))
        self.Bat_D.setText(_translate("Form", "Bat_D", None))
        self.rotX.setText(_translate("Form", "rotX (degree)", None))
        self.rotY.setText(_translate("Form", "rotY (degree)", None))
        self.rotZ.setText(_translate("Form", "rotZ (degree)", None))
        self.rotX_D.setText(_translate("Form", "rotX_D", None))
        self.rotY_D.setText(_translate("Form", "rotY_D", None))
        self.rotZ_D.setText(_translate("Form", "rotZ_D", None))


class Widget(QtGui.QWidget, Ui_Form):
    takeoff_pub = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=10)
    land_pub = rospy.Publisher("ardrone/land", Empty, queue_size=10)

    # Allow the controller to publish to the /cmd_vel topic and thus control the drone
    velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    def __init__(self, parent=None):
        
        QtGui.QWidget.__init__(self, parent)
        self.setupUi(self)

        self.TakeOff.clicked.connect(self.takeOff)
        self.Land.clicked.connect(self.land)
        
        self.Forward.clicked.connect(self.forward)
        self.Backward.clicked.connect(self.backward)
        self.Left.clicked.connect(self.left)
        self.Right.clicked.connect(self.right)
        self.Up.clicked.connect(self.up)
        self.Down.clicked.connect(self.down)
        self.TurnLeft.clicked.connect(self.turnLeft)
        self.TurnRight.clicked.connect(self.turnRight)
        self.Stop.clicked.connect(self.stop)
        self.Square.clicked.connect(self.square)
        self.Circle.clicked.connect(self.circle)
        self.Straight.clicked.connect(self.straight)
        self.Rectangle.clicked.connect(self.rectangle)
        self.Hexagon.clicked.connect(self.hexagon)
        self.Triangle.clicked.connect(self.triangle)
    
    def ReceiveNavdata(self, data):
        battery = data.batteryPercent  # self.batteryPercent
        state = data.state
        altitude = data.altd
        vx = data.vx
        vy = data.vy
        vz = data.vz
        rotX = data.rotX
        rotY = data.rotY
        rotZ = data.rotZ

        print("%f %f"%(rotY,vx))  
        rospy.sleep(10)
        print("%f,%f,%f,%f,%f,%f,%f,%f,%f" % (battery, state, altitude, vx, vy, vz, rotX, rotY, rotZ))
        
        print("data_start")

        self.Bat_D.setText(str(battery))
        
        self.Vx_D.setText(str(vx))
        self.Vy_D.setText(str(vy))
        self.Vz_D.setText(str(vz))
        self.Alt_D.setText(str(altitude))
        self.State_D.setText(str(state))
        self.rotX_D.setText(str(rotX))
        self.rotY_D.setText(str(rotY))
        self.rotZ_D.setText(str(rotZ))
        
        print("data_end")
        
        if state == 2:
           self.State_D.setText("Landed")
        elif state == 3 or 7:
            self.State_D.setText("Flying")
        elif state == 6:
            self.State_D.setText("Taking-Off")
        elif state == 8:
            self.State_D.setText("Landing")
        elif state == 4:
            self.State_D.setText("Hovering") 
        elif state ==1:
            self.State_D.setText("inited")
        elif state ==9:
            self.State_D.setText("Looping")
             

    def takeOff(self):
        self.takeoff_pub.publish(Empty())
        print("Take-Off Called")

    def land(self):
        self.land_pub.publish(Empty())
        print("Land called")

    def stop(self):
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.angular.z = 0.0
        self.velocity_pub.publish(self.twist)
        print("inside stop")

    def forward(self):
        # fw.write("inside--forward\n")
        self.twist = Twist()
        self.twist.linear.x = 0.1
        self.velocity_pub.publish(self.twist)
        print("inside forward")

    def backward(self):
        # fw.write("inside--backward\n")
        self.twist = Twist()
        self.twist.linear.x = -0.1
        self.velocity_pub.publish(self.twist)
        print("inside backward")

    def right(self):
        # fw.write("inside--right\n")
        self.twist = Twist()
        self.twist.linear.y = 0.1
        self.velocity_pub.publish(self.twist)

    def left(self):
        # fw.write("inside--Left\n")
        self.twist = Twist()
        self.twist.linear.y = -0.1
        self.velocity_pub.publish(self.twist)

    def turnLeft(self):
        # fw.write("inside--rotationPositive\n")
        self.twist = Twist()
        self.twist.angular.z = 0.1
        self.velocity_pub.publish(self.twist)

    def turnRight(self):
        # fw.write("inside--RotationNegative\n")
        self.twist = Twist()
        self.twist.angular.z = -0.1
        self.velocity_pub.publish(self.twist)

    def up(self):
        # fw.write("inside_flyup\n")
        self.twist = Twist()
        self.twist.linear.z = 0.1
        self.velocity_pub.publish(self.twist)

    def down(self):
        # fw.write("inside_flydown")
        self.twist = Twist()
        self.twist.linear.z = -0.1
        self.velocity_pub.publish(self.twist)


    def odometryCallback(self,odoData):
    #position 
    
    
        x= odoData.pose.pose.position.x
        y= odoData.pose.pose.position.y
        z= odoData.pose.pose.position.z


        print ("%f %f %f"%(x,y,z))
        rospy.sleep(10)
        
    def uploadFile(self):
        print "hello its upload file"   

    def square(self):
        print ("inside the square")
        self.twist = Twist()
        fRead = open("squarenew.txt","r")
        for line in fRead.readlines():
            values = line.split(',')
            print values
            print values[0] + ',' + values[1] + ',' + values[2] + ',' + values[3] + ',' + values[4]
            x = float(values[0])
            y = float(values[1])
            z = float(values[2])
            aT = radians(float(values[3]))
            print aT

            self.twist.linear.x = x
            self.twist.linear.y = y
            self.twist.linear.z = z
            self.twist.angular.z = aT

            self.velocity_pub.publish(self.twist)
            if self.twist.angular.z == 0:
                print "Moving"
            else:
                print "Turing"
            rospy.sleep(4)#tB)
    def straight(self):
        print("inside straight move")
        self.twist = Twist()
        fRead = open("line.txt","r")
        for line in fRead.readlines():
            values = line.split(',')
            print values
            print values[0] + ',' + values[1] + ',' + values[2] + ',' + values[3] + ',' + values[4]
            x = float(values[0])
            y = float(values[1])
            z = float(values[2])
            aT = radians(float(values[3]))
            print aT
            tB = 15#float(values[4])
            
            self.twist.linear.x = x
            self.twist.linear.y = y
            self.twist.linear.z = z
            self.twist.angular.z = aT

            self.velocity_pub.publish(self.twist)
            if self.twist.angular.z == 0:
                print "Moving"
            else:
                print "Turing"
            rospy.sleep(0.5)#tB)
        
    def hexagon(self):
        print ("inside the square")
        self.twist = Twist()
        fRead = open("hexagon.txt","r")
        for line in fRead.readlines():
            values = line.split(',')
            print values
            print values[0] + ',' + values[1] + ',' + values[2] + ',' + values[3] + ',' + values[4]
            x = float(values[0])
            y = float(values[1])
            z = float(values[2])
            aT = radians(float(values[3]))
            print aT
            #tB = float(values[4])#float(values[4])
            
            self.twist.linear.x = x
            self.twist.linear.y = y
            self.twist.linear.z = z
            self.twist.angular.z = aT

            self.velocity_pub.publish(self.twist)
            if self.twist.angular.z == 0:
                print "Moving"
            else:
                print "Turing"
            rospy.sleep(4)#tB)

    def rectangle(self):
        print ("inside the square")
        self.twist = Twist()
        fRead = open("rectangle.txt","r")
        #fRead = open("rectangle.txt","r")
        for line in fRead.readlines():
            values = line.split(',')
            print values
            print values[0] + ',' + values[1] + ',' + values[2] + ',' + values[3] + ',' + values[4]
            x = float(values[0])
            y = float(values[1])
            z = float(values[2])
            aT = radians(float(values[3]))
            print aT
            tB = float(values[4])#float(values[4])
            
            self.twist.linear.x = x
            self.twist.linear.y = y
            self.twist.linear.z = z
            self.twist.angular.z = aT

            self.velocity_pub.publish(self.twist)
            if self.twist.angular.z == 0:
                print "Moving"
            else:
                print "Turing"
            rospy.sleep(tB)#tB)

    def circle(self):
        print("inside circle")
        self.twist = Twist()
        fRead = open("circle.txt","r")
        for line in fRead.readlines():
            values = line.split(',')
            print values
            print values[0] + ',' + values[1] + ',' + values[2] + ',' + values[3] + ',' + values[4]
            x = float(values[0])
            y = float(values[1])
            z = float(values[2])
            aT = radians(float(values[3]))
            print aT
            tB = 15#float(values[4])
            
            self.twist.linear.x = x
            self.twist.linear.y = y
            self.twist.linear.z = z
            self.twist.angular.z = aT

            self.velocity_pub.publish(self.twist)
            if self.twist.angular.z == 0:
                print "Moving"
            else:
                print "Turing"
            rospy.sleep(0.5)#tB)

    def triangle(self):
        print ("inside the square")
        self.twist = Twist()
        fRead = open("triangle.txt","r")
        for line in fRead.readlines():
            values = line.split(',')
            print values
            print values[0] + ',' + values[1] + ',' + values[2] + ',' + values[3] + ',' + values[4]
            x = float(values[0])
            y = float(values[1])
            z = float(values[2])
            aT = radians(float(values[3]))
            print aT
            
            self.twist.linear.x = x
            self.twist.linear.y = y
            self.twist.linear.z = z
            self.twist.angular.z = aT

            self.velocity_pub.publish(self.twist)
            if self.twist.angular.z == 0:
                print "Moving"
            else:
                print "Turing"
            rospy.sleep(4)#tB)

if __name__ == "__main__":
    import sys

    rospy.init_node('ardrone_control_node', anonymous=True)

    try:
                app
    except:
        
        app = QtGui.QApplication(sys.argv)
        
        Form = Widget()

        rospy.Subscriber('/ardrone/navdata', Navdata, lambda data : Form.ReceiveNavdata(data))
        rospy.Subscriber('/ardrone/odometry', Odometry,lambda odoData : Form.odometryCallback(odoData))
         

        Form.setWindowTitle("ROS_GUI_AR")
        Form.setGeometry(0, 0, 1200, 800)
        Form.show()
        sys.exit(app.exec_())


#if __name__ == "__main__":
#    import sys
#    app = QtGui.QApplication(sys.argv)
#    Form = QtGui.QWidget()
#    ui = Ui_Form()
#    ui.setupUi(Form)
#    Form.show()
#    sys.exit(app.exec_())

