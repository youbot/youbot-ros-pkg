#!/usr/bin/env python
# Initial code created by Graylin Trevor Jay (tjay@cs.brown.edu) an published under Crative Commens Attribution license.
# addition for signal interrupt by Koen Buys

import roslib; roslib.load_manifest('youbot_oodl')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty, signal

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""

moveBindings = {
#		     x,y,tetha ratio
		'i':(1,0,0), 	# forwards
		'o':(1,0,-1), 	# forwards + rotation right
		'j':(0,1,0), 	# left
		'l':(0,-1,0),	# right
		'u':(1,0,1), 	# forwards + rotation left
		',':(-1,0,0), 	# backward
		'.':(0,0,-1), 	# turn right on spot
		'm':(0,0,1), 	# turn left on spot
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

class TimeoutException(Exception): 
    pass 

def getKey():
    def timeout_handler(signum, frame):
        raise TimeoutException()
    
    old_handler = signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(1) #this is the watchdog timing
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    try:
       key = sys.stdin.read(1)
       #print "Read key"
    except TimeoutException:
       #print "Timeout"
       return "-"
    finally:
       signal.signal(signal.SIGALRM, old_handler)

    signal.alarm(0)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 0.1
turn = 0.1

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist)
	rospy.init_node('teleop_twist_keyboard')

	x = 0
	y = 0
	th = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				th = moveBindings[key][2]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed 
			twist.linear.y = y*speed 
			twist.linear.z = 0

			twist.angular.x = 0 
			twist.angular.y = 0
			twist.angular.z = th*turn
			pub.publish(twist)

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


