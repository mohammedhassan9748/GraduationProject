#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Use the keyboard layout:
   Q    W    E
   A    S    D
   Z    X    C

W/X : increase/decrease linear velocity (0.44 m/s max)
A/D : increase/decrease angular velocity (3.2 rad/s max)
S : stop

CTRL-C to quit
"""

moveBindings = {
    'w':(1,0),
    'e':(1,-1),
    'a':(0,1),
    'd':(0,-1),
    'q':(1,1),
    'x':(-1,0),
    'c':(-1,1),
    'z':(-1,-1),
    's':(0,0),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('robot_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    speed = 0.0
    turn = 0.0
    x = 0
    th = 0
    status = 0

    try:
        print(msg)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            else:
                x = 0
                th = 0
                if (key == '\x03'):
                    break

            speed = x * 1.2375  # Max linear speed
            turn = th * 6.836   # Max angular speed

            twist = Twist()
            twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn

            pub.publish(twist)
            print(vels(speed, turn))

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
