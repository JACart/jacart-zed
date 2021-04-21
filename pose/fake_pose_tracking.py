import rospy
import sys
import os
import curses
import json
from std_msgs.msg import String

class fake_pose_tracking(object):
    def __init__(self):
        rospy.init_node('fake_pose_tracking')
        # publish cart empty
        self.cart_empty_safe_pub = rospy.Publisher('/cart_empty_safe', String, queue_size=10)

        self.prev_key = 1

        curses.wrapper(self.get_input)


    def get_input(self, stdscr):
        curses.use_default_colors()
        for i in range(0, curses.COLORS):
            curses.init_pair(i, i, -1)

        q = 113
        w = 119
        e = 101
    
        passenger = False
        safe = True

        stdscr.nodelay(True)
        rate = rospy.Rate(10) 
        stdscr.addstr(0,0,'Fake Pose Tracking')
        stdscr.addstr(1,0,'CTRL-C to exit')
        stdscr.addstr(3,0,'-------------------------------')
        stdscr.addstr(4,0,'Usage:')
        stdscr.addstr(5,0,'q: passenger true, safe true ')
        stdscr.addstr(6,0,'w: passenger true, safe false')
        stdscr.addstr(7,0,'e: passenger false, safe true')
        stdscr.addstr(8,0,'-------------------------------')
        stdscr.addstr(10,0,'Last Posted')
        while not rospy.is_shutdown():
            keyval = stdscr.getch()

            if keyval == self.prev_key:
                continue
            if keyval == q:
                passenger = True
                safe = True
            elif keyval == w:
                passenger = True
                safe = False
            elif keyval == e:
                passenger = False
                safe = True

            empty_safe_state = json.dumps({'passenger': passenger, 'safe': safe})
            self.cart_empty_safe_pub.publish(empty_safe_state)
        
            self.prev_key = keyval
            stdscr.addstr(11,0, empty_safe_state)
            stdscr.addstr(13,0, '')
            rate.sleep()

if __name__ == "__main__":
    try:
        fake_pose_tracking()
    except rospy.ROSInterruptException:
        pass