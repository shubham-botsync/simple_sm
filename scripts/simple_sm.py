#!/usr/bin/env python

import rospy
import smach
import smach_ros
from actionlib import *
from actionlib_msgs import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class Apple(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=['pass'])

        rospy.loginfo('Initializing Apple!\n')
        self.time_counter = 0

    def execute(self, userdata):

        rospy.loginfo('Executing Apple!\n')
        return 'pass'


class Banana(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['pass'])

        rospy.loginfo('Initializing Banana!\n')

    def execute(self, userdata):

        rospy.loginfo('Executing Banana!\n')
        return 'pass'


def main():

    rospy.init_node('simple_sm_node')

    sm = smach.StateMachine(outcomes=['sm_pass', 'sm_fail'])

    with sm:

        smach.StateMachine.add('APPLE', Apple(),
                               transitions={'pass': 'BANANA'}
                               )

        smach.StateMachine.add('BANANA', Banana(),
                               transitions={'pass': 'APPLE'}
                               )

    # Introspection Server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')

    sis.start()

    outcome = sm.execute()

    rospy.spin()

    # sis.stop()
    rospy.signal_shutdown('All done.')


if __name__ == '__main__':

    main()
