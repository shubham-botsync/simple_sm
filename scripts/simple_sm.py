#!/usr/bin/env python

import rospy
import smach
import smach_ros
from actionlib import *
from actionlib_msgs import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class Idle(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=['pass'])

        rospy.loginfo('Initializing IDLE State!\n')
        self.time_counter = 0

    def execute(self, userdata):

        rospy.loginfo('IDLE!\n')

        while self.time_counter < 4:
            self.time_counter += 1
            rospy.sleep(1.0)

            if self.time_counter == 5:
                return 'pass'


class Navigation(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['pass'])

    def execute(self, userdata):

        #rospy.loginfo('Executing FwdNavigation!\n')
        # rospy.sleep(1.0)
        return 'pass'


def main():

    rospy.init_node('simple_sm_node')

    sm = smach.StateMachine(outcomes=['sm_pass', 'sm_fail'])

    '''
    with sm:

        goal_ = MoveBaseGoal()

        goal_.target_pose.header.frame_id = "bi_tf/map"
        goal_.target_pose.pose.position.x = 3.19812631607
        goal_.target_pose.pose.position.y = -4.74170589447
        goal_.target_pose.pose.orientation.x = 0
        goal_.target_pose.pose.orientation.y = 0
        goal_.target_pose.pose.orientation.z = -0.17400141277
        goal_.target_pose.pose.orientation.w = 0.984745402809

        smach.StateMachine.add('STATE_ONE',
                               smach_ros.SimpleActionState('bi',
                                                           MoveBaseAction, goal=goal_),
                               transitions={'succeeded': 'STATE_ONE'})

        smach.StateMachine.add('GOAL_DEFAULT',
                               smach_ros.SimpleActionState(
                                   'bi/move_base', MoveBaseAction),
                               {'succeeded': 'GOAL_DEFAULT', 'aborted': 'GOAL_DEFAULT'})

        smach.StateMachine.add('GOAL_STATIC',
                               smach_ros.SimpleActionState('bi/move_base', MoveBaseAction,
                                                           goal=goal_),
                               {'aborted': 'GOAL_STATIC'})

        '''

    with sm:

        smach.StateMachine.add('IDLE', Idle(),
                               transitions={'pass': 'NAVIGATION'}
                               )

        smach.StateMachine.add('NAVIGATION', Navigation(),
                               transitions={'pass': 'IDLE'}
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
