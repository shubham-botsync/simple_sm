#!/usr/bin/env python

import rospy
import smach
import smach_ros


class NavFwd(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['pass', 'fail'])

    def execute(self, userdata):

        rospy.loginfo('Executing FwdNavigation!\n')
        rospy.sleep(1.0)
        return 'pass'


class NavRev(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['pass', 'fail'])

    def execute(self, userdata):

        rospy.loginfo('Executing RevNavigation!\n')
        rospy.sleep(1.0)
        return 'pass'


def main():

    rospy.init_node('simple_sm_node')

    sm = smach.StateMachine(outcomes=['SM_PASS', 'SM_FAIL'])

    with sm:

        smach.StateMachine.add('NAV_FWD', NavFwd(),
                               transitions={'pass': 'NAV_REV',
                                            'fail': 'NAV_FWD'})

        smach.StateMachine.add('NAV_REV', NavRev(),
                               transitions={'pass': 'NAV_FWD',
                                            'fail': 'NAV_REV'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':

    main()
