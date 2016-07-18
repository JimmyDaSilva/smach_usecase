#!/usr/bin/env python

import rospy
import smach
from smach import CBState
import smach_ros

from std_msgs.msg import String


def condition_30_cb(user_data, status):
    # ... parse JSON in status message ...
    # return TRUE as long as condition is not met
    return False


@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished'])
def action_30_cb(user_data):
    # ... do something
    return 'finished'


@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished'])
def action_spike_cb(user_data):
    # ... do something
    return 'finished'

if __name__ == "__main__":
    # Create a ROS node for this state machine
    rospy.init_node("hbp_nrp_backend_sm_exp_control")

    sm_one = smach.StateMachine(
    outcomes=['FINISHED', 'ERROR', 'CONDITION_PREEMPTED',
              'ACTION_PREEMPTED', 'ACTION_ERROR'])

    with sm_one:
        smach.StateMachine.add('CONDITION_30',
                               smach_ros.MonitorState('/ros_cle_simulation/status',
                                                      String,
                                                      condition_30_cb),
                               {'valid': 'CONDITION_30', 'invalid': 'ACTION_30',
                                'preempted': 'CONDITION_PREEMPTED'})
        smach.StateMachine.add('ACTION_30', CBState(action_30_cb),
                               {'finished': 'FINISHED'})

    sis = smach_ros.IntrospectionServer('hbp_nrp_backend_sm_exp_control', sm_one, '/MONITOR_COMPLEX')
    sis.start()

    result = sm_one.execute()
    rospy.loginfo(result)