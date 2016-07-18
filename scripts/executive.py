#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_usecase')

import rospy
import smach
import turtlesim.srv
import std_srvs.srv
import smach_ros
from smach_ros import ServiceState, SimpleActionState
from turtle_actionlib.msg import ShapeAction, ShapeGoal
from smach import Concurrence

def main():
    rospy.init_node('smach_usecase_executive')

    sm_root = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    with sm_root:
        smach.StateMachine.add('RESET', 
                               ServiceState('reset', std_srvs.srv.Empty), 
                               {'succeeded':'SPAWN'})
        
        request = turtlesim.srv.SpawnRequest(0.0,0.0,0.0,'turtle2')
        smach.StateMachine.add('SPAWN', 
                               ServiceState('spawn', turtlesim.srv.Spawn, request), 
                               {'succeeded':'TELEPORT1'})
        
        teleport1 = turtlesim.srv.TeleportAbsoluteRequest(5.0,1.0,0.0)
        smach.StateMachine.add('TELEPORT1', 
                               ServiceState('turtle1/teleport_absolute', turtlesim.srv.TeleportAbsolute, teleport1), 
                                {'succeeded':'TELEPORT2'})
        
        teleport2 = turtlesim.srv.TeleportAbsoluteRequest(9.0,5.0,0.0)        
        smach.StateMachine.add('TELEPORT2', 
                               ServiceState('turtle2/teleport_absolute', turtlesim.srv.TeleportAbsolute, teleport2),
                                {'succeeded':'SHAPES'})
                               
        shapes_concurrence = Concurrence(
                            outcomes=['succeeded','aborted','preempted'],
                            default_outcome='aborted',
                            outcome_map = {'succeeded':{'BIG':'succeeded','SMALL':'succeeded'}})
        smach.StateMachine.add('SHAPES', shapes_concurrence)
        with shapes_concurrence:             
            Concurrence.add('BIG',
                               SimpleActionState('turtle_shape1', ShapeAction, ShapeGoal(11,4.0)),
                                {'succeeded':'SMALL'})
            Concurrence.add('SMALL',
                               SimpleActionState('turtle_shape2', ShapeAction, ShapeGoal(6,0.5)))
                               
                               
                               
    
    sis = smach_ros.IntrospectionServer('intro_server', sm_root, '/Intro')
    sis.start()

    outcome = sm_root.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
