#!/usr/bin/env python
import roslib; roslib.load_manifest('smach_usecase')
import rospy
import smach
import std_srvs.srv
import smach_ros
from smach_ros import ServiceState, SimpleActionState
from lwr_peg_in_hole.srv import UpdateSceneService, UpdateSceneServiceRequest
from lwr_peg_in_hole.msg import RobotMoveGoal, RobotMoveAction
from ar_track_alvar_msgs.msg import AlvarMarkers

class PoseIteratorCB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['pose_index', 'look_poses'], output_keys=['pose_index'])
            
    def execute(self, userdata):
        if userdata.pose_index < (len(userdata.look_poses)-1):
            userdata.pose_index = userdata.pose_index + 1
            return 'succeeded'
        else:
            userdata.pose_index = 0
            return 'succeeded'
            
class MarkerCB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_tag', 'failed', 'error'], input_keys=['tag_id'], output_keys=['tag_pose'])
        self.outcome = 'error'
        self.target_tag_id = None
        self.tag_pose = None
        
    def execute(self, userdata):
        self.outcome = 'error'
        self.target_tag_id = userdata.tag_id
        rospy.sleep(0.5)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.cb_once)
        rospy.sleep(0.5)
        userdata.tag_pose = self.tag_pose
        return self.outcome
        
    def cb_once(self, msg):
        if self.target_tag_id is None:
            return
        self.outcome = 'failed'
        markers = msg.markers
        if len(markers) > 0:
            for marker in markers:
                if marker.id == self.target_tag_id :
                    self.outcome = 'found_tag'
                    self.tag_pose = marker.pose
                    return
        return
                  
@smach.cb_interface(input_keys=['pose_index', 'look_poses'], outcomes=['succeeded'])
def robot_goal_cb(userdata, goal):
    goal = RobotMoveGoal(userdata.look_poses[userdata.pose_index],False)
    return goal

def main():
    rospy.init_node('smach_usecase_executive')

    sm = smach.StateMachine(outcomes=[])
    sm.userdata.tag_id = 0
    sm.userdata.object_name = 'plaque2'
    sm.userdata.look_poses = [[1.64647737821113, -0.06401549522493077, -0.37189732272924836, -1.7298683559749657, -0.010311991928772635, 1.2837615843359318, -1.5951452987645744], 
                              [1.7450482624189192, -0.2792615169621282, -0.7326299417794155, -1.6953566682827867, -0.1489061268374794, 1.5780836265219742, -1.8916098999458724], 
                              [1.9830371040607986, -0.0006218652097080835, 0.009821608276626925, -1.7048954646128616, -0.0576423149512717, 1.3746182539883787, -0.910279200952389]]   
    
    with sm:
        req_update = UpdateSceneServiceRequest()
        req_update.object_name = sm.userdata.object_name
        req_update.tag_id = sm.userdata.tag_id
                                 
        tag_sm = smach.StateMachine(outcomes=['succeeded'], input_keys = ['object_name', 'look_poses', 'tag_id' ])
        tag_sm.userdata.pose_index = 0
        tag_sm.userdata.nb_poses = len(sm.userdata.look_poses)
        with tag_sm:
            smach.StateMachine.add('POSE_I', SimpleActionState('robot_mover', RobotMoveAction, goal_cb=robot_goal_cb),{'succeeded':'TAG?','aborted':'POSE_I','preempted':'POSE_I'})
            smach.StateMachine.add('TAG?', MarkerCB(), {'found_tag':'succeeded', 'failed':'I++', 'error':'TAG?'})
            smach.StateMachine.add('I++', PoseIteratorCB(), {'succeeded':'POSE_I'})
            
        smach.StateMachine.add('FIND_TAG', tag_sm, {'succeeded':'UPDATE_SCENE'})
        smach.StateMachine.add('UPDATE_SCENE', ServiceState('update_scene', UpdateSceneService, req_update), {'succeeded':'COMPUTE_HOLES', 'aborted':'UPDATE_SCENE', 'preempted':'UPDATE_SCENE'})
        smach.StateMachine.add('COMPUTE_HOLES', ServiceState('reset', std_srvs.srv.Empty), {'succeeded':'PLACE_FASTENER','aborted':'COMPUTE_HOLES', 'preempted':'COMPUTE_HOLES'})
            
                        
        place_sm = smach.StateMachine(outcomes=['succeeded'])
        with place_sm:
            smach.StateMachine.add('TAKE_EPINGLE', ServiceState('reset', std_srvs.srv.Empty), {'succeeded':'GO_TO_START','aborted':'TAKE_EPINGLE', 'preempted':'TAKE_EPINGLE'})                               
            smach.StateMachine.add('GO_TO_START', ServiceState('reset', std_srvs.srv.Empty),{'succeeded':'PUT_EPINGLE','aborted':'GO_TO_START', 'preempted':'GO_TO_START'})                               
            smach.StateMachine.add('PUT_EPINGLE', ServiceState('reset', std_srvs.srv.Empty), {'succeeded':'succeeded','aborted':'PUT_EPINGLE', 'preempted':'PUT_EPINGLE'})                               
        smach.StateMachine.add('PLACE_FASTENER', place_sm, {'succeeded':'REMOVE_FASTENER'})

        remove_sm = smach.StateMachine(outcomes=['succeeded'])
        with remove_sm:
            smach.StateMachine.add('TAKE_EPINGLE', ServiceState('reset', std_srvs.srv.Empty), {'succeeded':'GO_TO_START','aborted':'TAKE_EPINGLE', 'preempted':'TAKE_EPINGLE'})
            smach.StateMachine.add('GO_TO_START', ServiceState('reset', std_srvs.srv.Empty), {'succeeded':'PUT_EPINGLE','aborted':'GO_TO_START', 'preempted':'GO_TO_START'})                               
            smach.StateMachine.add('PUT_EPINGLE', ServiceState('reset', std_srvs.srv.Empty), {'succeeded':'succeeded','aborted':'PUT_EPINGLE', 'preempted':'PUT_EPINGLE'})
        smach.StateMachine.add('REMOVE_FASTENER', remove_sm, {'succeeded':'PLACE_FASTENER'})
                  

    sis = smach_ros.IntrospectionServer('demo_introspection_server', sm, '/DEMO')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
    