#!/usr/bin/env python
import roslib; roslib.load_manifest('smach_usecase')
import rospy
import smach
import smach_ros
from ar_track_alvar_msgs.msg import AlvarMarkers

class bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['bar_succeeded'])
    def execute(self, userdata):
        rospy.sleep(0.1)
        return 'bar_succeeded'
        
class CBTest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_tag', 'failed', 'error'])
        self.outcome = 'error'
        
    def execute(self, userdata):
        self.outcome = 'error'
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.cb_once)
        rospy.sleep(0.1)
        return self.outcome
        
    def cb_once(self, msg):
        self.outcome = 'failed'
        markers = msg.markers
        if len(markers) > 0:
            self.outcome = 'found_tag'
        return

def main():
    rospy.init_node("monitor_example")

    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('TAG?', CBTest(), transitions={'found_tag':'UPDATE', 'failed':'MOVE', 'error':'ERROR'})
        smach.StateMachine.add('ERROR',bar(), transitions={'bar_succeeded':'TAG?'})
        smach.StateMachine.add('MOVE',bar(), transitions={'bar_succeeded':'TAG?'})
        smach.StateMachine.add('UPDATE',bar(), transitions={'bar_succeeded':'TAG?'})

    sis = smach_ros.IntrospectionServer('cb_test_intro_server', sm, '/CALLBACK_SM')
    sis.start()
    
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()