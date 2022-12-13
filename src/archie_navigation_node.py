#!/usr/bin/python3

import rospy

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from geometry_msgs.msg import PoseStamped, Twist
from actionlib import SimpleActionClient

from archie_manipulation.srv import Pick, PickRequest, Place, PlaceRequest

pickPose = PoseStamped()
pickPose.header.frame_id = 'map'
pickPose.pose.position.x = 2.557
pickPose.pose.position.y = -3.177
pickPose.pose.position.z = 0.055
pickPose.pose.orientation.z = -0.224
pickPose.pose.orientation.w =  0.975

placePose = PoseStamped()
placePose.header.frame_id = 'map'
placePose.pose.position.x = 1.855
placePose.pose.position.y = -2.371
placePose.pose.position.z = 0.055
placePose.pose.orientation.z = 0.964
placePose.pose.orientation.w =  0.265

midPose = PoseStamped()
midPose.header.frame_id = 'map'
midPose.pose.position.x = 2.050
midPose.pose.position.y = -2.830
midPose.pose.position.z = 0.055
midPose.pose.orientation.z = -0.368
midPose.pose.orientation.w =  0.930

moveBaseAction = None
cmdVelPublisher = None

def backAway() -> None :
    msg = Twist()
    msg.linear.x = 0.2
    cmdVelPublisher.publish(msg)
    rospy.sleep(0.8)
    msg.linear.x = 0.0
    cmdVelPublisher.publish(msg)
    rospy.sleep(0.2)

def goToPose(pose: PoseStamped) -> None:
    goal = MoveBaseGoal()
    goal.target_pose = pose
    moveBaseAction.send_goal(goal)
    moveBaseAction.wait_for_result()

if __name__ == "__main__":
    rospy.init_node('archie_navigation', anonymous=False)
    moveBaseAction = SimpleActionClient('move_base', MoveBaseAction)
    moveBaseAction.wait_for_server()
    cmdVelPublisher =  rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamp',
                                                   Twist,
                                                   queue_size=20)

    pickService = rospy.ServiceProxy("pick_service", Pick)
    placeService = rospy.ServiceProxy("place_service", Place)
    pickService.wait_for_service()
    placeService.wait_for_service()

    print("started execution")

    # goToPose(midPose)
    # rospy.sleep(0.5)

    # goToPose(pickPose)
    # rospy.sleep(3)

    pickRequest = PickRequest()
    pickRequest.objectName.data = 'cup1'
    pickService.call(pickRequest)

    backAway()
    rospy.sleep(1)

    goToPose(placePose)
    rospy.sleep(3)

    placeRequest = PlaceRequest()
    placeRequest.objectName.data = 'cup1'
    placeService.call(placeRequest)

    backAway()
    rospy.sleep(1)

