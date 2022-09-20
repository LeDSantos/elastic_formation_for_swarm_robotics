import rospy

import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_demo():    
    
    robot1 = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)
    robot2 = actionlib.SimpleActionClient('robot2/move_base', MoveBaseAction)
    robot3 = actionlib.SimpleActionClient('robot3/move_base', MoveBaseAction)
    
    robot1.wait_for_server();
    robot2.wait_for_server();
    robot3.wait_for_server();
    
    goal1 = MoveBaseGoal()
    goal1.target_pose.header.frame_id = "map"
    goal1.target_pose.header.stamp = rospy.Time.now()
    goal1.target_pose.pose.position.x = 7.0
    goal1.target_pose.pose.position.y = 1.0
    goal1.target_pose.pose.orientation.w = 1.0
    goal2 = MoveBaseGoal()
    goal2.target_pose.header.frame_id = "map"
    goal2.target_pose.header.stamp = rospy.Time.now()
    goal2.target_pose.pose.position.x = 1.0
    goal2.target_pose.pose.position.y = 7.0
    goal2.target_pose.pose.orientation.w = 1.0
    goal3 = MoveBaseGoal()
    goal3.target_pose.header.frame_id = "map"
    goal3.target_pose.header.stamp = rospy.Time.now()
    goal3.target_pose.pose.position.x = -7.0
    goal3.target_pose.pose.position.y = -1.0
    goal3.target_pose.pose.orientation.w = 1.0
    
    
    robot1.send_goal(goal1)
    robot2.send_goal(goal2)
    robot3.send_goal(goal3)
    
    wait1 = robot1.wait_for_result()
    wait2 = robot2.wait_for_result()
    wait3 = robot3.wait_for_result()
    
    return robot1.get_result(), robot2.get_result(), robot3.get_result()
    
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_demo')
        result = move_demo()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
    
    