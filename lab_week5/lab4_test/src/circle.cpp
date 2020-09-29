// #include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
// Main moveit libraries are included
int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(0);
    spinner.start(); 
    /* For moveit implementation we need AsyncSpinner, we
    cant use ros::spinOnce() */

    
    static const std::string PLANNING_GROUP = "group1_controller";     
    /* Now we specify with what group we want work,
       here group1 is the name of my group controller*/
    
    moveit::planning_interface::MoveGroupInterface
        move_group(PLANNING_GROUP); // loading move_group
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState() -> getJointModelGroup(PLANNING_GROUP); //For joint control
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped starting_pose;
    geometry_msgs::PoseStamped target_pose_1;
    geometry_msgs::PoseStamped target_pose_2;
    geometry_msgs::PoseStamped target_pose_3;
    geometry_msgs::PoseStamped target_pose_4;
    geometry_msgs::PoseStamped target_pose_5;
    geometry_msgs::PoseStamped target_pose_6;
    geometry_msgs::PoseStamped target_pose_7;
    geometry_msgs::PoseStamped target_pose_8;
    geometry_msgs::PoseStamped target_pose_9;
    geometry_msgs::PoseStamped target_pose_10;
    geometry_msgs::PoseStamped target_pose_11;
    geometry_msgs::PoseStamped target_pose_12;
    /* Pose in ROS is implemented using geometry_msgs::PoseStamped, 
       google what is the type of this msg */
        
        current_pose = move_group.getCurrentPose(); 
    /* Retrieving the information about the
       current position and orientation of the end effector*/
    
    starting_pose = current_pose;
    starting_pose.pose.position.x = starting_pose.pose.position.x - 0.1;
    
    target_pose_1 = starting_pose;
    target_pose_1.pose.position.x = target_pose_1.pose.position.x - 0.1;
    target_pose_1.pose.position.y = target_pose_1.pose.position.y + 0.2;
    
    target_pose_2 = target_pose_1;
    target_pose_2.pose.position.x = target_pose_2.pose.position.x - 0.1;
    target_pose_2.pose.position.y = target_pose_2.pose.position.y + 0.1;

    target_pose_3 = target_pose_2;
    target_pose_3.pose.position.x = target_pose_3.pose.position.x - 0.2;
    target_pose_3.pose.position.y = target_pose_3.pose.position.y + 0.1;

    target_pose_4 = target_pose_3;
    target_pose_4.pose.position.x = target_pose_4.pose.position.x - 0.2;
    target_pose_4.pose.position.y = target_pose_4.pose.position.y - 0.1;

    target_pose_5 = target_pose_4;
    target_pose_5.pose.position.x = target_pose_5.pose.position.x - 0.1;
    target_pose_5.pose.position.y = target_pose_5.pose.position.y - 0.1;

    target_pose_6 = target_pose_5;
    target_pose_6.pose.position.x = target_pose_6.pose.position.x - 0.1;
    target_pose_6.pose.position.y = target_pose_6.pose.position.y - 0.2;

    target_pose_7 = target_pose_6;
    target_pose_7.pose.position.x = target_pose_7.pose.position.x + 0.1;
    target_pose_7.pose.position.y = target_pose_7.pose.position.y - 0.2;
    
    target_pose_8 = target_pose_7;
    target_pose_8.pose.position.x = target_pose_8.pose.position.x + 0.1;
    target_pose_8.pose.position.y = target_pose_8.pose.position.y - 0.1;

    target_pose_9 = target_pose_8;
    target_pose_9.pose.position.x = target_pose_9.pose.position.x + 0.2;
    target_pose_9.pose.position.y = target_pose_9.pose.position.y - 0.1;

    target_pose_10 = target_pose_9;
    target_pose_10.pose.position.x = target_pose_10.pose.position.x + 0.2;
    target_pose_10.pose.position.y = target_pose_10.pose.position.y + 0.1;

    target_pose_11 = target_pose_10;
    target_pose_11.pose.position.x = target_pose_11.pose.position.x + 0.1;
    target_pose_11.pose.position.y = target_pose_11.pose.position.y + 0.1;

    target_pose_12 = target_pose_11;
    target_pose_12.pose.position.x = target_pose_12.pose.position.x + 0.1;
    target_pose_12.pose.position.y = target_pose_12.pose.position.y + 0.2;
    /* Basically our target pose is the same as current,
       except that we want to move it a little bit along x-axis*/
    ros::Rate loop_rate(50); //Frequency


    while (ros::ok())
    {
        move_group.setApproximateJointValueTarget(starting_pose); // To calculate the trajectory
        move_group.move(); // Move the robot
	ROS_INFO("Starting Pose - Done");

	move_group.setApproximateJointValueTarget(target_pose_1); // To calculate the trajectory
        move_group.move(); // Move the robot
	ROS_INFO("Pose 1 - Done");

	move_group.setApproximateJointValueTarget(target_pose_2); // To calculate the trajectory
        move_group.move(); // Move the robot
	ROS_INFO("Pose 2 - Done");

	move_group.setApproximateJointValueTarget(target_pose_3); // To calculate the trajectory
        move_group.move(); // Move the robot
	ROS_INFO("Pose 3 - Done");

	move_group.setApproximateJointValueTarget(target_pose_4); // To calculate the trajectory
        move_group.move(); // Move the robot
	ROS_INFO("Pose 4 - Done");

	move_group.setApproximateJointValueTarget(target_pose_5); // To calculate the trajectory
        move_group.move(); // Move the robot	
	ROS_INFO("Pose 5 - Done");
	
	move_group.setApproximateJointValueTarget(target_pose_6); // To calculate the trajectory
        move_group.move(); // Move the robot	
	ROS_INFO("Pose 6 - Done");

	move_group.setApproximateJointValueTarget(target_pose_7); // To calculate the trajectory
        move_group.move(); // Move the robot
	ROS_INFO("Pose 7 - Done");

	move_group.setApproximateJointValueTarget(target_pose_8); // To calculate the trajectory
        move_group.move(); // Move the robot
	ROS_INFO("Pose 8 - Done");

	move_group.setApproximateJointValueTarget(target_pose_9); // To calculate the trajectory
        move_group.move(); // Move the robot
	ROS_INFO("Pose 9 - Done");

	move_group.setApproximateJointValueTarget(target_pose_10); // To calculate the trajectory
        move_group.move(); // Move the robot
	ROS_INFO("Pose 10 - Done");

	move_group.setApproximateJointValueTarget(target_pose_11); // To calculate the trajectory
        move_group.move(); // Move the robot	
	ROS_INFO("Pose 11 - Done");

	move_group.setApproximateJointValueTarget(target_pose_12); // To calculate the trajectory
        move_group.move(); // Move the robot		
	ROS_INFO("Pose 12 - Done");

        current_pose = move_group.getCurrentPose();
        if ((abs(current_pose.pose.position.x - target_pose_12.pose.position.x) < 0.01) &&
	    (abs(current_pose.pose.position.y - target_pose_12.pose.position.y) < 0.01))
        {
            break; // Basically, check if we reached the desired position
        }
        loop_rate.sleep();
    }
    ROS_INFO("Circle - Done");

    
    ros::shutdown();
    return 0;
}
