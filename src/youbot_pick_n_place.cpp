/*
  Modified from the code for PR2
  by Aneesh PA on 08/09/2017

  Gets pose info from the list of poses created through moveit-assistant

*/

#include<moveit/move_group_interface/move_group.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>

#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/DisplayTrajectory.h>

#include<moveit_msgs/CollisionObject.h>
#include<moveit_msgs/AttachedCollisionObject.h>

#include<brics_actuator/JointPositions.h>

#include<string.h>

//void openGripper();
// Actuate gripper to open/hold/release : 0.1/0.05/0.0
void move_gripper(float, float);

// Gripper positions
const float gr_close = 0.0;
const float gr_open = 0.01;
const float gr_hold = 0.005;

// Pause in seconds between target pose and start of next plan
const float t_pause = 1 ;

// loop count for repeating pick_n_place
int loop_count = 3;
// IMPORTANT
// maxVelocityScalingFactor 0.2;  t_pause 1; is a working combination for this program

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_move_group_tests");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Let Rviz come up
    ROS_INFO("Let Rviz come up");
    //   sleep(20.0);

    // The move_group that we are working on the youbot
    // moveit::planning_interface::MoveGroup group("arm_1");

    // Our handle to the world
     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Publisher for visualizing plans in Rviz; Optional
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true);
    //ros::Publisher pub = node_handle.advertise<brics_actuator::JointPositions>("/arm_1/gripper_controller/position_command", 1000);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Declare a planning group
    // Planning groups are stored in an object called JointModelGroup
    static const std::string PLANNING_GROUP = "arm_1";
    moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);
      
    //std::vector<std::string> *activeLinks = move_group.getActiveJoints();
    //ROS_INFO("Active joints are: %s", activeLinks->data().c_str());
    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Prints active joints; lu for long unsigned int
    ROS_INFO("Number of Activejoints: %lu", move_group.getActiveJoints().size());

    // Set planner
    move_group.setPlannerId("RRTConnectkConfigDefault");

    // Get all store=nd target poses
    std::vector<std::string> targetPoses;
    std::vector<std::string>::iterator it;
    targetPoses = move_group.getNamedTargets();

    for(it=targetPoses.begin(); it<targetPoses.end();it++)
    {
        ROS_INFO("Stored target poses are: %s", it->c_str());  //prints all stored target poses
    }

    // Set Velocity within safe limits, better to set it under 0.7
    double maxVelocityScalingFactor = 0.2;
    move_group.setMaxVelocityScalingFactor(maxVelocityScalingFactor);

    // close gripper;
    move_gripper(gr_close, gr_close);

    // Move to namedTarget "erect_high"
    bool success;
    move_group.setNamedTarget("erect_high");
    moveit::planning_interface::MoveGroup::Plan my_plan;
    success = move_group.plan(my_plan);
    // Show default planerId
    //ROS_INFO("Default planner id is: %s", move_group.getDefaultPlannerId("arm_1").c_str());

    /* Sleep so we have time to see the object in RViz */
    sleep(2.0);

    moveit::planning_interface::MoveItErrorCode Err = move_group.move();
    ROS_INFO("After move execution Err.SUCCESS: %d", Err.SUCCESS);
    ROS_INFO("Sleeping after erect_high");
    sleep(t_pause);
    // open gripper();
    move_gripper(gr_open, gr_open);

    do{
        ROS_INFO("Activating forward_bend");
        // Move to namedTarget "forward_bend"
        move_group.setNamedTarget("forward_bend");
        success = move_group.plan(my_plan);
        Err = move_group.move();
        ROS_INFO("After forward_bend execution Err.SUCCESS: %d", Err.SUCCESS);
        sleep(t_pause);

        ROS_INFO("Activating slide_in");
        move_group.setNamedTarget("slide_in");
        success = move_group.plan(my_plan);
        Err = move_group.move();
        ROS_INFO("After slide_in execution Err.SUCCESS: %d", Err.SUCCESS);
        sleep(t_pause);

        // hold position for the gripper
        move_gripper(gr_hold, gr_hold);
        sleep(t_pause);

        ROS_INFO("Activating carry_load");
        move_group.setNamedTarget("carry_load");
        success = move_group.plan(my_plan);
        Err = move_group.move();
        ROS_INFO("After carry_load execution Err.SUCCESS: %d", Err.SUCCESS);
        sleep(t_pause);

        ROS_INFO("Activating backward_bend");
        move_group.setNamedTarget("backward_bend");
        success = move_group.plan(my_plan);
        Err = move_group.move();
        ROS_INFO("After backward_bend execution Err.SUCCESS: %d", Err.SUCCESS);
        sleep(t_pause);

        ROS_INFO("Activating stoop_low");
        move_group.setNamedTarget("stoop_low");
        success = move_group.plan(my_plan);
        Err = move_group.move();
        ROS_INFO("After stoop_low execution Err.SUCCESS: %d", Err.SUCCESS);
        sleep(t_pause);

        // release /open gripper
        move_gripper(gr_open, gr_open);
        sleep(2*t_pause);

        /*
        ROS_INFO("Activating backward_bend");
        move_group.setNamedTarget("backward_bend");
        success = move_group.plan(my_plan);
        Err = move_group.move();
        ROS_INFO("After backward_bend execution Err.SUCCESS: %d", Err.SUCCESS);
        */

        --loop_count;


//        sleep(2*t_pause);
    } while(loop_count>0);

    // close gripper
    move_gripper(gr_close, gr_close);

    // go back to home
    ROS_INFO("Activating go home");
    // Move to namedTarget "gazebo_home"
    move_group.setNamedTarget("home_gazebo");
    success = move_group.plan(my_plan);
    Err = move_group.move();
    ROS_INFO("After gazebo_home execution Err.SUCCESS: %d", Err.SUCCESS);
    sleep(t_pause);

    ros::shutdown();
    return 0;
}

/*
void openGripper()
{


    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<brics_actuator::JointPositions>("/arm_1/gripper_controller/position_command", 1000);

    ROS_INFO("Finger opening pub node created");

    ros::Rate rate(1);
    //ROS_INFO_STREAM("enterring ros::ok loop");

    int count = 0;

    while (ros::ok() && count < 2) {

       brics_actuator::JointPositions msg;
       msg.positions[0].joint_uri="gripper_finger_joint_r";
       msg.positions[0].unit="m";
       msg.positions[0].value=0.01;

       msg.positions[1].joint_uri="gripper_finger_joint_l";
       msg.positions[1].unit="m";
       msg.positions[1].value=0.005;

       pub.publish(msg);
    //   pub.publish(msg_l);
       ++count;
       //wait until its time
       rate.sleep();
       ROS_INFO_STREAM("Opened Fingers");
       }

}
*/

void move_gripper(float l_val, float r_val)
{
    //  ros::init(argc, argv, "node_pubvel1");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<brics_actuator::JointPositions>("/arm_1/gripper_controller/position_command", 1000);

    ROS_INFO("Finger opening pub node created");

    ros::Rate rate(1);
    //ROS_INFO_STREAM("enterring ros::ok loop");

    int count = 0;

    while (ros::ok() && count < 2) {
        brics_actuator::JointPositions msg;
        //ROS_INFO_STREAM("to resize joint names array");
        msg.positions.resize(2);
        msg.positions[0].joint_uri="gripper_finger_joint_r";
        msg.positions[0].unit="m";
        msg.positions[0].value=r_val;

        msg.positions[1].joint_uri="gripper_finger_joint_l";
        msg.positions[1].unit="m";
        msg.positions[1].value=l_val;

        pub.publish(msg);
        //   pub.publish(msg_l);
        ++count;
        //wait until its time
        rate.sleep();
        ROS_INFO_STREAM("Opened Fingers");
     }

 }
