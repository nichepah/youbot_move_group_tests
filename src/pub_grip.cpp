#include<brics_actuator/JointPositions.h>

{
//    ros::init(argc, argv, "node_pubvel1");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<brics_actuator::JointPositions>("/arm_1/gripper_controller/position_command", 1000);

    ROS_INFO("Finger opening pub node created");

   ros::Rate rate(1);
   //ROS_INFO_STREAM("enterring ros::ok loop");

   // Publish once and die
   int count = 1;

   while (ros::ok() && count < 2) {

      brics_actuator::JointPositions msg_r, msg_l;
      //ROS_INFO_STREAM("to resize joint names array");

      msg_r.positions.resize(1);
      msg_r.positions[0].joint_uri="gripper_finger_joint_r";
      msg_r.positions[0].unit="m";
      msg_r.positions[0].value=0.01;

      msg_l.positions.resize(1);
      msg_l.positions[0].joint_uri="gripper_finger_joint_l";
      msg_l.positions[0].unit="m";
      msg_l.positions[0].value=0.01;

      pub.publish(msg_r);
      pub.publish(msg_l);
      ++count;
      //wait until its time
      rate.sleep();
      ROS_INFO_STREAM("Opened Fingers");
      }

}
