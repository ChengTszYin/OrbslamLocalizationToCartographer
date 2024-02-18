#include "Localize.h"

Localize::Localize(): local_method("cartographer")
{
  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
  orb_client_ = nh.serviceClient<std_srvs::Trigger>("/orb_service");
  pose_sub_ = nh.subscribe("orb_slam2_pose", 1000,&Localize::cartoInit_callback,this);
  service = nh.advertiseService("/Localize_service",&Localize::callback_,this);
  feature_sub_ = nh.subscribe("/featureLevel",1000,&Localize::featCallback_,this);
  euler_sub_ = nh.subscribe("/euler_angles",1000, &Localize::angleCallback_,this);
  finishTrajectoryClient = nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>("/finish_trajectory");
  StartTrajectoryCliebt = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/start_trajectory");
}

void Localize::angleCallback_(const std_msgs::Int16::ConstPtr& msgs)
{ 
  int old_angle = angle;
  angle = msgs->data;
  old_angle = old_angle==0 ? angle : old_angle;
  startAngle = (angle - old_angle)==0 ? startAngle : (startAngle + (angle - old_angle));
  pose_received = true;
}

void Localize::featCallback_(const std_msgs::Int16::ConstPtr& msgs)
{
  nfeature = msgs->data;
}

void Localize::cartoInit_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  xcoordinate = msg -> pose.position.x;
  ycoordinate = msg -> pose.position.y;
  xorient = msg -> pose.orientation.x;
  yorient = msg -> pose.orientation.y;
  zorient = msg -> pose.orientation.z;
  worient = msg -> pose.orientation.w;
}

void Localize::StartLocalize()
{ 
  geometry_msgs::Twist move;
  move.angular.z = 0.4;
  ros::Rate rate(5);
  while(nfeature<2000 && startAngle >= 0)
  { 
    //std::cout << nfeature << std::endl;
    std::cout << "angle: " << startAngle << std::endl; 
    vel_pub.publish(move);
    ros::spinOnce();
    rate.sleep();
  }
  startAngle = 0;
  ROS_INFO("ESCAPER FORM WHILE LOOP");
  if(nfeature<2000)
  {
    ROS_INFO("Localization Failed, Please retry at other localization");
  }
  pose_received = false;
  move.angular.z = 0;
  vel_pub.publish(move);
  if(nfeature >= 2000)
  { 
    if(local_method=="amcl"){
      ROS_INFO("Calling /orb_service: %s", orb_client_.call(srv)? "SUCCESS" : "Failed to call service");
    }
    else{
      cartographer_ros_msgs::FinishTrajectory srv;
      srv.request.trajectory_id = poseID;
      ROS_INFO("Calling /finishTrajectoryClient: %s",finishTrajectoryClient.call(srv) ? "SUCCESS" : "Failed to call service");
      std::cout << "poseID " << poseID << std::endl;
      poseID++;
      cartographer_ros_msgs::StartTrajectory carto;
      carto.request.configuration_directory = "/home/dllm/galio_ws/src/turtlebot3/turtlebot3/turtlebot3_slam/config";
      carto.request.configuration_basename = "turtlebot3_lds_2d_gazebo_localization.lua";
      carto.request.use_initial_pose = true;
      carto.request.initial_pose.position.x = xcoordinate;
      carto.request.initial_pose.position.y = ycoordinate;
      carto.request.initial_pose.orientation.x = xorient;
      carto.request.initial_pose.orientation.y = yorient;
      carto.request.initial_pose.orientation.z = zorient;
      carto.request.initial_pose.orientation.w = worient;
      ROS_INFO("Calling /start_trajectory: %s", StartTrajectoryCliebt.call(carto)? "SUCCESS" : "Failed to call service");
    }
  }
  else
  {
    ROS_INFO("Localization Failed. Please Localiza again near the wall");
  }
  ROS_INFO("Localization finished");
  taskFinish = false;
}

bool Localize::callback_(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  taskFinish = true;
  while(!pose_received)
  { 
    ROS_INFO("Service is not available. Please double check the /odom");
    ros::spinOnce();
  }

  while(taskFinish)
  {
    StartLocalize();
    if(!taskFinish)
    {
      ROS_INFO("Breaking out of the loop");
      break;
    }
  }
  ROS_INFO("Breaking out of the bool loop");
  return true;
}

void Localize::LocalMethodSelect(const std::string method)
{
  local_method = method;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Localize_node");
  Localize localization;
  std::string method = "cartographer";
  if(argc>1){
    localization.LocalMethodSelect(argv[1]);
    method = "amcl";
  }
  ROS_INFO("Set to %s ,ready to receive requests.",method.c_str());
  ros::spin();
  return 0;
}