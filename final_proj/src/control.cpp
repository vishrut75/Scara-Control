#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/srv/pose_to_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
using std::placeholders::_1;

//Node Class definition
class Control_Node : public rclcpp::Node
{
public:
  Control_Node()
  : Node("Control_Node")
  { // defining both the subscribers
    // Subscriber 1 to read data from "topic" and calculate forward kinematics
    
    //std::ofstream OutFile;
    //OutFile.open ("Test1.txt", std::ofstream::out | std::ofstream::app);
    //OutFile << "time;Joint1-desired;Joint1-actual;Joint2-desired;Joint2-actual;Joint3-desired;Joint3-actual;" << std::endl;
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states",10, std::bind(&Control_Node::topic_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("forward_effort_controller/commands", 10);
    service = this->create_service<custom_msgs::srv::PoseToArray>("input_state", std::bind(&Control_Node::input_state, this, _1, std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialization Completed.");
    //OutFile.close();
  }

  void input_state(const std::shared_ptr<custom_msgs::srv::PoseToArray::Request> request, 
          std::shared_ptr<custom_msgs::srv::PoseToArray::Response>       response)
  {
    q1_d = request->px;
    q2_d = request->py;
    q3_d = request->pz;
    response->sol1 = {0,0,0};
    response->sol2 = {0,0,0};
  }
  // Forward Kinematics callback function -> prints homogeneous matrix for input configuration
  void topic_callback(const sensor_msgs::msg::JointState & message) const
  {
    //reading the input configuration
    float q1_c,q2_c,q3_c;
    float P[3],D[3];
    q1_c = message.position[0];
    q2_c = message.position[1];
    q3_c = message.position[2];
    D[0] = -1*message.velocity[0];
    D[1] = -1*message.velocity[1];
    D[2] = -1*message.velocity[2];
    int t_c = message.header.stamp.sec*1000 + (message.header.stamp.nanosec/1000000) ;
    std::ofstream OutFile;
    OutFile.open ("Test1.txt", std::ofstream::out | std::ofstream::app);
    OutFile <<t_c<<";"<<q1_d<<";"<<q1_c<<";"<<q2_d<<";"<<q2_c<<";"<<q3_d<<";"<<q3_c<<std::endl;
    P[0] = q1_d-q1_c;
    P[1] = q2_d-q2_c;
    P[2] = q3_d-q3_c;

    float Kp[3]={8,5,15};
    float Kd[3]={13,7,8};
    double effort[3];
    effort[0] = Kp[0]*P[0] + Kd[0]*D[0];
    effort[1] = Kp[1]*P[1] + Kd[1]*D[1];
    effort[2] = Kp[2]*P[2] + Kd[2]*D[2] - 9.81;
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n Joint Efforts applied are :\n [q1,q2,q3] = {['%f','%f','%f']}.\n",effort[0],effort[1],effort[2]);
    
    auto end_effector = std_msgs::msg::Float64MultiArray();
    end_effector.data = {effort[0],effort[1],effort[2]};
    publisher_->publish(end_effector);
    OutFile.close();
  }

  // initializing the subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Service<custom_msgs::srv::PoseToArray>::SharedPtr service;
  //Desired joints
  float q1_d,q2_d,q3_d;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc,argv);
  // initializing the Node
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to solve Inverse Kinematics.");
  rclcpp::spin(std::make_shared<Control_Node>());
  rclcpp::shutdown();
  return 0;

}
