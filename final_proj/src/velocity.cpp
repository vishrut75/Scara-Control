#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/srv/pose_to_array.hpp"
#include "custom_msgs/srv/joint_to_end.hpp"
#include "custom_msgs/srv/end_to_joint.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
using std::placeholders::_1;

//Node Class definition
class Velocity_Node : public rclcpp::Node
{
public:
  Velocity_Node()
  : Node("Velocity_Node")
  { 
    v1_d = 0;
    v2_d = 0;
    v3_d = 0;
    J_1[0] = 5;
    J_1[1] = 5;
    J_1[2] = 0;
    J_2[0] = 5;
    J_2[1] = -5;
    J_2[2] = 0;
    J_3[0] = 0;
    J_3[1] = 0;
    J_3[2] = 1;
    start = 0;
    P[0] = 0;
    P[1] = 0;
    P[2] = 0;
    D[0] = 0;
    D[1] = 0;
    D[2] = 0;
    tp = 0;
    idx = 0;
    for(int i=0;i<10;i++){
      err[i][0]=0;
      err[i][1]=0;
      err[i][2]=0;
    }
    // defining both the subscribers
    // Subscriber 1 to read data from "joint_states" and calculate forward joint velocities
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states",10, std::bind(&Velocity_Node::topic_callback, this, _1));
    // publisher for joint efforts
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("forward_effort_controller/commands", 10);
    // forward velocity service
    serviceF = this->create_service<custom_msgs::srv::JointToEnd>("forward_velocity", std::bind(&Velocity_Node::forward_velocity, this, _1, std::placeholders::_2));
    //inverse velocity service
    serviceI = this->create_service<custom_msgs::srv::EndToJoint>("inverse_velocity", std::bind(&Velocity_Node::inverse_velocity, this, _1, std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialization Completed.");
}

  // takes in joint velocities and comverts to EE velocities
  void forward_velocity(const std::shared_ptr<custom_msgs::srv::JointToEnd::Request> request, 
          std::shared_ptr<custom_msgs::srv::JointToEnd::Response>       response)
  {
    //read requested joint velocities
    float varr[3] = {request->joint_vel[0],request->joint_vel[1],request->joint_vel[2]};
    float v1 = varr[0];
    float v2 = varr[1];
    float v3 = varr[2];

    // link lengths
    float ax = 0.05;
    // float l0 = 2.0;
    float l1 = 1.0-2*ax;
    float l2 = 1.0;

    // rows of the jacobian
    double J1[3] = {-1*l2*sin(q1_c+q2_c) - l1*sin(q1_c), -1*l2*sin(q1_c+q2_c), 0};
    double J2[3] = {l2*cos(q1_c+q2_c) + l1*cos(q1_c),  l2*cos(q1_c+q2_c), 0};
    // double J3[3] = {0,0,1};
    // float J4[3] = {0,0,0};
    // float J5[3] = {0,0,0};
    // float J6[3] = {1,1,0};

    //EE velocity calculations
    float end_vel[6] = {0,0,0,0,0,0};
    end_vel[0] = J1[0]*v1 + J1[1]*v2;
    end_vel[1] = J2[0]*v1 + J2[1]*v2;
    end_vel[2] = v3;
    end_vel[3] = 0;
    end_vel[4] = 0;
    end_vel[5] = v1 + v2;

    // send values as response
    response->end_vel = {end_vel[0],end_vel[1],end_vel[2],end_vel[3],end_vel[4],end_vel[5]};
  }

  //takes in EE velocities and converts to joint velocities
  void inverse_velocity(const std::shared_ptr<custom_msgs::srv::EndToJoint::Request> request, 
          std::shared_ptr<custom_msgs::srv::EndToJoint::Response>       response)
  {
    // read request velocities
    P[0] = 0;
    P[1] = 0;
    P[2] = 0;
    D[0] = 0;
    D[1] = 0;
    D[2] = 0;
    idx = 0;
    for(int i=0;i<10;i++){
      err[i][0]=0;
      err[i][1]=0;
      err[i][2]=0;
    }
    tp = 0;
    float varr[3] = {request->end_vel[0],request->end_vel[1],request->end_vel[2]};
    v1_d = varr[0];
    v2_d = varr[1];
    v3_d = varr[2];

    // link lengths
    float ax = 0.05;
    // float l0 = 2.0;
    float l1 = 1.0-2*ax;
    float l2 = 1.0;

    // calculate joint velocities
    float den = sin(q2_c);
    if(abs(den)>0.01){
      J_1[0] = cos(q1_c + q2_c)/(l1*den);
      J_1[1] = sin(q1_c + q2_c)/(l1*den);
      J_2[0] = -(l2*cos(q1_c + q2_c) + l1*cos(q1_c))/(l1*l2*den);
      J_2[1] = -(l2*sin(q1_c + q2_c) + l1*sin(q1_c))/(l1*l2*den);
      start = 1;
    }

    float vj1 = J_1[0]*v1_d + J_1[1]*v2_d;
    float vj2 = J_2[0]*v1_d + J_2[1]*v2_d;
    float vj3 = J_3[2]*v3_d;

    response->joint_vel = {vj1,vj2,vj3};
  }

  // Velocity controller
  void topic_callback(const sensor_msgs::msg::JointState & message)
  {
    //reading the input configuration
    //float P[3];//,D[3];
    float v1_c,v2_c,v3_c;
    q1_c = message.position[0];
    q2_c = message.position[1];
    q3_c = message.position[2];
    v1_c = message.velocity[0];
    v2_c = message.velocity[1];
    v3_c = message.velocity[2];

    // link legnths
    float ax = 0.05;
    //float l0 = 2.0;
    float l1 = 1.0-2*ax;
    float l2 = 1.0;

    double J1[3] = {-1*l2*sin(q1_c+q2_c) - l1*sin(q1_c), -1*l2*sin(q1_c+q2_c), 0};
    double J2[3] = {l2*cos(q1_c+q2_c) + l1*cos(q1_c),  l2*cos(q1_c+q2_c), 0};

    float end_vel[3] = {0,0,0};
    end_vel[0] = J1[0]*v1_c + J1[1]*v2_c;
    end_vel[1] = J2[0]*v1_c + J2[1]*v2_c;
    end_vel[2] = v3_c;
    
    float den = sin(q2_c);
    if(abs(den)>0.01){
      J_1[0] = cos(q1_c + q2_c)/(l1*den);
      J_1[1] = sin(q1_c + q2_c)/(l1*den);
      J_2[0] = -(l2*cos(q1_c + q2_c) + l1*cos(q1_c))/(l1*l2*den);
      J_2[1] = -(l2*sin(q1_c + q2_c) + l1*sin(q1_c))/(l1*l2*den);
      start = 1;
    }
    else{
      if(start){
        v1_d = -1*v1_d;
        v2_d = -1*v2_d;
        start = 0;
      }
    }

    float vj1 = J_1[0]*v1_d + J_1[1]*v2_d;
    float vj2 = J_2[0]*v1_d + J_2[1]*v2_d;
    float vj3 = J_3[2]*v3_d;

    int t_c = message.header.stamp.sec*1000 + (message.header.stamp.nanosec/1000000);

    float e0 = vj1-v1_c;
    float e1 = vj2-v2_c;
    float e2 = vj3-v3_c;

    D[0] = e0;
    D[1] = e1;
    D[2] = e2;

    if(tp>0){
	    P[0]-= err[idx][0];
	    P[1]-= err[idx][1];
	    P[2]-= err[idx][2];

	    err[idx][0] = e0*(t_c-tp)/1000;
	    err[idx][1] = e1*(t_c-tp)/1000;
	    err[idx][2] = e2*(t_c-tp)/1000;
	    idx = (idx+1)%10;
	    P[0]+= err[idx][0];
	    P[1]+= err[idx][1];
	    P[2]+= err[idx][2];
    }
    tp = t_c;

    // P controller
    // float Kp[3]={12,12,12};
    // float Kd[3]={7,8,7};
    // float effort[3];
    // effort[0] = Kp[0]*P[0] + Kd[0]*D[0];
    // effort[1] = Kp[1]*P[1] + Kd[1]*D[1];
    // effort[2] = Kp[2]*P[2] - 9.81 + Kd[2]*D[2];

    float Kp[3]={800,800,5};
    float Kd[3]={15,15,8};
    double effort[3];
    effort[0] = Kp[0]*P[0] + Kd[0]*D[0];
    effort[1] = Kp[1]*P[1] + Kd[1]*D[1];
    effort[2] = Kp[2]*P[2] + Kd[2]*D[2] - 9.81;
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n Joint Efforts applied are :\n [q1,q2,q3] = {['%f','%f','%f']}.\n",effort[0],effort[1],effort[2]);
    
    auto end_effector = std_msgs::msg::Float64MultiArray();
    end_effector.data = {effort[0],effort[1],effort[2]};
    publisher_->publish(end_effector);

    if(start){
      std::ofstream OutFile;
      OutFile.open ("Velocity.txt", std::ofstream::out | std::ofstream::app);
      OutFile <<t_c<<";"<<v1_d<<";"<<end_vel[0]<<";"<<v2_d<<";"<<end_vel[1]<<";"<<v3_d<<";"<<end_vel[2]<<std::endl;
      OutFile.close();
    }
  }

  // initializing the subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Service<custom_msgs::srv::JointToEnd>::SharedPtr serviceF;
  rclcpp::Service<custom_msgs::srv::EndToJoint>::SharedPtr serviceI;
  //Desired joints
  float v1_d,v2_d,v3_d;
  float q1_c,q2_c,q3_c;
  double J_1[3],J_2[3],J_3[3];
  bool start;
  float tp;
  float err[10][3];
  int idx;
  float P[3];
  float D[3];
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc,argv);
  // initializing the Node
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready for Velocity Control.");
  rclcpp::spin(std::make_shared<Velocity_Node>());
  rclcpp::shutdown();
  return 0;
}
