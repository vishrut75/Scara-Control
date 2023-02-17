#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

//Node Class definition
class Forward_Kinematics : public rclcpp::Node
{
public:
  Forward_Kinematics()
  : Node("Forward_Kinematics")
  { // defining both the subscribers
    // Subscriber 1 to read data from "topic" and calculate forward kinematics
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states",10, std::bind(&Forward_Kinematics::topic_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::String>("End_Pose", 10);
  }

private:
  // Forward Kinematics callback function -> prints homogeneous matrix for input configuration
  void topic_callback(const sensor_msgs::msg::JointState & message) const
  {
    //reading the input configuration
    float q1,q2,q3;
    q1 = message.position[0];
    q2 = message.position[1];
    q3 = message.position[2];

    //defining the system for the input configuration
    float c1,c2,s1,s2,l0,l1,l2,D3,ax;
    l0=2.0;
    l1=1.0;
    l2=1.0;
    ax=0.05;
    D3 = q3;
    c1 = cos(q1);
    s1 = sin(q1);
    c2 = cos(q2);
    s2 = sin(q2);
    
    //calculating homogenous matrices
    float H30[4][4] = {{c1*c2 - s1*s2, - c1*s2 - c2*s1, 0, c1*c2*l2 - c1*(2*ax - l1) - l2*s1*s2},{c1*s2 + c2*s1,   c1*c2 - s1*s2, 0, c1*l2*s2 - s1*(2*ax - l1) + c2*l2*s1},{0,0,1,l0-D3},{0,0,0,1}};

    //converting the double data type homogenous matrix to string for logging
    std::string row1="";
    for(int i=0;i<4;i++){
      row1+= "["+std::to_string(H30[i][0])+","+std::to_string(H30[i][1])+","+std::to_string(H30[i][2])+","+std::to_string(H30[i][3])+"]\n"; 
    }

    //printing the homogeneous matrix to the terminal
    //RCLCPP_INFO(this->get_logger(), "\nI received ['%f','%f','%f'].\n Pose of end effector can be represented using homogeneous matrix as:\n %s",q1,q2,q3,row1.c_str());
    
    auto end_effector = std_msgs::msg::String();
    end_effector.data = row1;
    publisher_->publish(end_effector);
    
  }

  // initializing the subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc,argv);
  // initializing the Node
  rclcpp::spin(std::make_shared<Forward_Kinematics>());
  rclcpp::shutdown();
  return 0;
}
