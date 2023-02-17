#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/srv/pose_to_array.hpp"
#include <memory>
using std::placeholders::_1;

void ik_solver(const std::shared_ptr<custom_msgs::srv::PoseToArray::Request> request, 
          std::shared_ptr<custom_msgs::srv::PoseToArray::Response>       response)
{
    float q1_1,q2_1,q2_2,q1_2,q3;
    float ax = 0.05;
    float l0 = 2.0;
    float l1 = 1.0-2*ax;
    float l2 = 1.0;
    // read the input message position
    double px = request->px;
    double py = request->py;
    double pz = request->pz;
    int precise;
    // calculate d3 i.e. q3
    q3 = l0 - pz;

    //calculate possible theta 3 values i.e. q3_1,q3_2
    double r = sqrt(px*px + py*py);
    double cq2 = (r*r - l1*l1 - l2*l2)/(2*l1*l2);
    precise =  cq2*10000;
    cq2 = precise/10000.00;
    double sq2 = sqrt(1-(cq2*cq2));
    q2_1 = atan2(sq2,cq2);
    q2_2 = atan2(-1*sq2,cq2);

    //calculate possible theta2 values for all theta 3 values i.e. q2_1,q2_2
    q1_1 = (atan2(py,px) - atan2(-1*l2*sq2,l1+l2*cq2));
    q1_2 = (atan2(py,px) - atan2(l2*sq2,l1+l2*cq2));
    
    
    precise = q2_1*10000;
    q2_1 = precise/10000.00;
    precise = q2_2*10000;
    q2_2 = precise/10000.00;
    precise = q1_1*10000;
    q1_1 = precise/10000.00;
    precise = q1_2*10000;
    q1_2 = precise/10000.00;
    
    //print the calculated values to the terminal
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n Inverse Kinematic solutions for received Pose are :\n [q1,q2,q3] = {['%f','%f','%f'],['%f','%f','%f']}.\n",q1_1,q2_1,q3,q1_2,q2_2,q3);
    response->sol1 = {q1_1,q2_1,q3};
    response->sol2 = {q1_2,q2_2,q3};
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("Inverse_Kinematics");

  rclcpp::Service<custom_msgs::srv::PoseToArray>::SharedPtr service =
    node->create_service<custom_msgs::srv::PoseToArray>("ik_solver", &ik_solver);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to solve Inverse Kinematics.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
