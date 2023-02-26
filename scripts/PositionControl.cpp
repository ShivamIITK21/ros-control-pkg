#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Wrench.h"
#include "PID.hpp"
#include "std_msgs/String.h"
#include <iostream>
#include <vector>
#include <map>
#include <string>

class TestNode{
    public:
        TestNode(){
            ros::NodeHandle n;
            ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",    1000);
            ros::Rate loop_rate(10);

            int count = 0;
            while(ros::ok()){
                std_msgs::String msg;
                std::stringstream ss;
                ss << "Hello World" << count;
                msg.data = ss.str();
                ROS_INFO("%s", msg.data.c_str());
                chatter_pub.publish(msg);
                ros::spinOnce();
                loop_rate.sleep();
                ++count;
            }
        }
};

class PositionControlNode{
    
    private:
        std::map<std::string, double> config;
        std::vector<double> pos_des;
        std::vector<double> quat_des;
        bool initialized;
        PID * control_rot;
        PID * control_pos;
        ros::Subscriber sub_cmd_pos;
        ros::Subscriber sub_odometry;
        ros::Publisher pub_cmd_thrust;


    public:
        PositionControlNode(){
            std:: cout << "HELLO"; 
        }
};

int main(int argc, char ** argv){
    std::cout << "Starting Test Node\n";
    ros::init(argc, argv, "test_node");

    try{
        TestNode();
    }catch(...){
        std::cout << "Exception!\n";
    }

    std::cout << "Exiting\n";
}
