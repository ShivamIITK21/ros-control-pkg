#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Wrench.h"
#include "PID.hpp"
#include "std_msgs/String.h"
#include <Eigen/Dense>
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

       // void cmd_pose_callback(geometry_msgs::Pose msg);
       // void odom_callback(geometry_msgs::PoseStamped msg);

        PositionControlNode(){
            pos_des = {0,0,0};
            quat_des = {0,0,0,-1};
            initialized = false;
            
            ros::NodeHandle n;

            control_rot = new PID(0.707, 0.1, 1.68, 0.05);
            control_pos = new PID(0.707, 0.1, 1.68, 0.05);
            
            sub_cmd_pos = n.subscribe<geometry_msgs::Pose>("cmd_pose", 10, &PositionControlNode::cmd_pose_callback, this);
           sub_odometry = n.subscribe<geometry_msgs::PoseStamped>("odom", 10, &PositionControlNode::odom_callback, this);
            pub_cmd_thrust = n.advertise<geometry_msgs::Wrench>("thruster_manager/input", 10);
        }


        void cmd_pose_callback(const geometry_msgs::Pose::ConstPtr& msg){
            auto pos = msg->position;
            auto quat = msg->orientation;
            pos_des = {pos.x, pos.y, pos.z};
            quat_des = {quat.x, quat.y, quat.z, quat.w};
        }

        void odom_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
            auto p = msg->pose.position;
            auto quat = msg->pose.orientation;
            
            std::vector<double> p_cur = {p.x, p.y, p.z};
            std::vector<double> quat_cur = {quat.x, quat.y, quat.z, quat.w};

            if(!initialized){
                pos_des = p_cur;
                quat_des = quat_cur;
                initialized = true;
            }

            //error calculation

            std::vector<double> e_pos_world = {
                pos_des[0]-p_cur[0],
                pos_des[1]-p_cur[1],
                pos_des[2]-p_cur[2]
            };

            Eigen::Quaternion q(quat_cur[3],quat_cur[0], quat_cur[1], quat_cur[2]);
            q.normalize();
            Eigen::Matrix4d quat_mat = Eigen::Matrix4d::Identity();
            mat.topLeftCorner<3,3>() = q.toRotationMatrix();

            Eigen::Vector3d e_pos_world_vec;
            e_pos_world_vec << e_pos_world[0], e_pos_world[1], e_pos_world[2];
            Eigen::Vector3d e_pos_body_vec = quat_mat.transpose().block<3,3>(0,0) * e_pos_world_vec;
            vector<double> e_pos_body = {e_pos_body_vec[0], e_pos_body_vec[1], e_pos_body[2]};

            Eigen::Vector4d e_rot_quat
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
