#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Wrench.h"
#include "PID.hpp"
#include "FuzzyPID.hpp"
#include "std_msgs/String.h"
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <filesystem>
#include <vector>
#include <map>
#include <string>


class PositionControlNode{
    
    private:
        std::map<std::string, double> config;
        std::vector<double> pos_des;
        std::vector<double> quat_des;
        bool initialized;
        PID * control_rot;
        PID * control_pos;
        FuzzyPID * control_rot_f;
        FuzzyPID * control_pos_f;
        ros::Subscriber sub_cmd_pos;
        ros::Subscriber sub_odometry;
        ros::Publisher pub_cmd_thrust;
        int type;

    public:

       // void cmd_pose_callback(geometry_msgs::Pose msg);
       // void odom_callback(geometry_msgs::PoseStamped msg);
        

        PositionControlNode(){
			
            pos_des = {0,0,0};
            quat_des = {0,0,0,-1};
            initialized = false;
            
            ros::NodeHandle n;

            std::cout << "Select Type of Controller\n1). Simple PID   2). Fuzzy PID\n" << std::endl;
            std::cin >> type;
            if(type != 1 && type != 2){
                std::cout << "Not available\n";
            }
            
            setConfig();
            sub_cmd_pos = n.subscribe<geometry_msgs::Pose>("cmd_pose", 10, &PositionControlNode::cmd_pose_callback, this);
			sub_odometry = n.subscribe<geometry_msgs::PoseStamped>("odom", 10, &PositionControlNode::odom_callback, this);
            pub_cmd_thrust = n.advertise<geometry_msgs::Wrench>("thruster_manager/input", 10);
        }

        void setConfig(){
			std::filesystem::path dir = std::filesystem::path(__FILE__).parent_path();
			if(type == 1){
                std::filesystem::path file = dir / "../config/PID_params.yaml";
			
                YAML::Node config = YAML::LoadFile(file);

                control_pos = new PID(config["pos_p"].as<double>(), config["pos_i"].as<double>(), config["pos_d"].as<double>(), config["pos_alpha"].as<double>());
                control_rot = new PID(config["rot_p"].as<double>(), config["rot_i"].as<double>(), config["rot_d"].as<double>(), config["rot_alpha"].as<double>());

                std::cout << "Config for PID set\n" << std::endl;
            }
            else if(type == 2){
                std::filesystem::path file = dir / "../config/FuzzyPID_params.yaml";
                YAML::Node config = YAML::LoadFile(file);

                gainRange pos_range = {
                    {config["pos_deledot"][0].as<double>(), config["pos_deledot"][1].as<double>()},
                    {config["pos_dele"][0].as<double>(), config["pos_dele"][1].as<double>()},
                    {config["pos_delkp"][0].as<double>(), config["pos_delkp"][1].as<double>()},
                    {config["pos_delki"][0].as<double>(), config["pos_delki"][1].as<double>()},
                    {config["pos_delkd"][0].as<double>(), config["pos_delkd"][1].as<double>()}
                };

                gainRange rot_range = {
                    {config["rot_deledot"][0].as<double>(), config["rot_deledot"][1].as<double>()},
                    {config["rot_dele"][0].as<double>(), config["rot_dele"][1].as<double>()},
                    {config["rot_delkp"][0].as<double>(), config["rot_delkp"][1].as<double>()},
                    {config["rot_delki"][0].as<double>(), config["rot_delki"][1].as<double>()},
                    {config["rot_delkd"][0].as<double>(), config["rot_delkd"][1].as<double>()}
                };

                control_pos_f = new FuzzyPID(config["pos_p"].as<double>(), config["pos_i"].as<double>(), config["pos_d"].as<double>(), pos_range);

                control_rot_f = new FuzzyPID(config["rot_p"].as<double>(), config["rot_i"].as<double>(), config["rot_d"].as<double>(), rot_range);

                std::cout << "Config for Fuzzy PID set\n" << std::endl;
            }
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

            Eigen::Quaterniond q(quat_cur[3],quat_cur[0], quat_cur[1], quat_cur[2]);
            q.normalize();
            Eigen::Matrix4d quat_mat = Eigen::Matrix4d::Identity();
            quat_mat.topLeftCorner<3,3>() = q.toRotationMatrix();

            Eigen::Vector3d e_pos_world_vec;
            e_pos_world_vec << e_pos_world[0], e_pos_world[1], e_pos_world[2];
            Eigen::Vector3d e_pos_body_vec = quat_mat.transpose().block<3,3>(0,0) * e_pos_world_vec;
            //std::cout << e_pos_body_vec << std::endl;
            std::vector<double> e_pos_body = {e_pos_body_vec[0], e_pos_body_vec[1], e_pos_body_vec[2]};

            Eigen::Quaterniond e_rot_quat = q.conjugate() * Eigen::Quaterniond(quat_des[3], quat_des[0], quat_des[1], quat_des[2]);
            e_rot_quat.normalize();
            Eigen::Vector3d euler_angles = e_rot_quat.toRotationMatrix().eulerAngles(2,1,0); //along the ZYX axis;
            std::vector<double> e_rot = {euler_angles[2], euler_angles[1], euler_angles[0]};
            //std::cout << euler_angles << std::endl;

            double t = msg->header.stamp.toSec();

            geometry_msgs::Wrench cmd_thrust;
            for(int i = 0; i < 3; i++){
				//std::cout << e_pos_body[i] << std::endl;
                double pos_out, rot_out;
                if(type == 1){
                    double pos_out = control_pos->update(e_pos_body[i], t);
                    double rot_out = control_rot->update(e_rot[i], t);
                }
                else if(type == 2){
                    double pos_out = control_pos_f->update(e_pos_body[i], t);
                    double rot_out = control_rot_f->update(e_rot[i], t);
                }

                if(i == 0){
                    cmd_thrust.force.x = pos_out;
                    cmd_thrust.torque.x = rot_out;
                }
                else if(i == 1){
                    cmd_thrust.force.y = pos_out;
                    cmd_thrust.torque.y = rot_out;
                }
                else if(i == 2){
                    cmd_thrust.force.z = pos_out;
                    cmd_thrust.torque.z = rot_out;
                }
            }

            pub_cmd_thrust.publish(cmd_thrust);

        }
};

int main(int argc, char ** argv){
    std::cout << "Starting: Position Control Node\n";
    ros::init(argc, argv, "position_control_node");
    

    try{
        PositionControlNode node;
        ros::spin();
    }catch(...){
        std::cout << "Exception!\n";
    }

    std::cout << "Exiting\n";
}
