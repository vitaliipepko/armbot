#include "armbot_controller/armbot_interface.h"
#include "armbot_controller/AnglesConverter.h"
#include <std_msgs/UInt16MultiArray.h>


ArmbotInterface::ArmbotInterface(ros::NodeHandle& nh) : nh_(nh), 
            pnh_("~"),
            pos_(4, 0),
            vel_(4, 0),
            eff_(4, 0),
            cmd_(4, 0),
            names_{"joint_1", "joint_2", "joint_3", "joint_4"}
{
    pnh_.param("joint_names", names_, names_);

    hardware_pub_ = pnh_.advertise<std_msgs::UInt16MultiArray>("/arduino/arm_actuate", 1000);
    hardware_srv_ = pnh_.serviceClient<armbot_controller::AnglesConverter>("/radians_to_degrees");
    
    ROS_INFO("Starting Armbot Hardware Interface...");

    hardware_interface::JointStateHandle state_handle1(names_.at(0), &pos_.at(0), &vel_.at(0), &eff_.at(0));
    joint_state_interface_.registerHandle(state_handle1);
    hardware_interface::JointStateHandle state_handle2(names_.at(1), &pos_.at(1), &vel_.at(1), &eff_.at(1));
    joint_state_interface_.registerHandle(state_handle2);
    hardware_interface::JointStateHandle state_handle3(names_.at(2), &pos_.at(2), &vel_.at(2), &eff_.at(2));
    joint_state_interface_.registerHandle(state_handle3);
    hardware_interface::JointStateHandle state_handle4(names_.at(3), &pos_.at(3), &vel_.at(3), &eff_.at(3));
    joint_state_interface_.registerHandle(state_handle4);

    registerInterface(&joint_state_interface_);

    hardware_interface::JointHandle position_handle1(joint_state_interface_.getHandle(names_.at(0)), &cmd_.at(0));
    joint_position_interface_.registerHandle(position_handle1);
    hardware_interface::JointHandle position_handle2(joint_state_interface_.getHandle(names_.at(1)), &cmd_.at(1));
    joint_position_interface_.registerHandle(position_handle2);
    hardware_interface::JointHandle position_handle3(joint_state_interface_.getHandle(names_.at(2)), &cmd_.at(2));
    joint_position_interface_.registerHandle(position_handle3);
    hardware_interface::JointHandle position_handle4(joint_state_interface_.getHandle(names_.at(3)), &cmd_.at(3));
    joint_position_interface_.registerHandle(position_handle4);

    registerInterface(&joint_position_interface_);

    ROS_INFO("Interfaces registered.");


    ROS_INFO("Preparing the Controller Manager");

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    update_freq_ = ros::Duration(0.1);
    looper_ = nh_.createTimer(update_freq_, &ArmbotInterface::update, this);
    
    ROS_INFO("Ready to execute the control loop");
}

void ArmbotInterface::update(const ros::TimerEvent& e)
{
    ROS_INFO("Update Event");
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ArmbotInterface::read()
{
    pos_.at(0) = cmd_.at(0);
    pos_.at(1) = cmd_.at(1);
    pos_.at(2) = cmd_.at(2);
    pos_.at(3) = cmd_.at(3);
}

void ArmbotInterface::write(ros::Duration elapsed_time)
{   
    armbot_controller::AnglesConverter srv;
    srv.request.base = cmd_.at(0);
    srv.request.shoulder = cmd_.at(1);
    srv.request.elbow = cmd_.at(2);
    srv.request.gripper = cmd_.at(3);

    if (hardware_srv_.call(srv))
    {
        std::vector<unsigned int> angles_deg;
        angles_deg.push_back(srv.response.base);
        angles_deg.push_back(srv.response.shoulder);
        angles_deg.push_back(srv.response.elbow);
        angles_deg.push_back(srv.response.gripper);

        std_msgs::UInt16MultiArray msg;

        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].size = angles_deg.size();
        msg.layout.dim[0].stride = 1;

        msg.data.clear();
        msg.data.insert(msg.data.end(), angles_deg.begin(), angles_deg.end());

        hardware_pub_.publish(msg);
    }
    else
    {
        ROS_ERROR("Failed to call service radians_to_degrees");
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "armbot_interface_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    ArmbotInterface robot(nh);

    spinner.spin();
    return 0;
}