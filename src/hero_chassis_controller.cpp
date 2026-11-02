#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.h>

#define pi 3.1415926
#define L 0.48

namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                   ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    // 1. 获取关节句柄
    front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
    front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
    back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
    back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

    // 2. 初始化 PID 控制器
    // 读取 YAML 中的命名空间，例如 "pid/front_left"
    ros::NodeHandle pid_nh(controller_nh, "pid");

    ros::NodeHandle fl_nh(pid_nh, "front_left");
    pid_front_left_.init(fl_nh);

    ros::NodeHandle fr_nh(pid_nh, "front_right");
    pid_front_right_.init(fr_nh);

    ros::NodeHandle bl_nh(pid_nh, "back_left");
    pid_back_left_.init(bl_nh);

    ros::NodeHandle br_nh(pid_nh, "back_right");
    pid_back_right_.init(br_nh);

    // 初始化期望速度缓冲区
    target_vel_fl_.writeFromNonRT(0.0);
    target_vel_fr_.writeFromNonRT(0.0);
    target_vel_bl_.writeFromNonRT(0.0);
    target_vel_br_.writeFromNonRT(0.0);

    return true;
}

void HeroChassisController::starting(const ros::Time &time) {
    pid_front_left_.reset();
    pid_front_right_.reset();
    pid_back_left_.reset();
    pid_back_right_.reset();
}

void HeroChassisController::stopping(const ros::Time &time) {
    front_left_joint_.setCommand(0.0);
    front_right_joint_.setCommand(0.0);
    back_left_joint_.setCommand(0.0);
    back_right_joint_.setCommand(0.0);
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {

    //pid控制英雄机器人走一个心形轨迹
    static ros::Time start_time = ros::Time::now();
    double t = (time - start_time).toSec();
    
    double target_vel_fl = 0.0;
    double target_vel_fr = 0.0;
    double target_vel_bl = 0.0;
    double target_vel_br = 0.0;

    static const double tai = 18*pi*L/12;
    static const double wheel_r = 0.07625; 

    if (t <= tai) {
        target_vel_fl = 1.20/wheel_r;
        target_vel_fr = 2.40/wheel_r;
        target_vel_bl = 1.20/wheel_r;
        target_vel_br = 2.40/wheel_r;
    }
    else if (t <= 3*tai) {
        target_vel_fl = 1.50/wheel_r;
        target_vel_fr = 2.10/wheel_r;
        target_vel_bl = 1.50/wheel_r;
        target_vel_br = 2.10/wheel_r;
    }
    else if (t <= 3.92*tai) {
        target_vel_fl = 1.20/wheel_r;
        target_vel_fr = 2.40/wheel_r;
        target_vel_bl = 1.20/wheel_r;
        target_vel_br = 2.40/wheel_r;
    }
    else {
        target_vel_fl = 0.0;
        target_vel_fr = 0.0;
        target_vel_bl = 0.0;
        target_vel_br = 0.0;
    }

    double current_vel_fl = front_left_joint_.getVelocity();
    double current_vel_fr = front_right_joint_.getVelocity();
    double current_vel_bl = back_left_joint_.getVelocity();
    double current_vel_br = back_right_joint_.getVelocity();

    double error_fl = target_vel_fl - current_vel_fl;
    double error_fr = target_vel_fr - current_vel_fr;
    double error_bl = target_vel_bl - current_vel_bl;
    double error_br = target_vel_br - current_vel_br;

    double effort_fl = pid_front_left_.computeCommand(error_fl, period);
    double effort_fr = pid_front_right_.computeCommand(error_fr, period);
    double effort_bl = pid_back_left_.computeCommand(error_bl, period);
    double effort_br = pid_back_right_.computeCommand(error_br, period);

    front_left_joint_.setCommand(effort_fl);
    front_right_joint_.setCommand(effort_fr);
    back_left_joint_.setCommand(effort_bl);
    back_right_joint_.setCommand(effort_br);
}
 // namespace hero_chassis_controller
}
// 导出插件
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)