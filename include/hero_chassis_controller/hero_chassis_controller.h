#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    HeroChassisController() = default;
    ~HeroChassisController() override = default;

    bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
              ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    void starting(const ros::Time &time) override;
    void stopping(const ros::Time &time) override;

private:
    hardware_interface::JointHandle front_left_joint_;
    hardware_interface::JointHandle front_right_joint_;
    hardware_interface::JointHandle back_left_joint_;
    hardware_interface::JointHandle back_right_joint_;

    // 四个独立的PID控制器
    control_toolbox::Pid pid_front_left_;
    control_toolbox::Pid pid_front_right_;
    control_toolbox::Pid pid_back_left_;
    control_toolbox::Pid pid_back_right_;

    // 存储期望速度（使用实时缓冲区保证线程安全）
    realtime_tools::RealtimeBuffer<double> target_vel_fl_;
    realtime_tools::RealtimeBuffer<double> target_vel_fr_;
    realtime_tools::RealtimeBuffer<double> target_vel_bl_;
    realtime_tools::RealtimeBuffer<double> target_vel_br_;
};

} // namespace hero_chassis_controller

#endif // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H