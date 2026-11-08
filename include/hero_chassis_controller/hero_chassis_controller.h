#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // 用于接收 cmd_vel
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h> // 新增：用于查询 TF
#include <tf/transform_datatypes.h>

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
    void cmdVelCallback(const geometry_msgs::Twist& msg);

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

    ros::Subscriber cmd_vel_sub_;
    realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;
    

    // 物理参数
    double wheel_radius_;
    double track_width_;
    double wheelbase_;
    double max_effort_;
    bool use_global_frame_; // 模式开关

    // 里程计相关 
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster_;
    
    double x_;      // 当前 X 坐标
    double y_;      // 当前 Y 坐标
    double th_;     // 当前航向角 (yaw)
    ros::Time last_time_; // 上一次更新里程计的时间

    // TF查询器
    tf::TransformListener tf_listener_;
};

} // namespace hero_chassis_controller

#endif // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H