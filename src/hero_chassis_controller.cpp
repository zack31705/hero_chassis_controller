#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.h>

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

    // 初始化物理参数
    wheel_radius_ = 0.07625; // 轮子半径 (m)
    track_width_ = 0.48;     // 左右轮距 (m)
    wheelbase_ = 0.32;       // 前后轴距 (m)
    max_effort_ = 20.0;      // 力矩限制

    // 初始化期望速度缓冲区
    target_vel_fl_.writeFromNonRT(0.0);
    target_vel_fr_.writeFromNonRT(0.0);
    target_vel_bl_.writeFromNonRT(0.0);
    target_vel_br_.writeFromNonRT(0.0);

    // 初始化里程计发布者
    odom_pub_ = controller_nh.advertise<nav_msgs::Odometry>("/odom", 50);
    x_ = 0.0; y_ = 0.0; th_ = 0.0;
    last_time_ = ros::Time::now();

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
    
    //心型轨迹
    static ros::Time start_time = ros::Time::now();
    double t = (time - start_time).toSec();

    const double pi = 3.1415926;
    const double tau = 18.5 * pi * track_width_ / 12.0; 

    double Lx = track_width_ / 2.0;
    double Ly = wheelbase_ / 2.0;
    double l_sum = Lx + Ly; // 中心到轮子的曼哈顿距离

    double target_vel_fl = 0.0;
    double target_vel_fr = 0.0;
    double target_vel_bl = 0.0;
    double target_vel_br = 0.0;

    if (t <= tau) {
        target_vel_fl = 1.20 / wheel_radius_;
        target_vel_fr = 2.40 / wheel_radius_;
        target_vel_bl = 1.20 / wheel_radius_;
        target_vel_br = 2.40 / wheel_radius_;
    } else if (t <= 2.5 * tau) {
        target_vel_fl = 1.50 / wheel_radius_;
        target_vel_fr = 2.10 / wheel_radius_;
        target_vel_bl = 1.50 / wheel_radius_;
        target_vel_br = 2.10 / wheel_radius_;
    } else if (t <= 3.47 * tau) {
        target_vel_fl = 1.20 / wheel_radius_;
        target_vel_fr = 2.40 / wheel_radius_;
        target_vel_bl = 1.20 / wheel_radius_;
        target_vel_br = 2.40 / wheel_radius_;
    } else {
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

    //以上是为了贴近目标速度，下面用实际速度来计算坐标

    // 将实际角速度 (rad/s) 转换为轮子线速度 (m/s)
    double vx_fl = current_vel_fl * wheel_radius_;
    double vx_fr = current_vel_fr * wheel_radius_;
    double vx_bl = current_vel_bl * wheel_radius_;
    double vx_br = current_vel_br * wheel_radius_;

    // 正运动学矩阵 (逆矩阵推导得出)
    double base_vx = (vx_fl + vx_fr + vx_bl + vx_br) / 4.0;
    double base_vy = (-vx_fl + vx_fr + vx_bl - vx_br) / 4.0;
    double base_wz = (-vx_fl + vx_fr - vx_bl + vx_br) / (4.0 * l_sum);

    // 时间积分 (计算这段时间走了多远)
    double dt = (time - last_time_).toSec();
    if (dt <= 0.0) dt = period.toSec(); // 防除零
    last_time_ = time;

    // 坐标累加
    double delta_x = (base_vx * cos(th_) - base_vy * sin(th_)) * dt;
    double delta_y = (base_vx * sin(th_) + base_vy * cos(th_)) * dt;
    double delta_th = base_wz * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    // 发布 TF 变换 (odom -> base_link)
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
    
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;// 组织被发布消息

    odom_broadcaster_.sendTransform(odom_trans);

    // 发布 odom 话题
    nav_msgs::Odometry odom;
    odom.header.stamp = time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = base_vx;
    odom.twist.twist.linear.y = base_vy;
    odom.twist.twist.angular.z = base_wz;
    
    odom_pub_.publish(odom);

}
 // namespace hero_chassis_controller
}
// 导出插件
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)