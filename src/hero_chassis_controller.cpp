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

    // 初始化 cmd_vel 订阅器 (话题名就是 /cmd_vel)
    cmd_vel_sub_ = controller_nh.subscribe("/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);
    geometry_msgs::Twist init_twist;
    cmd_vel_buffer_.writeFromNonRT(init_twist);

    // 初始化里程计发布者
    odom_pub_ = controller_nh.advertise<nav_msgs::Odometry>("/odom", 50);
    x_ = 0.0; y_ = 0.0; th_ = 0.0;
    last_time_ = ros::Time::now();

    // 读取模式参数，默认使用底盘局部模式
    controller_nh.param<bool>("use_global_frame", use_global_frame_, false);

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

// 键盘指令回调函数
void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
    cmd_vel_buffer_.writeFromNonRT(msg);
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
    
    // 1. 获取最新的速度指令
    geometry_msgs::Twist cmd_vel = *(cmd_vel_buffer_.readFromRT());

    // 2. 逆运动学计算（麦克纳姆轮标准公式）
    // 按键盘后接收 Vx: 前后， Vy: 左右， Wz: 旋转
    double Vx = cmd_vel.linear.x;
    double Vy = cmd_vel.linear.y;
    double Wz = cmd_vel.angular.z;

        // 全局/局部速度坐标系变换 
    if (use_global_frame_) {
        try {
            // 查询 odom 到 base_link 的最新变换
            tf::StampedTransform transform;
            tf_listener_.lookupTransform("odom", "base_link", ros::Time(0), transform);
            
            // 提取当前的偏航角 (Yaw)
            double roll, pitch, yaw;
            tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
            
            // 旋转矩阵变换（将全局 Vx, Vy 转到 底盘坐标系下）
            // 原理：如果你在世界坐标系朝正东走，但车头朝北，你实际需要向右平移
            double local_vx = Vx * cos(yaw) + Vy * sin(yaw);
            double local_vy = -Vx * sin(yaw) + Vy * cos(yaw);
            
            Vx = local_vx;
            Vy = local_vy;
            // 角速度 Wz 不需要转换，因为它本身就是相对的
        } catch (tf::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "TF 查询失败: %s", ex.what());
        }
    }

    // 3. 逆运动学 
    double Lx = track_width_ / 2.0;
    double Ly = wheelbase_ / 2.0;
    double L = Lx + Ly; 

    double v_fl = Vx - Vy - L * Wz;
    double v_fr = Vx + Vy + L * Wz;
    double v_bl = Vx + Vy - L * Wz;
    double v_br = Vx - Vy + L * Wz;

    double target_vel_fl = 2 * v_fl / wheel_radius_;
    double target_vel_fr = 2 * v_fr / wheel_radius_;
    double target_vel_bl = 2 * v_bl / wheel_radius_;
    double target_vel_br = 2 * v_br / wheel_radius_;

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
    double base_wz = (-vx_fl + vx_fr - vx_bl + vx_br) / (4.0 * L);

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