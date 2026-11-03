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

    // 初始化期望速度缓冲区
    // target_vel_fl_.writeFromNonRT(0.0);
    // target_vel_fr_.writeFromNonRT(0.0);
    // target_vel_bl_.writeFromNonRT(0.0);
    // target_vel_br_.writeFromNonRT(0.0);

    // 初始化物理参数
    wheel_radius_ = 0.07625; // 轮子半径 (m)
    track_width_ = 0.48;     // 左右轮距 (m)
    wheelbase_ = 0.32;       // 前后轴距 (m)
    max_effort_ = 10.0;      // 力矩限制

    // 初始化 cmd_vel 订阅器 (话题名就是 /cmd_vel)
    cmd_vel_sub_ = controller_nh.subscribe("/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);
    
    // 初始化缓冲区，默认全 0
    geometry_msgs::Twist init_twist;
    init_twist.linear.x = 0;
    init_twist.linear.y = 0;
    init_twist.angular.z = 0;
    cmd_vel_buffer_.writeFromNonRT(init_twist);

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

    ROS_INFO_STREAM_THROTTLE(1.0, "Vx=" << Vx << " Vy=" << Vy << " Wz=" << Wz);

    double Lx = track_width_ / 2.0;
    double Ly = wheelbase_ / 2.0;
    double L = Lx + Ly; // 中心到轮子的曼哈顿距离

    // 根据vx,vy,wz 计算四个轮子的期望线速度 (m/s)
    double v_fl = Vx - Vy - L * Wz;
    double v_fr = Vx + Vy + L * Wz;
    double v_bl = Vx + Vy - L * Wz;
    double v_br = Vx - Vy + L * Wz;

    // 将线速度转换为角速度 (rad/s)
    double target_vel_fl = 0.05*v_fl / wheel_radius_;
    double target_vel_fr = 0.05*v_fr / wheel_radius_;
    double target_vel_bl = 0.05*v_bl / wheel_radius_;
    double target_vel_br = 0.05*v_br / wheel_radius_;

    // 3. 获取实际速度
    double current_vel_fl = front_left_joint_.getVelocity();
    double current_vel_fr = front_right_joint_.getVelocity();
    double current_vel_bl = back_left_joint_.getVelocity();
    double current_vel_br = back_right_joint_.getVelocity();

    // 4. 用 computeCommand 直接计算力矩
    // 注意：computeCommand 的标准用法是 (目标值, 当前值, 周期)
    double effort_fl = pid_front_left_.computeCommand(target_vel_fl, current_vel_fl, period);
    double effort_fr = pid_front_right_.computeCommand(target_vel_fr, current_vel_fr, period);
    double effort_bl = pid_back_left_.computeCommand(target_vel_bl, current_vel_bl, period);
    double effort_br = pid_back_right_.computeCommand(target_vel_br, current_vel_br, period);

    // 5. 安全限幅（防止重蹈覆辙）
    if (effort_fl > max_effort_) effort_fl = max_effort_; else if (effort_fl < -max_effort_) effort_fl = -max_effort_;
    if (effort_fr > max_effort_) effort_fr = max_effort_; else if (effort_fr < -max_effort_) effort_fr = -max_effort_;
    if (effort_bl > max_effort_) effort_bl = max_effort_; else if (effort_bl < -max_effort_) effort_bl = -max_effort_;
    if (effort_br > max_effort_) effort_br = max_effort_; else if (effort_br < -max_effort_) effort_br = -max_effort_;

    // 6. 下发力矩
    front_left_joint_.setCommand(effort_fl);
    front_right_joint_.setCommand(effort_fr);
    back_left_joint_.setCommand(effort_bl);
    back_right_joint_.setCommand(effort_br);
}
 // namespace hero_chassis_controller
}
// 导出插件
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)