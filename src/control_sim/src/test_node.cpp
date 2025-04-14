#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <cstdlib>
#include <ctime>
#include <mutex>

class ArmController : public rclcpp::Node {
    public:
      ArmController() : Node("arm_controller") {
        // 声明参数
        this->declare_parameter<std::string>("urdf_path", "");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("ee_frame", "Link6");
        this->declare_parameter<double>("ik_tolerance", 1e-2);
        this->declare_parameter<int>("ik_max_iter", 100);
    
        // 初始化模型
        if (!initialize_robot_model()) {
          RCLCPP_FATAL(this->get_logger(), "模型初始化失败!");
          rclcpp::shutdown();
        }
    
        // 初始化关节状态
        current_q_.resize(model.nq);
        current_q_.setZero();
    
        // 创建订阅器和发布器
        cur_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "current_pose", 10);
        target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_commands", 10);
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "target_poses", 10,
          std::bind(&ArmController::pose_callback, this, std::placeholders::_1));
        gripper_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "gripper_cmd", 10,
            std::bind(&ArmController::gripper_callback, this, std::placeholders::_1));
        actual_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "actual_joint_states",  // 与实际硬件/仿真对接的话题
            10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
              this->actual_joint_callback(msg);
            });
    
        // 初始化计时器
        // timer_ = this->create_wall_timer(
        //   std::chrono::milliseconds(50),
        //   std::bind(&ArmController::timer_callback, this));
      }
    
    private:
      // 初始化机器人模型
      bool initialize_robot_model() {
        // 获取参数
        std::string urdf_path;
        this->get_parameter("urdf_path", urdf_path);
        this->get_parameter("base_frame", base_frame_);
        this->get_parameter("ee_frame", ee_frame_);
    
        // 加载URDF模型
        try {
          pinocchio::urdf::buildModel(urdf_path, model);
          data = pinocchio::Data(model);
          std::cout << "reading urdf model" << std::endl;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "加载URDF失败: %s", e.what());
          return false;
        }
    
        // 验证末端执行器帧
        ee_frame_id_ = model.getFrameId(ee_frame_);
        if (ee_frame_id_ >= model.nframes) {
          RCLCPP_ERROR(this->get_logger(), "末端帧 '%s' 不存在", ee_frame_.c_str());
          return false;
        } else {
            RCLCPP_INFO(this->get_logger(), "末端Frame[%zu]: %s", 
                ee_frame_id_, 
                model.frames[ee_frame_id_].name.c_str());
        }
    
        return true;
      }

      void actual_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // std::cout << "ininininiinj actual " << msg->name.size() << " " << msg->position.size() << std::endl;
        // std::lock_guard<std::mutex> lock(q_mutex_);
        
        // 数据有效性检查
        if (msg->name.size() != msg->position.size()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "无效关节状态消息: 名称与位置数量不匹配");
          return;
        }
    
        // 遍历更新每个关节
        for (size_t i = 0; i < msg->name.size(); ++i) {
          const std::string& joint_name = msg->name[i];
          // std::cout << "ininininiinj actual 1111" << std::endl;
          try {
            const auto joint_id = model.getJointId(joint_name);
            const auto& joint = model.joints[joint_id];
            // std::cout << "ininininiinj actual 2222" << current_q_.size() << std::endl;
            if (current_q_.size() == msg->position.size()) {
                current_q_[i] = msg->position[i];
                // std::cout << "ininiin sideeee " << current_q_[i] << std::endl;
            }
            
            // 仅处理可移动关节
            // if (joint.idx_q() >= 0 && joint.nq() == 1) {
            //   const size_t idx = static_cast<size_t>(joint.idx_q());
            //   std::cout << "ininininiinj actual 33333 " << joint.idx_q() << " " << current_q_.size() << std::endl;
            //   if (idx < current_q_.size()) {
            //     current_q_[idx] = msg->position[i];
            //     std::cout << "ininiin " << current_q_[idx] << std::endl;
            //   }
            // }
          } catch (const std::invalid_argument& e) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "忽略未知关节: %s", joint_name.c_str());
          }
        }
        pinocchio::forwardKinematics(model, data, current_q_);
        pinocchio::updateFramePlacement(model, data, ee_frame_id_);
        current_pose_ = data.oMf[ee_frame_id_];
        // 转换为ROS消息
        auto msg_pose_stamp = geometry_msgs::msg::PoseStamped();
        msg_pose_stamp.header.stamp = this->now();
        msg_pose_stamp.header.frame_id = "base_link";  // 根据实际情况修改坐标系

        // 设置位置
        msg_pose_stamp.pose.position.x = current_pose_.translation().x();
        msg_pose_stamp.pose.position.y = current_pose_.translation().y();
        msg_pose_stamp.pose.position.z = current_pose_.translation().z();

        // 设置方向（将旋转矩阵转换为四元数）
        Eigen::Quaterniond quat(current_pose_.rotation());
        msg_pose_stamp.pose.orientation.x = quat.x();
        msg_pose_stamp.pose.orientation.y = quat.y();
        msg_pose_stamp.pose.orientation.z = quat.z();
        msg_pose_stamp.pose.orientation.w = quat.w();

        // 发布消息
        cur_pose_pub_->publish(msg_pose_stamp);
        // fake_target_command();
      }

      // void fake_target_command() {
      //   pinocchio::forwardKinematics(model, data, current_q_);
      //   pinocchio::updateFramePlacement(model, data, ee_frame_id_);
      //   const pinocchio::SE3 current_pose = data.oMf[ee_frame_id_];

      //   // 创建目标位姿（Z轴增加偏移）
      //   const double z_offset = 0.1;  // 单位：米
      //   const double x_offset = 0.1;
      //   pinocchio::SE3 target_pose = current_pose;
      //   std::cout << "x " << target_pose.translation().x();
      //   std::cout << "y " << target_pose.translation().y();
      //   std::cout << "z " << target_pose.translation().z();
      //   target_pose.translation().z() += z_offset;
      //   target_pose.translation().x() += x_offset;
      //   Eigen::Matrix3d R_z_90 = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

      //   // (2) 将旋转应用到 target_pose 的旋转部分
      //   target_pose.rotation() = target_pose.rotation() * R_z_90;
      //   std::cout << "x after " << target_pose.translation().x();
      //   std::cout << "y after " << target_pose.translation().y();
      //   std::cout << "z after " << target_pose.translation().z();

      //   // 转换为ROS消息
      //   auto msg = geometry_msgs::msg::PoseStamped();
      //   msg.header.stamp = this->now();
      //   msg.header.frame_id = "base_link";  // 根据实际情况修改坐标系

      //   // 设置位置
      //   msg.pose.position.x = target_pose.translation().x();
      //   msg.pose.position.y = target_pose.translation().y();
      //   msg.pose.position.z = target_pose.translation().z();

      //   // 设置方向（将旋转矩阵转换为四元数）
      //   Eigen::Quaterniond quat(target_pose.rotation());
      //   msg.pose.orientation.x = quat.x();
      //   msg.pose.orientation.y = quat.y();
      //   msg.pose.orientation.z = quat.z();
      //   msg.pose.orientation.w = quat.w();

      //   // 发布消息
      //   target_pose_pub_->publish(msg);

      //   RCLCPP_INFO(this->get_logger(), "已发布目标位姿 Z: %.3f", 
      //               target_pose.translation().z());
      // }
    
      // 位姿回调函数
      void gripper_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        gripper_7_ = msg->position.x;
        gripper_8_ = msg->position.y;
      }

      void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 转换ROS消息到SE3
        std::cout << "testtttt" << std::endl;
        const auto& p = msg->pose.position;
        const auto& q = msg->pose.orientation;
        Eigen::Vector3d position(p.x, p.y, p.z);
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        
        target_pose_.translation() = position;
        target_pose_.rotation() = quat.toRotationMatrix();
    
        // 求解逆运动学
        if (solve_ik(target_pose_)) {
            target_q_(6) = gripper_7_;
            target_q_(7) = gripper_8_;
            publish_joint_commands(target_q_);
            RCLCPP_INFO(this->get_logger(), "逆解成功，发送命令");
        } else {
          RCLCPP_WARN(this->get_logger(), "逆解失败，保持当前状态");
        }
      }
    
      // 逆运动学求解
      bool solve_ik(const pinocchio::SE3& target) {
        Eigen::VectorXd initial_q;
    
    // 安全获取当前实际状态
        {
    //   std::lock_guard<std::mutex> lock(q_mutex_);
            initial_q = current_q_;
            temp_cal_q_ = initial_q;
        }

    // 使用实际状态作为初始值进行迭代
        Eigen::VectorXd q = initial_q;
        // 获取当前末端位姿
        pinocchio::forwardKinematics(model, data, initial_q);
        pinocchio::updateFramePlacement(model, data, ee_frame_id_);
        const pinocchio::SE3 current_pose = data.oMf[ee_frame_id_];
    
        // 设置逆解参数
        const double tolerance = this->get_parameter("ik_tolerance").as_double();
        const int max_iter = this->get_parameter("ik_max_iter").as_int();
    
        // 使用伪逆法迭代求解
        Eigen::VectorXd dq(model.nv);
        for (int i = 0; i < max_iter; ++i) {
          // 计算误差
          pinocchio::forwardKinematics(model, data, temp_cal_q_);
          pinocchio::updateFramePlacement(model, data, ee_frame_id_);
          const pinocchio::SE3 current_pose = data.oMf[ee_frame_id_];
        //   const pinocchio::Motion err = pinocchio::log6(target.actInv(current_pose));

          pinocchio::Motion err;
          Eigen::Vector3d pos_err = target.translation() - current_pose.translation();
          for (int i = 0; i < 3; i++) {
            if (abs(pos_err(i)) < 5e-4) {
              pos_err(i) = 0;
            }
          }

        // 旋转误差：用旋转矩阵的轴角表示
        Eigen::Matrix3d R_err = target.rotation() * current_pose.rotation().transpose();
        Eigen::AngleAxisd angle_axis(R_err);
        Eigen::Vector3d rot_err = angle_axis.angle() * angle_axis.axis();

        for (int i = 0; i < 3; i++) {
          if (abs(rot_err(i)) < 5e-4) {
            rot_err(i) = 0;
          }
        }
        err.linear() = pos_err;   // 位置误差（线性）
        err.angular() = rot_err;
    
          double pos_err_cal = err.linear().norm();
          double rot_err_cal = err.angular().norm();
          std::cout << "err pos " << err.linear().transpose() << std::endl;
          std::cout << "rot pos " << err.angular().transpose() << std::endl;
          if (pos_err_cal < tolerance && rot_err_cal < tolerance) {
            target_q_ = temp_cal_q_;
            
            Eigen::Matrix3d R_final = current_pose.rotation();
            Eigen::AngleAxisd angle_axis(R_final);
            Eigen::Vector3d rot_final = angle_axis.angle() * angle_axis.axis();
            pinocchio::Motion final_motion;
            final_motion.linear() = current_pose.translation();
            final_motion.angular() = rot_final;
            std::cout << "final pos " << current_pose.translation().transpose() << std::endl;
            std::cout << "final rot " << final_motion.angular().transpose() << std::endl;
            for (int i = 0; i < target_q_.size(); i++) {
              std::cout << "target q " << target_q_(i) << std::endl;
            }
            
            return true;
          }
    
          // 计算雅可比矩阵
          Eigen::Matrix<double, 6, Eigen::Dynamic> J;
          J.setZero(6, model.nv);
          pinocchio::computeFrameJacobian(model, data, temp_cal_q_, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);

          // RCLCPP_INFO(this->get_logger(), "Frame Jacobian Matrix:");
          //   std::stringstream jac_ss;
          //   jac_ss << std::fixed << std::setprecision(4);
          //   jac_ss << "\n" << J.block<6,6>(0,0).matrix() << "\n"; // 只打印前6列（针对6自由度机械臂）
          //   RCLCPP_INFO(this->get_logger(), "%s", jac_ss.str().c_str());
    
          // 伪逆求解
          dq = J.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(err.toVector());
    
          // 更新关节角度
          temp_cal_q_ = pinocchio::integrate(model, temp_cal_q_, dq * 0.2);
    
          // 应用关节限位
          apply_joint_limits(temp_cal_q_);
        }
    
        return false;
      }
    
      // 应用关节限位
      void apply_joint_limits(Eigen::VectorXd& q) {
        for (int i = 0; i < model.njoints; ++i) {
          const auto& joint = model.joints[i];
          if (joint.idx_q() >= 0) {
            const int idx = joint.idx_q();
            q[idx] = std::clamp(q[idx], 
                               model.lowerPositionLimit[idx],
                               model.upperPositionLimit[idx]);
          }
        }
      }
    
      // 独立控制指令发布函数
  void publish_joint_commands(const Eigen::VectorXd& q) {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.header.frame_id = "command";
    
    // 按模型顺序填充数据
    for (const auto& joint_name : model.names) {
      if (model.getJointId(joint_name) > 0) {
        msg.name.push_back(joint_name);
        const auto idx = model.joints[model.getJointId(joint_name)].idx_q();
        std::cout << "idx out " << idx << std::endl;
        msg.position.push_back(q[idx]);
      }
    }
    for (int i = 0 ; i < 6; i++) {
      msg.effort.push_back(0.0);
    }
    msg.effort.push_back(10000.0);
    msg.effort.push_back(10000.0);

    joint_pub_->publish(msg);
  }
    
      // 备用运动生成（示例用）
      void timer_callback() {
        static double t = 0.0;
        t += 0.05;
        
        Eigen::VectorXd demo_q(model.nq);
        demo_q << sin(t * 1.0),
                  sin(t * 0.8) * 0.5,
                  sin(t * 0.6) * 0.35,
                  sin(t * 0.4) * 0.25,
                  sin(t * 0.2) * 0.15,
                  0.0;
        
        publish_joint_commands(demo_q);
      }
    
      // Pinocchio模型数据
      pinocchio::Model model;
      pinocchio::Data data;
      pinocchio::FrameIndex ee_frame_id_;
      Eigen::VectorXd current_q_;
      Eigen::VectorXd temp_cal_q_;
      Eigen::VectorXd target_q_;
      pinocchio::SE3 target_pose_;
    
      // ROS2接口
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
      rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr gripper_sub_;
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr actual_joint_sub_;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cur_pose_pub_;
      pinocchio::SE3 current_pose_;
      double gripper_7_;
      double gripper_8_;
      std::mutex q_mutex_;
      
      // 参数存储
      std::string base_frame_;
      std::string ee_frame_;
    };
    
    int main(int argc, char** argv) {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<ArmController>();
      rclcpp::spin(node);
      rclcpp::shutdown();
      return 0;
    }