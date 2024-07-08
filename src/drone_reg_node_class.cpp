#include <thread>
#include <boost/thread/recursive_mutex.hpp>
#include "euler_angles_lib/euler_angles.hpp"
#include <rclcpp/time.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <drone_msgs/msg/drone_pose.hpp>
#include <drone_msgs/msg/goal.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std;





class DroneRegNode : public rclcpp::Node
{
public:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub;
  
  DroneRegNode()
  : Node("drone_reg_node")
  {
    drone_pose.course = 0;
    drone_pose.point.x = 0;
    drone_pose.point.y = 0;
    drone_pose.point.z = 0;
    goal.course = 0;
    goal.point.x = 0;
    goal.point.y = 0;
    goal.point.z = 0;

    local_pose_topic = mavros_root + local_pose_topic;
    velFieldSub = this->create_subscription<geometry_msgs::msg::TwistStamped>("/field_vel", queue_size, std::bind(&DroneRegNode::vel_field_cb, this, std::placeholders::_1));
    navVelSub = this->create_subscription<geometry_msgs::msg::TwistStamped>(mavros_root + "/local_position/velocity", rclcpp::QoS(rclcpp::KeepLast{10}).best_effort().durability_volatile(), std::bind(&DroneRegNode::nav_vel_cb, this, std::placeholders::_1));

    if (use_geo_mode == true)
        goalSub = this->create_subscription<drone_msgs::msg::Goal>(goal_global_topic, queue_size, std::bind(&DroneRegNode::goal_cb, this, std::placeholders::_1));
    else {
        if (use_planner_flag == true)
            goalSub = this->create_subscription<drone_msgs::msg::Goal>(goal_planner_topic, queue_size, std::bind(&DroneRegNode::goal_cb, this, std::placeholders::_1));
        else
            goalSub = this->create_subscription<drone_msgs::msg::Goal>(goal_local_topic, queue_size, std::bind(&DroneRegNode::goal_cb, this, std::placeholders::_1));
    }

    if (use_geo_mode == true)
        navPosSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(geo_pose_topic, queue_size, std::bind(&DroneRegNode::nav_pos_cb, this, std::placeholders::_1));
    else
        navPosSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(local_pose_topic, rclcpp::QoS(rclcpp::KeepLast{10}).best_effort(), std::bind(&DroneRegNode::nav_pos_cb, this, std::placeholders::_1));

    altSonarSub = this->create_subscription<std_msgs::msg::Float32>(alt_sonar_topic, queue_size, std::bind(&DroneRegNode::alt_sonar_cb, this, std::placeholders::_1));

    vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped> (mavros_root + "/setpoint_velocity/cmd_vel", rclcpp::QoS(rclcpp::KeepLast{10}).best_effort().durability_volatile());
    pub_marker = this->create_publisher<visualization_msgs::msg::Marker> ("/marker_reg_point", queue_size);
    pub_vector_twist = this->create_publisher<sensor_msgs::msg::Imu> ("/reg_vector_twist", queue_size);
    
    // auto timer_callback =
    //   [this]() -> void {
    //     auto message = std_msgs::msg::String();
    //     message.data = "Hello, world! " + std::to_string(this->count_++);
    //     RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //     this->publisher_->publish(message);
    //   };
      
    auto timer_callback = [this]() -> void 
    {
      double d = this->get_clock()->now().seconds() - *old_time_p;
      dtp = &d;
      goal_timer += *dtp;
      pose_timer += *dtp;
      print_timer += *dtp;
      
      double ot = this->get_clock()->now().seconds();
      old_time_p = &ot;

      pub_marker->publish(setup_marker(goal));

      control = get_control(goal);

      this->ctr_msg.twist.linear.x = control[0] + vel_field.twist.linear.x;
      this->ctr_msg.twist.linear.y = control[1] + vel_field.twist.linear.y;
      this->ctr_msg.twist.linear.z = control[2] + vel_field.twist.linear.z;
      this->ctr_msg.twist.angular.z = control[3];

      if (pose_timer < pose_lost_time)
      {
          this->vel_pub->publish(ctr_msg);
          this->pub_vector_twist->publish(setup_vector_twist(ctr_msg));  
      }
      else {
          if (print_timer > print_delay) {
              //ROS_INFO("lost goal callback: %f %f %f %f", goal_timer, goal_timer, pose_timer, pose_timer);
              print_timer = 0;
          }
        }
    };

    double old_time = this->get_clock()->now().seconds();
    double dt = 0.0;
    dtp = &dt;
    old_time_p = &old_time;
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
    rclcpp::Subscription<drone_msgs::msg::Goal>::SharedPtr goalSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr navPosSub; 
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr navVelSub; 
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr stateSub; 
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr exStateSub; 
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velFieldSub; 
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr altSonarSub; 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_vector_twist;

    double *dtp, *old_time_p;
    std::vector<double_t> control;
    geometry_msgs::msg::TwistStamped ctr_msg;
    
    rclcpp::TimerBase::SharedPtr timer_;

    /*----Global params----*/

    bool use_planner_flag = false;
    bool use_geo_mode = false;
    bool use_alt_sonar = true;
    bool calc_vel_from_pos = true;

    /*----Глобальные переменные----*/

    // Коэффициенты регулятора
    double_t hor_kp = 1.5;
    double_t hor_kd = 0.3;
    double_t max_hor_vel = 0.5;
    double_t ver_kp = 1.5;
    double_t ver_kd = 1.0;
    double_t max_ver_vel = 1.0;
    double_t angular_p = 2.5;
    double_t angular_d = 0.1;
    double_t prev_error = 0.0;

    int ctr_type;                                       // по умолчанию используем управление по координатам
    drone_msgs::msg::DronePose drone_pose;                   // координаты дрона
    drone_msgs::msg::DronePose goal;                         // целевая позиция дрона
    drone_msgs::msg::DronePose prev_goal;
    drone_msgs::msg::DronePose prev_vec;
    geometry_msgs::msg::TwistStamped current_vel;            // Текущая скорость дрона
    geometry_msgs::msg::TwistStamped vel_field;
    mavros_msgs::msg::State drone_status;

    // std::string mavros_root = "/mavros";
    std::string mavros_root = "/uav1/mavros";
    std::string OFFBOARD = "OFFBOARD";

    // topics
    std::string goal_local_topic = "/goal_pose";             // напрямую
    std::string goal_planner_topic = "/goal_pose_to_reg";    // для планировщика
    std::string goal_global_topic = "/geo/goal_pose";         // через geolib

    std::string local_pose_topic = "/local_position/pose";
    // std::string local_pose_topic = "/mavros/local_position/pose";
    std::string geo_pose_topic = "/geo/local_pose";
    std::string alt_sonar_topic = "/drone/alt";

    // yaml
    YAML::Node yaml_node;
    std::string yaml_path = "~/drone_reg_params.yaml";

    // таймеры
    double goal_timer = 0.0;
    double pose_timer = 0.0;
    double goal_lost_time = 1.0;
    double pose_lost_time = 0.5;
    double print_delay = 3.0;
    double print_timer = 0.;
    const int queue_size = 10;

    bool init_server = false;
/////////////////////////////////////////////////////
//   size_t count_;

      //функция проверки существования файла
  bool fileExists(const std::string& filename)
  {
      std::ifstream ifile(filename);
      return (bool) ifile;
  }

  void calculate_velocity_from_pose()
  {
    static double last_time;
    static drone_msgs::msg::DronePose last_pose;
    double dt = this->get_clock()->now().seconds() - last_time; // время в ros1

    current_vel.twist.linear.x = (drone_pose.point.x-last_pose.point.x) / dt;
    current_vel.twist.linear.y = (drone_pose.point.y-last_pose.point.y) / dt;
    current_vel.twist.linear.z = (drone_pose.point.z-last_pose.point.z) / dt;

    last_time = this->get_clock()->now().seconds();  // переделать под ros2
    last_pose = drone_pose;
  }

  void nav_pos_cb(geometry_msgs::msg::PoseStamped::SharedPtr data) {
      // navigation-position callback
      tf2::Quaternion quat(data->pose.orientation.x,
                          data->pose.orientation.y,
                          data->pose.orientation.z,
                          data->pose.orientation.w);
      tf2::Matrix3x3 m(quat);
      double_t roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);


      drone_pose.course = yaw;                        //getYaw
      drone_pose.point.x = data->pose.position.x;
      drone_pose.point.y = data->pose.position.y;

    //   if(!use_alt_sonar)
      drone_pose.point.z = data->pose.position.z;

      pose_timer = 0.0;

      if(calc_vel_from_pos)
        calculate_velocity_from_pose();
  }

  void goal_cb(drone_msgs::msg::Goal::SharedPtr data) {
      // goal_pose callback
      ctr_type = data->ctr_type;
      goal = data->pose;
      goal_timer = 0.0;
    //   goal_timer = 0.0;
  }

  void state_cb(mavros_msgs::msg::State::SharedPtr data) {
      // state callback
      drone_status.armed = data->armed;
      drone_status.mode = data->mode;
  }

  void extended_state_cb(mavros_msgs::msg::ExtendedState::SharedPtr data) {
      /***/
  }

  void imu_cb(sensor_msgs::msg::Imu::SharedPtr data) {
      /***/
  }

  void on_shutdown_cb() {

  }

  void vel_field_cb(geometry_msgs::msg::TwistStamped::SharedPtr data) {
      // velocity field callback
      vel_field.twist = data->twist;
  }

  void nav_vel_cb(geometry_msgs::msg::TwistStamped::SharedPtr data) {
      // navigation velocity callback
    current_vel.twist.linear.x = data->twist.linear.x;
    current_vel.twist.linear.y = data->twist.linear.y;
    if(!calc_vel_from_pos)
      current_vel.twist.linear.z = data->twist.linear.z;
  }

  void alt_sonar_cb(std_msgs::msg::Float32::SharedPtr data)
  {
    if(use_alt_sonar)
      drone_pose.point.z = data->data;
  }

  /*----Функции управления дроном----*/

  std::vector<double_t> get_control(drone_msgs::msg::DronePose goal_in) {
      // 
      RCLCPP_INFO_STREAM(this->get_logger(), "goal_in.point.x = " << goal_in.point.x);
      RCLCPP_INFO_STREAM(this->get_logger(), "goal_in.point.z = " << goal_in.point.z);
      RCLCPP_INFO_STREAM(this->get_logger(), "drone_pose.point.x = " << drone_pose.point.x);
      RCLCPP_INFO_STREAM(this->get_logger(), "drone_pose.point.z = " << drone_pose.point.z);
      std::vector<double_t> coords_vec = {goal_in.point.x - drone_pose.point.x,
                                          goal_in.point.y - drone_pose.point.y,
                                          goal_in.point.z - drone_pose.point.z};

      // double_t dr = sqrt(pow(goal_in.point.x - drone_pose.point.x, 2) + pow(goal_in.point.y - drone_pose.point.y, 2));
      // std::vector<double_t> coords_vec = {dr * cos(drone_pose.course),
      //                                     dr * sin(drone_pose.course),
      //                                     goal_in.point.z - drone_pose.point.z};
      double_t diff_ang = goal_in.course - drone_pose.course;
      RCLCPP_INFO_STREAM(this->get_logger(), "dx = " << coords_vec[0]);
      RCLCPP_INFO_STREAM(this->get_logger(), "dy = " << coords_vec[1]);
      RCLCPP_INFO_STREAM(this->get_logger(), "dz = " << coords_vec[2]);
      RCLCPP_INFO_STREAM(this->get_logger(), "\nDiff ang before = " << diff_ang);
      // cout << "\n\nDiff ang =" << diff_ang;
      // RCLCPP_INFO_STREAM("\n\nDiff ang = " << diff_ang);
      while (diff_ang >= M_PI or diff_ang < - M_PI)
      {
        if (diff_ang >= M_PI)
            diff_ang -= 2 * M_PI;
        if (diff_ang < -M_PI)
            diff_ang += 2 * M_PI;
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "\nDiff ang after = " << diff_ang);
      // ROS_INFO_STREAM("Diff ang = " << diff_ang);
      // cout << "Diff ang =" << diff_ang;

      std::vector<double_t> current_acc_vel = {current_vel.twist.linear.x,
                                              current_vel.twist.linear.y,
                                              current_vel.twist.linear.z};

      std::vector<double_t> vel_ctr_vec = get_linear_vel_vec(coords_vec, current_acc_vel);

      double_t ang = get_angular_vel(diff_ang, current_vel.twist.angular.z, angular_p, angular_d);
      // std::vector<double_t> res_ctr_vec = limit_vector(vel_ctr_vec, max_hor_vel);
      vel_ctr_vec.push_back(ang);
      return vel_ctr_vec;
  }

  void arm() {
      if (drone_status.armed != true) {
          RCLCPP_INFO(this->get_logger(), "arming");
          /***/
      }
  }

  void disarm() {
      if (drone_status.armed = true) {
          RCLCPP_INFO(this->get_logger(), "disarming");
          /***/
      }
  }

  void set_mode(string new_mode) {
      if (drone_status.mode != new_mode) {
          /***/
      }
  }

  /*----Вспомогательные функции----*/

  double_t angle_between(std::vector<double_t> vec1, std::vector<double_t> vec2) {
      // Возвращает угол между двумя двухмерными векторами

      double_t x = (vec2[1] - vec1[1]);
      double_t y = -(vec2[0] - vec1[0]);
      double_t res = atan2(x, y) + M_PI;

      if (res > M_PI)
          res -= 2 * M_PI;
      if (res < -M_PI)
          res += 2 * M_PI;

      return res;
  }

  std::vector<double_t> limit_vector(std::vector<double_t> r, double_t max_val) {
      // ограничивает управление (НЕ ИСПОЛЬЗУЕТСЯ)

      double_t l = norm_d(r);
      if (l > max_val) {
          r[0] = r[0] * max_val / l;
          r[1] = r[1] * max_val / l;
          r[2] = r[2] * max_val / l;
      }
      return r;
  }

  std::vector<double_t> get_linear_vel_vec(std::vector<double_t> r, std::vector<double_t> vel) {
      // возвращает вектор линейных скоростей
      // double_t dr = sqrt(pow(r[0], 2) + pow(r[1], 2));
      // double_t alpha = atan2(r[1], r[0]);

      double_t error = norm_d(r);
      std::vector<double_t> v = {0.0, 0.0, 0.0};
      v[0] = r[0] * hor_kp - vel[0] * hor_kd;
      v[1] = r[1] * hor_kp - vel[1] * hor_kd;
      v[2] = r[2] * ver_kp - vel[2] * ver_kd;

      for (int i = 0; i < (v.size() - 1); i++) {
          if (v[i] < -max_hor_vel)
              v[i] = -max_hor_vel;
          else if (v[i] > max_hor_vel)
              v[i] = max_hor_vel;
      }
      if (v[2] < -max_ver_vel)
          v[2] = -max_ver_vel;
      else if (v[2] > max_ver_vel)
          v[2] = max_ver_vel;

      prev_error = error;
      return v;
  }

  double_t get_angular_vel(double_t ang, double_t vel, double_t k, double_t d) {
      // возвращает угловую скорость

      if (ang == 0) {
          return 0;
      }

      if (ang > 0.2)
      {
          ang = 0.2;
      }
      if (ang < -0.2)
      {
          ang = -0.2;
      }

      if (ang >= M_PI)
          ang -= 2 * M_PI;
      if (ang < -M_PI)
          ang += 2 * M_PI;
      return k * ang - vel * d;
  }

  visualization_msgs::msg::Marker setup_marker(drone_msgs::msg::DronePose point) {
      // возвращает маркер в rviz

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
      marker.ns = "goal_test_reg";
      marker.id = 0;
      marker.action = 0;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.x = 1.0;

      marker.scale.x = 0.4;
      marker.scale.y = 0.4;
      marker.scale.z = 0.4;
      marker.type = visualization_msgs::msg::Marker::Type::SPHERE;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.pose.position.x = point.point.x;
      marker.pose.position.y = point.point.y;
      marker.pose.position.z = point.point.z;

      return marker;
  }

  sensor_msgs::msg::Imu setup_vector_twist(geometry_msgs::msg::TwistStamped vector_twist) {
      // отображает маркер скорости в rviz

      sensor_msgs::msg::Imu vector;
      vector.header.frame_id = "base_link";
      vector.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  
      vector.linear_acceleration.x = vector_twist.twist.linear.x;
      vector.linear_acceleration.y = vector_twist.twist.linear.y;
      vector.linear_acceleration.z = vector_twist.twist.linear.z;

      return vector;
  }

  void set_server_value() {
      /***/
  }

  drone_msgs::msg::DronePose lerp_point(std::vector<double_t> current_pos, drone_msgs::msg::DronePose newGoal, double_t step) {
      // интерполяция точки (НЕ ИСПОЛЬЗУЕТСЯ)

      std::vector<double_t> current_vec (3);
      current_vec[0] = newGoal.point.x - current_pos[0];
      current_vec[1] = newGoal.point.y - current_pos[1];
      current_vec[2] = newGoal.point.z - current_pos[2];

      prev_vec.point.x = prev_goal.point.x - current_pos[0];
      prev_vec.point.y = prev_goal.point.y - current_pos[1];
      prev_vec.point.z = prev_goal.point.z - current_pos[2];

      double_t dist = norm_d(current_vec);
      std::vector<double_t> prev_vec_temp = {prev_vec.point.x, prev_vec.point.y, prev_vec.point.z};
      double_t angle_between_goal = abs(angle_between(current_vec, prev_vec_temp));
      double_t dist_to_prev_point = norm_d(prev_vec_temp);

      if (dist < step) {
          prev_goal = newGoal;
          return newGoal;
      }
      if (((angle_between_goal > degToRad(30)) && (angle_between_goal < degToRad(180))) &&
              (dist_to_prev_point > (step / 2.0)) &&
              (dist > dist_to_prev_point))
          return prev_goal;
      std::vector<double_t> val(3);
      for (int i = 0; i < 3; i++)
          val[i] = current_vec[i] / dist * step;

      newGoal.point.x = val[0] + current_pos[0];
      newGoal.point.y = val[1] + current_pos[1];
      newGoal.point.z = val[2] + current_pos[2];

      prev_goal = newGoal;
      return newGoal;
  }

  /*----Python function replacements----*/

  double_t norm_d(std::vector<double_t> r) {
      // норма вектора
      return sqrt((r[0] * r[0]) + (r[1] * r[1]) + (r[2] * r[2]));
  }

  double_t degToRad(double_t deg) {
      // перевод из градусов в радианы
      return deg * M_PI / 180.0;
  }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneRegNode>();
    rclcpp::spin(node);
    geometry_msgs::msg::TwistStamped ctr_msg_sd;
    ctr_msg_sd.twist.linear.x = 0;
    ctr_msg_sd.twist.linear.y = 0;
    ctr_msg_sd.twist.linear.z = 0;
    ctr_msg_sd.twist.angular.x = 0;
    ctr_msg_sd.twist.angular.y = 0;
    ctr_msg_sd.twist.angular.z = 0;
    node->vel_pub->publish(ctr_msg_sd);
    rclcpp::shutdown();
  return 0;
}