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
// #include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32.hpp>

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
std::string mavros_root = "/uav1";
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

/*----Функции подписки на топики----*/

// void cfg_callback(drone_reg::DroneRegConfig &config, uint32_t level);

void nav_pos_cb(geometry_msgs::msg::PoseStamped::SharedPtr &data);

void goal_cb(drone_msgs::msg::Goal::SharedPtr &data);

void state_cb(mavros_msgs::msg::State::SharedPtr &data);

void extended_state_cb(mavros_msgs::msg::ExtendedState::SharedPtr &data);

void imu_cb(sensor_msgs::msg::Imu::SharedPtr &data);

void on_shutdown_cb();

void vel_field_cb(geometry_msgs::msg::TwistStamped::SharedPtr &data);

void nav_vel_cb(geometry_msgs::msg::TwistStamped::SharedPtr &data);

void alt_sonar_cb(std_msgs::msg::Float32::SharedPtr &data);

/*----Функции управления дроном----*/

std::vector<double_t> get_control(drone_msgs::msg::DronePose data);

void arm();

void disarm();

void set_mode(std::string new_mode);

/*----Вспомогательные функции----*/

double_t angle_between(std::vector<double_t> vec1, std::vector<double_t> vec2);

std::vector<double_t> limit_vector(std::vector<double_t> r, double_t max_val);

std::vector<double_t> get_linear_vel_vec(std::vector<double_t> r, std::vector<double_t> vel);

double_t get_angular_vel(double_t ang, double_t vel, double_t k, double_t d);

visualization_msgs::msg::Marker setup_marker(drone_msgs::msg::DronePose point);

void set_server_value();

drone_msgs::msg::DronePose lerp_point(std::vector<double_t> current_pos, drone_msgs::msg::DronePose newGoal, double_t step);

/*----Python function replacements----*/

double_t norm_d(std::vector<double_t> r);

double_t degToRad(double_t rad);

/*-------------------------------------*/
