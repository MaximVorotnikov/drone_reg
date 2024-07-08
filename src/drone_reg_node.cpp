#include "drone_reg_node.h"
#include <thread>
#include <boost/thread/recursive_mutex.hpp>
#include "euler_angles_lib/euler_angles.hpp"
#include <rclcpp/time.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std;

std::shared_ptr<rclcpp::Node> node_pointer;



/*----Функции подписки на топики----*/

// void cfg_callback(drone_reg::DroneRegConfig &config, uint32_t level) {
//     // dynamic_reconfigurate callback

//     hor_kp = config.hor_kp;                         //берем данные из .cfg файла
//     hor_kd = config.hor_kd;
//     max_hor_vel = config.max_hor_vel;

//     ver_kp = config.ver_kp;
//     ver_kd = config.ver_kd;
//     max_ver_vel = config.max_ver_vel;

//     angular_p = config.angular_p;
//     angular_d = config.angular_d;

//     if (init_server == true) {                      //сохраняем в .yaml файл
//         yaml_node["angular_d"] = angular_d;
//         yaml_node["angular_p"] = angular_p;
//         yaml_node["hor_kd"] = hor_kd;
//         yaml_node["hor_kp"] = hor_kp;
//         yaml_node["max_hor_vel"] = max_hor_vel;
//         yaml_node["max_ver_vel"] = max_ver_vel;
//         yaml_node["ver_kd"] = ver_kd;
//         yaml_node["ver_kp"] = ver_kp;

//         std::ofstream fout(yaml_path);
//         fout << yaml_node;
//     }

//     init_server = true;
// }

////получение параметров из ямла
// drone_reg::DroneRegConfig getYamlCfg()
// {
//     //загружаем файл
//     YAML::Node y_cfg = YAML::LoadFile(yaml_path);
//     //объявляем конфиг
//     drone_reg::DroneRegConfig cfg;
//     //заносим значения из ЯМЛА в переменные
//     hor_kp = y_cfg["hor_kp"].as<double>();
//     hor_kd = y_cfg["hor_kd"].as<double>();
//     max_hor_vel = y_cfg["max_hor_vel"].as<double>();

//     ver_kp = y_cfg["ver_kp"].as<double>();
//     ver_kd = y_cfg["ver_kd"].as<double>();
//     max_ver_vel = y_cfg["max_ver_vel"].as<double>();

//     angular_p = y_cfg["angular_p"].as<double>();
//     angular_d = y_cfg["angular_d"].as<double>();

//     //заносим значения в конфиг
//     cfg.hor_kp = hor_kp;
//     cfg.hor_kd = hor_kd;
//     cfg.max_hor_vel = max_hor_vel;

//     cfg.ver_kp = ver_kp;
//     cfg.ver_kd = ver_kd;
//     cfg.max_ver_vel = max_ver_vel;

//     cfg.angular_p = angular_p;
//     cfg.angular_d = angular_d;
//     //уазуращуаем уасся!
//     return cfg;
// }

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
  double dt = node_pointer->get_clock()->now().seconds() - last_time; // время в ros1
//   rclcpp::Time now = rclcpp::clock::now();        //мои мысли по поводу времени в ros2
//   double current_time_in_seconds = now.seconds();
//   double dt = rclcpp::Clock::now().seconds() - last_time;

  //current_vel.twist.linear.x = (drone_pose.point.x-last_pose.point.x) / dt;
  //current_vel.twist.linear.y = (drone_pose.point.y-last_pose.point.y) / dt;
  current_vel.twist.linear.z = (drone_pose.point.z-last_pose.point.z) / dt;

  last_time = node_pointer->get_clock()->now().seconds();  // переделать под ros2
  last_pose = drone_pose;
}

void nav_pos_cb(geometry_msgs::msg::PoseStamped::SharedPtr &data) {
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

    if(!use_alt_sonar)
      drone_pose.point.z = data->pose.position.z;

    pose_timer = 0.0;

    if(calc_vel_from_pos)
      calculate_velocity_from_pose();
}

void goal_cb(drone_msgs::msg::Goal::SharedPtr &data) {
    // goal_pose callback
    ctr_type = data->ctr_type;
    goal = data->pose;
    goal_timer = 0.0;
    goal_timer = 0.0;
}

void state_cb(mavros_msgs::msg::State::SharedPtr &data) {
    // state callback
    drone_status.armed = data->armed;
    drone_status.mode = data->mode;
}

void extended_state_cb(mavros_msgs::msg::ExtendedState::SharedPtr &data) {
    /***/
}

void imu_cb(sensor_msgs::msg::Imu::SharedPtr &data) {
    /***/
}

void on_shutdown_cb() {

}

void vel_field_cb(geometry_msgs::msg::TwistStamped::SharedPtr &data) {
    // velocity field callback
    vel_field.twist = data->twist;
}

void nav_vel_cb(geometry_msgs::msg::TwistStamped::SharedPtr &data) {
    // navigation velocity callback
  current_vel.twist.linear.x = data->twist.linear.x;
  current_vel.twist.linear.y = data->twist.linear.y;
  if(!calc_vel_from_pos)
	  current_vel.twist.linear.z = data->twist.linear.z;
}

void alt_sonar_cb(std_msgs::msg::Float32::SharedPtr &data)
{
  if(use_alt_sonar)
    drone_pose.point.z = data->data;
}

/*----Функции управления дроном----*/

std::vector<double_t> get_control(drone_msgs::msg::DronePose data) {
    //
    std::vector<double_t> coords_vec = {data.point.x - drone_pose.point.x,
                                        data.point.y - drone_pose.point.y,
                                        data.point.z - drone_pose.point.z};

    // double_t dr = sqrt(pow(data.point.x - drone_pose.point.x, 2) + pow(data.point.y - drone_pose.point.y, 2));
    // std::vector<double_t> coords_vec = {dr * cos(drone_pose.course),
    //                                     dr * sin(drone_pose.course),
    //                                     data.point.z - drone_pose.point.z};
    double_t diff_ang = data.course - drone_pose.course;
    RCLCPP_INFO(node_pointer->get_logger(), "\n\nDiff ang = '%d'", diff_ang);
    // cout << "\n\nDiff ang =" << diff_ang;
    // RCLCPP_INFO_STREAM("\n\nDiff ang = " << diff_ang);
    if (diff_ang >= M_PI)
        diff_ang -= 2 * M_PI;
    if (diff_ang < -M_PI)
        diff_ang += 2 * M_PI;
    RCLCPP_INFO(node_pointer->get_logger(), "Diff ang =  '%d'", diff_ang);
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
        RCLCPP_INFO(node_pointer->get_logger(), "arming");
        /***/
    }
}

void disarm() {
    if (drone_status.armed = true) {
        RCLCPP_INFO(node_pointer->get_logger(), "disarming");
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


double *dtp, *old_time_p;
std::vector<double_t> control;
geometry_msgs::msg::TwistStamped ctr_msg;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_vector_twist;



void loop()
{
    double d = node_pointer->get_clock()->now().seconds() - *old_time_p;
    dtp = &d;
    goal_timer += *dtp;
    pose_timer += *dtp;
    print_timer += *dtp;
    
    double ot = node_pointer->get_clock()->now().seconds();
    old_time_p = &ot;

    pub_marker->publish(setup_marker(goal));

    control = get_control(goal);

    ctr_msg.twist.linear.x = control[0] + vel_field.twist.linear.x;
    ctr_msg.twist.linear.y = control[1] + vel_field.twist.linear.y;
    ctr_msg.twist.linear.z = control[2] + vel_field.twist.linear.z;
    ctr_msg.twist.angular.z = control[3];

    if (pose_timer < pose_lost_time)
    {
        vel_pub->publish(ctr_msg);
        pub_vector_twist->publish(setup_vector_twist(ctr_msg));  
    }
    else {
        if (print_timer > print_delay) {
            //ROS_INFO("lost goal callback: %f %f %f %f", goal_timer, goal_timer, pose_timer, pose_timer);
            print_timer = 0;
        }
        }

    // rclcpp::spinOnce();
    // loop_rate.sleep();
}
///////////////////////////////////////
    // double old_time = node_pointer->get_clock()->now().seconds();
    // double dt = 0.0;
    // dtp = &dt;
    // old_time_p = &old_time;
///////////////////////////////////////
/*----Главная функция----*/
int main(int argc, char** argv) {
    local_pose_topic = mavros_root + local_pose_topic;
    goal.point.z = 1.0;

    rclcpp::init(argc, argv);
    // rclcpp::init(argc, argv, "drone_reg_vel_node");
    // rclcpp::Node n("drone_reg_node");
    node_pointer = std::make_shared<rclcpp::Node>("drone_reg_node");
    // rclcpp::NodeHandle n("~");



    rclcpp::Subscription<drone_msgs::msg::Goal::SharedPtr>::SharedPtr goalSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped::SharedPtr>::SharedPtr navPosSub; 
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped::SharedPtr>::SharedPtr navVelSub; 
    rclcpp::Subscription<mavros_msgs::msg::State::SharedPtr>::SharedPtr stateSub; 
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState::SharedPtr>::SharedPtr exStateSub; 
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped::SharedPtr>::SharedPtr velFieldSub; 
    rclcpp::Subscription<std_msgs::msg::Float32::SharedPtr>::SharedPtr altSonarSub; 

    //инициализация сервера динамической реконцигурации
    node_pointer->get_parameter_or("yaml_path", yaml_path); // Тотальный разбор лаунч файлов -> Параметры нодов C++
    node_pointer->get_parameter_or("use_planner", use_planner_flag);
    node_pointer->get_parameter_or("use_geo_mode", use_geo_mode);
    node_pointer->get_parameter_or("use_alt_sonar", use_alt_sonar);
    node_pointer->get_parameter_or("calc_vel_from_pos", calc_vel_from_pos);
    

    // boost::recursive_mutex config_mutex;
    // dynamic_reconfigure::Server<drone_reg::DroneRegConfig> server(config_mutex);
    // dynamic_reconfigure::Server<drone_reg::DroneRegConfig>::CallbackType f;
    // f = boost::bind(&cfg_callback, _1, _2);
    // //проверяем существование файла настроек
    // if (fileExists(yaml_path)) {
    //     //апдейтим значения на сервере из ямла
    //     drone_reg::DroneRegConfig yaml_cfg = getYamlCfg();
    //     server.updateConfig(yaml_cfg);
    // }
    // //получаем данные с сервера
    // server.setCallback(f);


    //stateSub = node_pointer->subscribe(mavros_root + "/state", queue_size, state_cb);
    //exStateSub = node_pointer->subscribe(mavros_root + "/extended_state", queue_size, extended_state_cb);
    velFieldSub = node_pointer->create_subscription<geometry_msgs::msg::TwistStamped::SharedPtr>("/field_vel", queue_size, vel_field_cb);
    navVelSub = node_pointer->create_subscription<geometry_msgs::msg::TwistStamped::SharedPtr>(mavros_root + "/local_position/velocity", queue_size, nav_vel_cb);

    if (use_geo_mode == true)
        goalSub = node_pointer->create_subscription<drone_msgs::msg::Goal::SharedPtr>(goal_global_topic, queue_size, goal_cb);
    else {
        if (use_planner_flag == true)
            goalSub = node_pointer->create_subscription<drone_msgs::msg::Goal::SharedPtr>(goal_planner_topic, queue_size, goal_cb);
        else
            goalSub = node_pointer->create_subscription<drone_msgs::msg::Goal::SharedPtr>(goal_local_topic, queue_size, goal_cb);
    }

    if (use_geo_mode == true)
        navPosSub = node_pointer->create_subscription<geometry_msgs::msg::PoseStamped::SharedPtr>(geo_pose_topic, queue_size, nav_pos_cb);
    else
        navPosSub = node_pointer->create_subscription<geometry_msgs::msg::PoseStamped::SharedPtr>(local_pose_topic, queue_size, nav_pos_cb);

    altSonarSub = node_pointer->create_subscription<std_msgs::msg::Float32::SharedPtr>(alt_sonar_topic, queue_size, alt_sonar_cb);

    vel_pub = node_pointer->create_publisher<geometry_msgs::msg::TwistStamped> (mavros_root + "/setpoint_velocity/cmd_vel", queue_size);
    pub_marker = node_pointer->create_publisher<visualization_msgs::msg::Marker> ("/marker_reg_point", queue_size);
    pub_vector_twist = node_pointer->create_publisher<sensor_msgs::msg::Imu> ("/reg_vector_twist", queue_size);


///////////////////////////////////////
    double old_time = node_pointer->get_clock()->now().seconds();
    double dt = 0.0;
    dtp = &dt;
    old_time_p = &old_time;
///////////////////////////////////////

    node_pointer->create_wall_timer(std::chrono::milliseconds(100), loop);
    while (rclcpp::ok()) {
        rclcpp::spin(node_pointer);
    }
    geometry_msgs::msg::TwistStamped ctr_msg_sd;
    ctr_msg_sd.twist.linear.x = 0;
    ctr_msg_sd.twist.linear.y = 0;
    ctr_msg_sd.twist.linear.z = 0;
    ctr_msg_sd.twist.angular.x = 0;
    ctr_msg_sd.twist.angular.y = 0;
    ctr_msg_sd.twist.angular.z = 0;
    vel_pub->publish(ctr_msg_sd);
    RCLCPP_INFO(node_pointer->get_logger(), "shutdown");

    rclcpp::shutdown();

    return 0;
}
