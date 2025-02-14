/**
 * @file dynamic_forest_seq_sensing.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief generate a sequence of dynamic forest maps for fake sensing
 * To generate the current global map and local map in next time steps,
 * we need to publish current global point cloud and velocity
 *
 * The cylinder velocity is published as the difference between current position
 * and next position, which is (marker.points[1] - marker.points[0])
 *
 * @version 1.0
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

// for cylinders
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <map_generator/ObjectTpyeArray.h>
#include <std_msgs/Int32MultiArray.h>  // 使用 Int32MultiArray 消息类型
/*动态障碍物部分*/
// for dynamic obstacles
#include <Eigen/Eigen>
#include <random>

#include "moving_circle.h"
#include "moving_cylinder.h"
#include "moving_column.h"

#include "talker.h"
#define STATIC_OBJ 1
#define DYNAMIC_OBJ 2
#define UNKNOW_OBJ 3

using namespace std;
using namespace dynamic_map_objects;
enum class StaticMode {  //指定静态障碍物的生成模式
    RANDOM = 0,  //静态障碍物的位置是随机生成的
    FIXED = 1,   //静态障碍物的位置是固定的
};

/////////////////////////////////////////////////// start2.0 ///////////////////////////////////////////////////////////////////////////////////
extern  Vector2d dynamic_count;
extern int dynamic_number; //动态障碍物的数量
extern int static_number;//静态障碍物的数量
extern double matrix[][2]; //提取每个动态障碍物的坐标，暂存
extern double matrix_static_position[][2]; //提取每个静态障碍物的坐标，暂存
/////////////////////////////////////////////////// end  2.0 ///////////////////////////////////////////////////////////////////////////////////

vector<int>   pointIdxRadiusSearch;//半径搜索是一种常见的搜索方法，用于查找   给定点周围一定半径内的邻近点
vector<float> pointRadiusSquaredDistance;//用于存储在点云中进行半径搜索时，每个找到的点到查询点的 平方距离

/*生成随机数的一系列步骤，涉及到随机数生成器的初始化和配置*/
random_device                     rd; //非确定性随机数生成器，它提供一个随机的种子值
default_random_engine             eng(static_cast<unsigned>(rd()));//随机数生成器，它使用一个种子值来产生一系列伪随机数
/** @brief 模板类
 * 用于生成指定范围内均匀分布的随机浮点数。这里的 rand_x、rand_y、rand_w 和 rand_h 对象分别用于生成不同范围内的随机数：
 *   rand_x 和 rand_y 可能用于生成障碍物或对象在二维空间中的x和y坐标。
 *   rand_w 和 rand_h 可能用于生成障碍物或对象的宽度和高度。
*/
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;

/** @brief 话题发布
 * _all_map_cloud_pub：发布全局点云地图，全局点云地图通常包含了环境中所有静态和动态障碍物的信息。
 * _all_map_cylinder_pub：发布全局圆柱形障碍物的信息，每个圆柱形障碍物的位置和方向
 *_all_map_cylinder_pub_vis：发布圆柱形障碍物的可视化标记，在 RViz 或其他可视化工具中用来显示障碍物
 *_object_type_pub：发布障碍物类型的信息，用来发布每种障碍物的类型标识，以便其他节点或算法可以根据类型进行不同的处理
 *click_map_pub_：可能用于发布与点击地图相关的信息或事件。
 *_cylinder_state_pub：发布圆柱形障碍物的状态信息，可能包括障碍物的速度、加速度或其他动态特性
*/
// ros::Publisher _local_map_pub;
ros::Publisher _all_map_cloud_pub, _all_map_cylinder_pub, _all_map_cylinder_pub_vis;
ros::Publisher _object_type_pub;
ros::Publisher click_map_pub_, _cylinder_state_pub;
vector<double> _state;//状态信息

/** @brief 生成和模拟地图时的一些变量
 * _seed: 随机数生成器的种子值
 * _obs_num, _circle_num, _static_nums: 动态障碍物的数量，动态圆形障碍物的数量，静态障碍物的数量
 * _use_static_obs: 否在模拟中使用静态障碍物
 * _static_mode: 静态障碍物的生成模式
 * _fixed_pos: 存储固定位置的静态障碍物的坐标。每个 Eigen::Vector2d 包含一个障碍物的 x 和 y 坐标
 * _x_size, _y_size, _z_size: 定义环境地图的尺寸
 * _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _v_h, _a_h, _dr: 这些变量定义了障碍物的尺寸和物理参数的范围，x、y、宽度、高度、速度、加速度和障碍物的直径或半径
 * _static_w_l, _static_w_h, _static_h_l, _static_h_h: 定义静态障碍物的尺寸范围，宽度和高度的上下限
 * _measure_noise, _system_noise: 定义噪声参数，测量噪声和系统噪声
 * _radius_h, _radius_l, _z_l, _z_h, _theta, _omega_h: 定义圆形障碍物的参数，圆形障碍物半径的上限和下限、z 坐标的下限和上限、圆形障碍物的方向或角度、圆形障碍物的最大角速度
 * _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y: 定义感知和初始化参数，z 方向上的限制或界限、传感器的感知范围、地图的分辨率或障碍物的精度、感知更新的频率、初始位置的 x 和 y 坐标
*/
int         _seed;
int         _obs_num, _circle_num, _static_nums;//动态障碍物的数量，动态圆形障碍物的数量，静态障碍物的数量
bool        _use_static_obs;
StaticMode  _static_mode;
vector<Eigen::Vector2d>    _fixed_pos;
double      _x_size, _y_size, _z_size;
double      _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _v_h, _a_h, _dr;
double      _static_w_l, _static_w_h, _static_h_l, _static_h_h;
double      _measure_noise, _system_noise;
double      _radius_h, _radius_l, _z_l, _z_h, _theta, _omega_h;
double      _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;
std::string _frame_id;

/**
 * _map_ok：用来指示地图是否已经成功生成或加载
 * _has_odom：是否拥有里程计（odometry）信息
 * _set_cylinder:否设置或使用圆柱形障碍物
 * _test_mode：用于激活测试模式
*/
bool _map_ok       = false;
bool _has_odom     = false;
bool _set_cylinder = false;
bool _test_mode    = false;


/** @brief 设置地图模式
 * 它控制动态障碍物的速度随机化方式：
    _mode = 0：同时随机化水平方向的速度分量 vx 和垂直方向的速度分量 vy。
    _mode = 1：随机化 vx，而 vy 设为 0，即障碍物只在水平方向移动。
    _mode = 2：随机化 vy，而 vx 设为 0，即障碍物只在垂直方向移动。
 */
/** @brief map mode
 * 0: randomize both vx and vy
 * 1: randomize vx, vy = 0
 * 2: randomize vy, vx = 0
 */
int _mode = 0;

/* map sequence settings */
bool   _future_map = false;//用于指示是否启用未来地图序列的生成
int    _num_future_map;//用于设置未来地图序列的数量
double _future_step_size;//用于设置未来地图序列中每个时间步的大小

sensor_msgs::PointCloud2 globalMap_pcd;//存储和传输全局地图的点云数据。可以被填充到一个 PointCloud2 消息中，并发布到一个 ROS 主题上，供其他节点订阅和使用。
sensor_msgs::PointCloud2 globalCylinders_pcd;//存储和传输全局圆柱形障碍物的点云数据。同上

pcl::PointCloud<pcl::PointXYZ> clouds;//一个 PCL 点云对象，专门用于存储三维点云数据
geometry_msgs::PoseArray obstacle_array;//用于存储多个障碍物的位置和姿态信息。PoseArray 是一个数组，包含多个 
map_generator::ObjectTpyeArray object_type;//存储障碍物的类型信息

/** @brief 这些变量是 ROS中用于可视化的
 * cylinders_vis: 其中每个标记代表一个圆柱形对象的可视化信息
 * cylinder_mk: 用于定义单个圆柱形对象的可视化属性，如位置、大小、颜色和类型
 * columns_mk: 定义柱形对象（可能是立方体或长方体）的可视化属性
 * circle_vis: 其中每个标记代表一个圆形对象的可视化信息。这可以用于在 RViz 中显示一组圆形障碍物或标记。
 * circle_mk: 定义单个圆形对象的可视化属性
 * obstacle_state_list: 其中每个标记代表一个障碍物的状态信息，如位置、速度或方向。这些标记可以在 RViz 中显示，以提供关于障碍物动态状态的视觉反馈。
 * obstacle_state: 定义单个障碍物的状态信息
 */
visualization_msgs::MarkerArray cylinders_vis;
visualization_msgs::Marker      cylinder_mk;
visualization_msgs::Marker      columns_mk;
visualization_msgs::MarkerArray circle_vis;
visualization_msgs::Marker      circle_mk;
visualization_msgs::MarkerArray obstacle_state_list;
visualization_msgs::Marker      obstacle_state;

/** @brief 存储动态障碍物对象的实例
 * _dyn_cylinders:动态的圆柱形障碍物，它可能包含了位置、速度、加速度等动态属性
 * _dyn_circles:动态的圆形障碍物，它可能包含了圆心位置、半径、速度等属性
 * _dyn_columns:动态的柱形障碍物，它可能包含了柱体的位置、尺寸、速度等属性
 */
std::vector<dynamic_map_objects::MovingCylinder> _dyn_cylinders;
std::vector<dynamic_map_objects::MovingCircle>   _dyn_circles;
std::vector<dynamic_map_objects::MovingColumn>   _dyn_columns;   //这里*****************

ros::Time _last_time;

/**
 * @name RandomMapGenerate()
 * @brief generate random map 
 * 地图生成和障碍物初始化   
 */
void RandomMapGenerate(std::mt19937& generator, std::normal_distribution<double>& noise_sys, std::normal_distribution<double>& noise_mea) {

  /**
   * 1.初始化随机分布
   */
  pcl::PointXYZ       pt_random;
  geometry_msgs::Pose pt;
  pt.orientation.w = 1.0;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  /**
   * 2.清空并准备障碍物数组
   */
  object_type.data.clear();
  // generate pillar obstacles
  _dyn_columns.clear();
  /**
   * 3.生成柱形障碍物
   */
  _dyn_columns.reserve(_static_nums);
  for(int i = 0; i < _static_nums; i++){
    dynamic_map_objects::MovingColumn column(_x_l, _x_h, _y_l, _y_h, _static_w_l, _static_w_h, _static_h_l, _static_h_h,
                                             _v_h, _a_h, eng, _resolution, _use_static_obs);
    column.noiseInit(generator, noise_sys, noise_mea);
    if(_static_mode == StaticMode::FIXED){
      column.setPos(_fixed_pos[i]);
    }
    object_type.data.push_back(STATIC_OBJ);
    _dyn_columns.push_back(column);
  }

  /**
   * 4.生成圆柱形障碍物
   */
  _dyn_cylinders.clear();
  _dyn_cylinders.reserve(_obs_num);
  for (int i = 0; i < _obs_num; i++) {
    dynamic_map_objects::MovingCylinder cylinder(_x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h,
                                                 _v_h, _a_h, eng, _resolution);
    cylinder.noiseInit(generator, noise_sys, noise_mea);
    cylinder.setMode(_mode);
    _dyn_cylinders.push_back(cylinder);
    object_type.data.push_back(DYNAMIC_OBJ);
  }

  /**
   * 5.生成圆形障碍物
   */
  _dyn_circles.clear();
  _dyn_circles.reserve(_circle_num);
  for (int i = 0; i < _circle_num; i++) {
    dynamic_map_objects::MovingCircle circle(_x_l, _x_h, _y_l, _y_h, _z_l, _z_h, _radius_l,
                                             _radius_h, _dr, _v_h, _a_h, eng, _resolution);
    circle.noiseInit(generator, noise_sys, noise_mea);
    _dyn_circles.push_back(circle);
    object_type.data.push_back(DYNAMIC_OBJ);
  }

  /**
   * 6.更新最后生成时间
   */
  _last_time = ros::Time::now();

  /**
   * 7.输出日志并设置地图状态
   */
  ROS_WARN("Finished generate random map ");  

  _map_ok = true; //表示地图已成功生成
}

/**
 * @name pubSensedPoints()
 * @brief 将感知到的点云数据和障碍物状态发布到 ROS 系统中
 * 这个函数处理动态障碍物的更新，并将它们的信息转换为 ROS 消息，然后发布到不同的主题上
 */
void pubSensedPoints() 
{
  
  /**
   * 1.初始化和清空容器
   * 
    清空 clouds 点云对象的点数据，为新的点数据预留空间。
    清空 cylinders_vis 和 obstacle_state_list 标记数组，为新的标记数据预留空间。
    设置 cylinder_mk 的时间戳和 ID。
   */
  // concatenate all points
  clouds.points.clear();
  clouds.points.reserve(_obs_num);
  cylinders_vis.markers.clear();
  cylinders_vis.markers.reserve(_obs_num);
  obstacle_state_list.markers.clear();
  obstacle_state_list.markers.reserve(_obs_num);
  cylinder_mk.header.stamp = ros::Time::now();
  cylinder_mk.id           = 0;

  obstacle_array.poses.clear();

  obstacle_state.header.stamp = ros::Time::now();
  obstacle_state.points.clear();
  obstacle_state.id = 0;

  /**
   * 2.更新当前时间并计算时间差
   * 
    获取当前时间 cur_time 并计算与上次发布时间 _last_time 的时间差 delta_time。
    更新 _last_time 为当前时间。

    double delta_time = (cur_time - _last_time).toSec();
        cur_time：这是一个 ros::Time 类型的对象，表示当前的时间戳。
        _last_time：这也是一个 ros::Time 类型的对象，表示上次更新时的时间戳。
        cur_time - _last_time：这个操作会返回一个 ros::Duration 类型的对象，它表示两个时间点之间的时间间隔。
        .toSec()：这是 ros::Duration 类的一个成员函数，它将时间间隔转换为秒。

   */
  pcl::PointCloud<pcl::PointXYZ> cloud_all;
  
  ros::Time cur_time = ros::Time::now();
  double delta_time = (cur_time - _last_time).toSec();//计算两次更新之间的时间差，单位是秒
  _last_time = cur_time;

  /**
   * 3.处理柱形障碍物
   * 
    遍历 _dyn_columns 中的所有柱形障碍物，更新它们的状态。
    将更新后的柱形障碍物添加到 cloud_all 点云中。
    为每个柱形障碍物创建 geometry_msgs::Pose 和 visualization_msgs::Marker 对象，并将它们添加到 obstacle_array 和 cylinders_vis 中。
   */
  for (auto& dyn_cld : _dyn_columns) {
    /**
     * 更新障碍物状态        
     * 如果当前模式不是测试模式（_test_mode 为 false），则调用每个障碍物的 update 方法，传入时间差 delta_time 来更新障碍物的状态，包括位置和速度  
     */
    if (!_test_mode) {
      dyn_cld.update(delta_time);
    }

    /**
     * 合并点云数据
     */
    cloud_all += dyn_cld._cloud;
    /**
     * 设置障碍物的位置
     */
    geometry_msgs::Pose obs_pose;
    obs_pose.position.x = dyn_cld.x;
    obs_pose.position.y = dyn_cld.y;
    obs_pose.position.z = 0.2 * dyn_cld.h;
    
    /**
     * 计算障碍物的方向
     */
    double yaw = std::atan2(dyn_cld._set_vy, dyn_cld._set_vx);
    /**
     * 设置障碍物的姿态
     */
    obs_pose.orientation.x = 0.0; // 无俯仰角和翻滚角
    obs_pose.orientation.y = 0.0;
    if(_use_static_obs){
      obs_pose.orientation.z = 0.0;
      obs_pose.orientation.w = 1.0;
    }else{
      obs_pose.orientation.z = std::sin(yaw / 2.0);
      obs_pose.orientation.w = std::cos(yaw / 2.0);
    }
    /**
     * 添加障碍物到数组
     */
    obstacle_array.poses.push_back(obs_pose);
    /**
     * 设置可视化标记
     */
    geometry_msgs::Pose pose;
    pose.position.x    = dyn_cld.x;
    pose.position.y    = dyn_cld.y;
    pose.position.z    = 0.5 * dyn_cld.h;
    pose.orientation.w = 1.0;

    columns_mk.pose = pose;
    columns_mk.scale.x = columns_mk.scale.y = dyn_cld.w + _resolution;  // less then 1
    columns_mk.scale.z                       = dyn_cld.h + _resolution;
    cylinders_vis.markers.push_back(columns_mk);
    columns_mk.id += 1;
    /**
     * 设置障碍物状态标记
     */
    // obstacle_state.pose               = pose;
    obstacle_state.pose.position.x    = dyn_cld.x;
    obstacle_state.pose.position.y    = dyn_cld.y;
    obstacle_state.pose.position.z    = 0.5 * dyn_cld.h;
    obstacle_state.pose.orientation.w = 1.0;
    obstacle_state.points.clear();
    geometry_msgs::Point pts;
    pts.x = pose.position.x;
    pts.y = pose.position.y;
    pts.z = pose.position.z;
    obstacle_state.points.push_back(pts);
    pts.x += dyn_cld.vx * _sense_rate;
    pts.y += dyn_cld.vy * _sense_rate;
    obstacle_state.points.push_back(pts);
    obstacle_state.scale.x = dyn_cld.w;
    obstacle_state.scale.y = dyn_cld.w;
    obstacle_state.type    = visualization_msgs::Marker::ARROW;
    obstacle_state_list.markers.push_back(obstacle_state);
    obstacle_state.id += 1;
  }

  /**
   * 4.处理圆柱形障碍物     这个动态障碍物
   * 
    遍历 _dyn_cylinders 中的所有圆柱形障碍物，更新它们的状态。
    将更新后的圆柱形障碍物添加到 cloud_all 点云中。
    为每个圆柱形障碍物创建 geometry_msgs::Pose 和 visualization_msgs::Marker 对象，并将它们添加到 obstacle_array 和 cylinders_vis 中。
   */
  // for (auto& dyn_cld : _dyn_cylinders) {
  //   if (!_test_mode) {
  //     dyn_cld.update(delta_time);
  //   }

  ///////////////////////////// start 1 ///////////////////////////////////////
  for (size_t i = 0; i < _dyn_cylinders.size(); ++i) {
        auto& dyn_cld = _dyn_cylinders[i];
        if (!_test_mode) {
          /**
           * 根据障碍物的编号选择相应的updatei函数
           * i：对应障碍物的编号
           */
            switch (i+1) 
            {
              case 1:dynamic_count=dyn_cld.update1(delta_time);break;
              case 2:dynamic_count=dyn_cld.update2(delta_time);break;
              case 3:dynamic_count=dyn_cld.update3(delta_time);break;
              case 4:dynamic_count=dyn_cld.update4(delta_time);break;
              case 5:dynamic_count=dyn_cld.update5(delta_time);break;
              case 6:dynamic_count=dyn_cld.update6(delta_time);break;
              default:break;
            }

            // dynamic_count=dyn_cld.update(delta_time);
            // dyn_cld.update(delta_time);
        }
        dynamic_number=_dyn_cylinders.size();
        // ROS_INFO("动态障碍物的个数: %d", dynamic_number);
  ///////////////////////////// end   1 ///////////////////////////////////////
    // ROS_INFO("动态障碍额1的坐标: %f", matrix[0][0]);
    // ROS_INFO("动态障碍额1的坐标: %f", matrix[0][1]);
    // ROS_INFO("动态障碍额2的坐标: %f", matrix[1][0]);
    // ROS_INFO("动态障碍额2的坐标: %f", matrix[1][1]);
    cloud_all += dyn_cld._cloud;

    // publish cylinder markerss
    geometry_msgs::Pose obs_pose;
    obs_pose.position.x = dyn_cld.x;
    obs_pose.position.y = dyn_cld.y;
    // ROS_INFO("点云坐标x: %f", dynamic_count.x());
    // ROS_INFO("点云坐标y: %f", dynamic_count.y());
    matrix[i][0]=dynamic_count.x();
    matrix[i][1]=dynamic_count.y();

    /////////////////////////////////////////////////////////////
    // all_x=obs_pose.position.x;
    // all_y=obs_pose.position.y;
    /////////////////////////////////////////////////////////////

    // ROS_INFO("11坐标x: %f", dyn_cld.x);
    // ROS_INFO("11坐标y: %f", dyn_cld.y);
    obs_pose.position.z = 0.2 * dyn_cld.h;
    double yaw = std::atan2(dyn_cld._set_vy, dyn_cld._set_vx);
    obs_pose.orientation.x = 0.0; // 无俯仰角和翻滚角
    obs_pose.orientation.y = 0.0;
    obs_pose.orientation.z = std::sin(yaw / 2.0);
    obs_pose.orientation.w = std::cos(yaw / 2.0);
    obstacle_array.poses.push_back(obs_pose);

    geometry_msgs::Pose pose;
    pose.position.x    = dyn_cld.x;
    pose.position.y    = dyn_cld.y;
    pose.position.z    = 0.5 * dyn_cld.h;
    pose.orientation.w = 1.0;

    cylinder_mk.pose    = pose;
    cylinder_mk.scale.x = cylinder_mk.scale.y = dyn_cld.w + _resolution;  // less then 1
    cylinder_mk.scale.z                       = dyn_cld.h + _resolution;
    cylinders_vis.markers.push_back(cylinder_mk);
    cylinder_mk.id += 1;

    obstacle_state.pose               = pose;
    obstacle_state.pose.position.x    = dyn_cld.x;
    obstacle_state.pose.position.y    = dyn_cld.y;
    obstacle_state.pose.position.z    = 0.5 * dyn_cld.h;
    obstacle_state.pose.orientation.w = 1.0;
    obstacle_state.points.clear();
    geometry_msgs::Point pts;
    pts.x = pose.position.x;
    pts.y = pose.position.y;
    pts.z = pose.position.z;
    obstacle_state.points.push_back(pts);
    pts.x += dyn_cld.vx * _sense_rate;
    pts.y += dyn_cld.vy * _sense_rate;
    obstacle_state.points.push_back(pts);
    obstacle_state.scale.x = dyn_cld.w;
    obstacle_state.scale.y = dyn_cld.w;
    obstacle_state.type    = visualization_msgs::Marker::ARROW;
    obstacle_state_list.markers.push_back(obstacle_state);
    obstacle_state.id += 1;
  }

  /**
   * 5.处理圆形障碍物    不是
   * 
    遍历 _dyn_circles 中的所有圆形障碍物，更新它们的状态。
    将更新后的圆形障碍物添加到 cloud_all 点云中。
    为每个圆形障碍物创建 geometry_msgs::Pose 和 visualization_msgs::Marker 对象，并将它们添加到 obstacle_array 和 cylinders_vis 中。

   */
  for (auto& dyn_crl : _dyn_circles) {
    if (!_test_mode) 
      dyn_crl.update(delta_time);

    cloud_all += dyn_crl._cloud;
    geometry_msgs::Pose obs_pose;
    obs_pose.position.x = dyn_crl.x;
    obs_pose.position.y = dyn_crl.y;
    obs_pose.position.z = dyn_crl.z;

    double yaw = std::atan2(dyn_crl._set_vy, dyn_crl._set_vx);
    obs_pose.orientation.x = 0.0; // 无俯仰角和翻滚角
    obs_pose.orientation.y = 0.0;
    obs_pose.orientation.z = std::sin(yaw / 2.0);
    obs_pose.orientation.w = std::cos(yaw / 2.0);
    obstacle_array.poses.push_back(obs_pose);

    geometry_msgs::Pose pose;
    pose.position.x    = dyn_crl.x;
    pose.position.y    = dyn_crl.y;
    pose.position.z    = dyn_crl.z;
    pose.orientation.w = dyn_crl.q.w();
    pose.orientation.x = dyn_crl.q.x();
    pose.orientation.y = dyn_crl.q.y();
    pose.orientation.z = dyn_crl.q.z();

    cylinder_mk.pose    = pose;
    cylinder_mk.scale.x = 2 * dyn_crl.r2;
    cylinder_mk.scale.y = 2 * dyn_crl.r1;
    cylinder_mk.scale.z = 0.1;
    cylinders_vis.markers.push_back(cylinder_mk);
    cylinder_mk.id += 1;

    obstacle_state.pose.position.x    = dyn_crl.x;
    obstacle_state.pose.position.y    = dyn_crl.y;
    obstacle_state.pose.position.z    = dyn_crl.z;
    obstacle_state.pose.orientation.w = dyn_crl.q.w();
    obstacle_state.pose.orientation.x = dyn_crl.q.x();
    obstacle_state.pose.orientation.y = dyn_crl.q.y();
    obstacle_state.pose.orientation.z = dyn_crl.q.z();

    obstacle_state.points.clear();
    geometry_msgs::Point pts;
    pts.x = pose.position.x;
    pts.y = pose.position.y;
    pts.z = pose.position.z;
    obstacle_state.points.push_back(pts);
    pts.x += dyn_crl.vx * _sense_rate;
    pts.y += dyn_crl.vy * _sense_rate;
    obstacle_state.points.push_back(pts);
    obstacle_state.scale.x = 2 * dyn_crl.r2;
    obstacle_state.scale.y = 2 * dyn_crl.r1;
    obstacle_state.scale.z = 0.1;
    obstacle_state.type    = visualization_msgs::Marker::ARROW;
    obstacle_state_list.markers.push_back(obstacle_state);
    obstacle_state.id += 1;
  }

///////////////////////////////////////////// start /////////////////////////////////////////////////////////

///////////////////////////////////////////// end   /////////////////////////////////////////////////////////


  /**
   * 6.发布点云数据
   * 将 cloud_all 点云转换为 ROS 消息 globalMap_pcd 并发布到 _all_map_cloud_pub 主题。
   */
  cloud_all.width    = cloud_all.points.size();
  cloud_all.height   = 1;
  cloud_all.is_dense = true;

  //ROS_WARN_STREAM("Publishing " << cloud_all.points.size() << " points");
  // ROS_WARN_STREAM("Publishing " << cylinders.points.size() << " cylinders");
  // ROS_WARN_STREAM("Publishing " << cylinders_vis.markers.size() << " cylinders markers");

  // publish cloud
  pcl::toROSMsg(cloud_all, globalMap_pcd);
  globalMap_pcd.header.frame_id = _frame_id;
  _all_map_cloud_pub.publish(globalMap_pcd);

  // publish cylinder markers
  // pcl::toROSMsg(clouds, globalCylinders_pcd);
  // globalCylinders_pcd.header.frame_id = _frame_id;
  // globalCylinders_pcd.header.stamp = ros::Time::now();

  /**
   * 7.发布障碍物状态
   * 
    发布 obstacle_array 到 _all_map_cylinder_pub 主题。
    发布 cylinders_vis 到 _all_map_cylinder_pub_vis 主题。
    发布 obstacle_state_list 到 _cylinder_state_pub 主题。
   */
  obstacle_array.header.frame_id = _frame_id;
  obstacle_array.header.stamp = ros::Time::now();
  _all_map_cylinder_pub.publish(obstacle_array);

  _all_map_cylinder_pub_vis.publish(cylinders_vis);
  _cylinder_state_pub.publish(obstacle_state_list);

  /**
   * 8.发布障碍物类型信息
   * 发布 object_type 到 _object_type_pub 主题。
   */
  object_type.header.frame_id = _frame_id;
  object_type.header.stamp = ros::Time::now();
  _object_type_pub.publish(object_type);
  return;
}
///////////////////////
/**
 * 初始化节点，设置参数，生成地图，并在循环中更新和发布感知到的点云数据
 */
int main(int argc, char** argv) {

  /**
   * 1.初始化 ROS 节点
   */
  ros::init(argc, argv, "dynamic_map_sequence_sensing");
  ros::NodeHandle n("~");

  /**
   * 2.创建发布者
   * 
    _all_map_cloud_pub 发布全局点云地图。
    _all_map_cylinder_pub 发布全局圆柱形障碍物的位置信息。
    _object_type_pub 发布障碍物类型信息。
    _all_map_cylinder_pub_vis 发布圆柱形障碍物的可视化标记。
    _cylinder_state_pub 发布圆柱形障碍物的状态信息。
   */
  //_local_map_pub = n.advertise<sensor_msgs::PointCloud2>("local_cloud", 1);
  _all_map_cloud_pub    = n.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
  _all_map_cylinder_pub = n.advertise<geometry_msgs::PoseArray>("global_cylinders", 1);
  _object_type_pub      = n.advertise<map_generator::ObjectTpyeArray>("object_type_flag", 1);
  _all_map_cylinder_pub_vis =n.advertise<visualization_msgs::MarkerArray>("global_cylinders_vis", 1);
  _cylinder_state_pub = n.advertise<visualization_msgs::MarkerArray>("global_cylinder_state", 1);

  /**
   * 3.从参数服务器加载参数
   * 使用 n.param 方法从参数服务器加载配置参数，这些参数包括初始状态、地图参数、障碍物参数等。
   */
  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);
  n.param("map/seed", _seed, 0);
  n.param("map/future", _future_map, true);
  n.param("map/future_num", _num_future_map, 6);
  n.param("map/time_step", _future_step_size, 0.2);
  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/test", _test_mode, false);

  /**
   * 4.调整地图尺寸
   * 为多机器人环境调整地图尺寸，减少 _x_size 和 _y_size 的值。
   */
  // clearance for multi robots.
  _x_size -= 2.0;
  _y_size -= 2.0;

  /**
   * 5.设置障碍物参数
   * 设置障碍物的形状、速度、加速度等参数
   */
  n.param("map/obs_num", _obs_num, 30);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/frame_id", _frame_id, string("map"));

  n.param("ObstacleShape/lower_rad", _w_l, 0.3);
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);
  n.param("ObstacleShape/lower_hei", _h_l, 3.0);
  n.param("ObstacleShape/upper_hei", _h_h, 7.0);
  n.param("ObstacleShape/upper_vel", _v_h, 1.0);
  n.param("ObstacleShape/upper_acc", _a_h, 1.0);
  n.param("ObstacleShape/system_noise", _system_noise, 0.05);
  n.param("ObstacleShape/measure_noise", _measure_noise, 0.05);
  n.param("ObstacleShape/set_cylinder", _set_cylinder, false);
  n.param("map/circle_num", _circle_num, 0);
  n.param("ObstacleShape/radius_l", _radius_l, 7.0);
  n.param("ObstacleShape/radius_h", _radius_h, 7.0);
  n.param("ObstacleShape/z_l", _z_l, 7.0);
  n.param("ObstacleShape/z_h", _z_h, 7.0);
  n.param("ObstacleShape/dr", _dr, 0.2);
  n.param("ObstacleShape/theta", _theta, 7.0);
  n.param("ObstacleShape/omega", _omega_h, 2.0);

  
  n.param("StaticObstacle/use", _use_static_obs, true);
  n.param("StaticObstacle/static_nums", _static_nums, 5);
  string staticmode = "";
  static_number=_static_nums;
  n.param("StaticObstacle/static_mode", staticmode, string("random"));
  if(staticmode == "random"){
    _static_mode = StaticMode::RANDOM;
  }else if(staticmode == "fixed"){
    _static_mode = StaticMode::FIXED;
  }else{
    ROS_WARN("_static_mode if wrong, please set the parameters correctly!");
    _static_mode = StaticMode::FIXED;
  }
  _fixed_pos.resize(_static_nums);


  /**
   * 静态障碍物坐标读取      重要
   */
  for(int i = 1; i <= _static_nums; i++){
    n.param(string("StaticObstacle/obs_") + to_string(i) + "_x", _fixed_pos[i - 1](0), 0.0);
    n.param(string("StaticObstacle/obs_") + to_string(i) + "_y", _fixed_pos[i - 1](1), 0.0);


    ///////////////////////////////////////    start  ///////////////////////////////////////////////////   
    matrix_static_position[i-1][0]= _fixed_pos[i - 1](0);//读取静态障碍物坐标
    matrix_static_position[i-1][1]= _fixed_pos[i - 1](1);
    
    ROS_INFO("ObstacleShape/obs_%d_x:%f", i,matrix_static_position[i-1][0]);
    ROS_INFO("ObstacleShape/obs_%d_y:%f", i,matrix_static_position[i-1][1]);

    ///////////////////////////////////////    end    ///////////////////////////////////////////////////   

  }

    ///////////////////////////////////////    start  ///////////////////////////////////////////////////   

  for (int i = 0; i < _static_nums; i++)
  {
      ROS_INFO("axcccc %d: (%f, %f)", i + 1, matrix_static_position[i][0], matrix_static_position[i][1]);
  }
    ///////////////////////////////////////    end    ///////////////////////////////////////////////////   


  n.param(string("StaticObstacle/lower_height"), _static_h_l, 0.5);
  n.param(string("StaticObstacle/upper_height"), _static_h_h, 2.5);
  n.param(string("StaticObstacle/lower_width"), _static_w_l, 0.5);
  n.param(string("StaticObstacle/upper_width"), _static_w_h, 2.5);

  n.param("sensing/radius", _sensing_range, 10.0);
  n.param("sensing/rate", _sense_rate, 10.0);
  n.param("mode", _mode, 0);


  /**
   * 8.设置随机种子
   */
  /* random seeds */
  // eng.seed(_seed);

  /**
   * 9.设置坐标范围
   *  根据地图尺寸设置 x 和 y 坐标的范围。
   */
  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  /**
   * 10.设置标记属性
   *  设置用于可视化的标记的属性，如类型、动作、ID、颜色等
   */
  columns_mk.header.frame_id = _frame_id;
  columns_mk.type            = visualization_msgs::Marker::CUBE;
  columns_mk.action          = visualization_msgs::Marker::ADD;
  columns_mk.id              = 0;
  columns_mk.color.r         = 0.5;
  columns_mk.color.g         = 0.5;
  columns_mk.color.b         = 0.5;
  columns_mk.color.a         = 0.02;

  cylinder_mk.header.frame_id = _frame_id;
  cylinder_mk.type            = visualization_msgs::Marker::CYLINDER;
  cylinder_mk.action          = visualization_msgs::Marker::ADD;
  cylinder_mk.id              = 0;
  cylinder_mk.color.r         = 0.5;
  cylinder_mk.color.g         = 0.5;
  cylinder_mk.color.b         = 0.5;
  cylinder_mk.color.a         = 0.4;

  obstacle_state.header = cylinder_mk.header;
  obstacle_state.type   = visualization_msgs::Marker::ARROW;
  obstacle_state.action = visualization_msgs::Marker::ADD;
  obstacle_state.id              = 0;
  obstacle_state.color.r         = 1.0;
  obstacle_state.color.g         = 0.0;
  obstacle_state.color.b         = 0.0;
  obstacle_state.color.a         = 0.6;

////////////////////////////test start////////////////////////////////////////////////////////////////
    int abc;
    if (n.param("ObstacleShape/abc", abc, 5)) // 默认值为5
    {
        ROS_INFO("ObstacleShape/abc is set to %d", abc);
    }
    else
    {
         abc = 6; // 设置默认值
         ROS_INFO("ObstacleShape/abc is set to %d", abc);

        
    }

      ROS_INFO("ObstacleShape/abc is set to %d", abc);
////////////////////////////test end  ////////////////////////////////////////////////////////////////

  /**
   * 11.生成随机数和噪声分布
   * 
    使用 std::random_device 和 std::mt19937 生成随机数。
    创建正态分布对象，用于模拟系统噪声和测量噪声。
   */
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> sys_noise(0.0, _system_noise / 2.0);
  std::normal_distribution<double> mea_noise(0.0, _measure_noise / 2.0);

  ros::Duration(0.5).sleep();

  /**
   * 12.生成地图
   * 调用 RandomMapGenerate 函数生成随机地图
   */
  RandomMapGenerate(gen, sys_noise, mea_noise);

  /**
   * 13.设置循环频率
   *  使用 ros::Rate 设置循环频率，等于感知频率 _sense_rate
   */
  ros::Rate loop_rate(_sense_rate);
  setlocale(LC_ALL, ""); // 使用系统默认编码
  /**
   * 14.主循化
   * 在 while (ros::ok()) 循环中，节点持续运行：
      调用 pubSensedPoints 函数更新和发布感知到的点云数据。
      调用 ros::spinOnce 处理回调函数。
      调用 loop_rate.sleep 保持循环频率。
   */
  // MovingCylinder sta;//声明类

  while (ros::ok()) {
    // delete old cylinders in rviz
    // visualization_msgs::Marker delete_cylinders;
    // delete_cylinders.header.frame_id = _frame_id;
    // delete_cylinders.action          = visualization_msgs::Marker::DELETEALL;
    // cylinders_vis.markers.clear();
    // cylinders_vis.markers.push_back(delete_cylinders);
    // _all_map_cylinder_pub_vis.publish(cylinders_vis);

    // update map
    pubSensedPoints();


    ros::spinOnce();
    loop_rate.sleep();
  }
}
