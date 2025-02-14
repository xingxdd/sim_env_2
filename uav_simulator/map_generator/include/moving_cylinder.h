/**
 * @file moving_cylinder.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-07-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __MOVING_CYLINDER_H__
#define __MOVING_CYLINDER_H__

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <random>

///////////////////////////////////////////////////// start 1.0/////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
using namespace std;
using namespace Eigen;
Vector2d dynamic_count={0.0,0.0};//单个动态障碍物的坐标信息，暂存量
int dynamic_number=0; //动态障碍物的数量
int static_number=0;//静态障碍物的数量
double matrix[6][2]={0.0};//各动态障碍物的坐标存储
double matrix_static_position[6][2]={0};//多静态障碍物的位置坐标
///////////////////////////////////////////////////// end 1.0/////////////////////////////////////////////////////////////////////////////////////


namespace dynamic_map_objects {

class MovingCylinder {
 private:
  std::uniform_real_distribution<double> _rand_x, _rand_y, _rand_w, _rand_h, _rand_v, _rand_a;
  std::default_random_engine _generator;
  double                                 _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _v_h, _a_h;
  double                                 _resolution;
  int                                    _mode = 0;

  std::mt19937 _gen;
  std::normal_distribution<double> _vel_error;
  std::normal_distribution<double> _pos_error;
  
  double _de_acc;

 public:
  pcl::PointCloud<pcl::PointXYZ> _cloud;
  double                         x;   // x coordinate of the cylinder center
  double                         y;   // y coordinate of the cylinder center
  double                         last_x;
  double                         last_y;
  double                         w;   // width
  double                         h;   // height
  double                         vx;  // x velocity
  double                         vy;  // y velocity
  double                         _init_vx, _init_vy;
  double                         _set_vx, _set_vy;





  ///////////////////////////////////////////////////// start 2.0 /////////////////////////////////////////////////////////////////////////////////////
    int a=0; //进入update函数的次数计数
    int count_position=1;//目标点标志位
  // 势场参数
    double attraction_coefficient = 1.0;  // 吸引力系数
    double repulsion_coefficient = 100.0;  // 斥力系数
    double repulsion_threshold = 2.0;  // 斥力作用范围

    // 路径规划参数
    double step_size = 0.09;  // 步长
    int max_iterations = 1000;  // 最大迭代次数
    Vector2d start_position_last;

  ////////////////////////////////////////////////////// end 2.0////////////////////////////////////////////////////////////////////////////////////



  MovingCylinder(double                      x_l,
                 double                      x_h,
                 double                      y_l,
                 double                      y_h,
                 double                      w_l,
                 double                      w_h,
                 double                      h_l,
                 double                      h_h,
                 double                      v_h,
                 double                      a_h,
                 std::default_random_engine &eng,
                 double                      _resolution);
  ~MovingCylinder() {}
  void setMode(int m);
  Vector2d update(double delta);
  void noiseInit(std::mt19937 gen, std::normal_distribution<double> vel_error, std::normal_distribution<double> pos_error);
//////////////////////////////////////////////// start 3.0/////////////////////////////////////////////////////////////////////////////////////////////////////////
  Vector2d update1(double delta);
  Vector2d update2(double delta);
  Vector2d update3(double delta);
  Vector2d update4(double delta);
  Vector2d update5(double delta);
  Vector2d update6(double delta);
  
  vector<Vector2d> obs_static_position_exchange(double matrix_static_position[6][2]); //静态坐标变换

  // 计算吸引力
  Vector2d calculate_attraction(const Vector2d& position, const Vector2d& goal, double attraction_coefficient) ;
  //计算斥力

  Vector2d calculate_repulsion(const Vector2d& position, const Vector2d& goal,const vector<Vector2d>& obstacles, double repulsion_coefficient, double repulsion_threshold);

//////////////////////////////////////////////// end 3.0  /////////////////////////////////////////////////////////////////////////////////////////////////////////



};

MovingCylinder::MovingCylinder(double                      x_l,
                               double                      x_h,
                               double                      y_l,
                               double                      y_h,
                               double                      w_l,
                               double                      w_h,
                               double                      h_l,
                               double                      h_h,
                               double                      v_h,
                               double                      a_h,
                              //  double                      sys_amp,
                              //  double                      mea_amp,
                               std::default_random_engine &eng,
                               double                      resolution)
    : _x_l(x_l)
    , _x_h(x_h)
    , _y_l(y_l)
    , _y_h(y_h)
    , _w_l(w_l)
    , _w_h(w_h)
    , _h_l(h_l)
    , _h_h(h_h)
    , _v_h(v_h)
    , _a_h(a_h)
    , _generator(eng){
  _rand_x     = std::uniform_real_distribution<double>(x_l, x_h);
  _rand_y     = std::uniform_real_distribution<double>(y_l, y_h);
  _rand_w     = std::uniform_real_distribution<double>(w_l, w_h);
  _rand_h     = std::uniform_real_distribution<double>(h_l, h_h);
  _rand_v     = std::uniform_real_distribution<double>(-v_h, v_h);
  _rand_a     = std::uniform_real_distribution<double>(-a_h, a_h);
  _resolution = resolution;

  _de_acc = 0.5;

  // genrate random 2D position, width, height
  x = _rand_x(eng);
  y = _rand_y(eng);
  // x = -4.0;
  // y = -0.5;
  h = _rand_h(eng);
  w = _rand_w(eng);

  // generate random veocity
  // _set_vx = _rand_v(eng);
  // _set_vx = _rand_v(eng);

  _set_vx = random() % 2 == 0 ? _v_h : -_v_h;
  _set_vy = random() % 2 == 0 ? _v_h : -_v_h;
  
  _init_vx = _set_vx > 0 ? _set_vx : - _set_vx;
  _init_vy = _set_vy > 0 ? _set_vy : - _set_vy;

  ROS_WARN("speed value: %f", _set_vx);

  int heightNum = ceil(h / _resolution);
  int widNum    = ceil(w / _resolution);

  // random point in grid scale
  x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  y = floor(y / _resolution) * _resolution + _resolution / 2.0;
  last_x = x;
  last_y = y;
  // generate point cloud
  _cloud.points.resize(0);
  _cloud.width  = 0;
  _cloud.height = 0;

  pcl::PointXYZ pt(x, y, h);
  _cloud.points.push_back(pt);

  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
      if ((r * r + s * s) > (widNum * widNum / 4.0)) {
        continue;
      }
      for (int t = -2.0; t < heightNum; t++) {
        pt.x = x + (r + 0.5) * _resolution + 1e-2;
        pt.y = y + (s + 0.5) * _resolution + 1e-2;
        pt.z = (t + 0.5) * _resolution + 1e-2;
        _cloud.points.push_back(pt);
      }
    }

  _cloud.width    = _cloud.points.size();
  _cloud.height   = 1;
  _cloud.is_dense = true;
};  // namespace dynamic_map_objects

void MovingCylinder::noiseInit(std::mt19937 gen, std::normal_distribution<double> vel_error, std::normal_distribution<double> pos_error){
  _gen = gen;
  _vel_error = vel_error;
  _pos_error = pos_error;
}

void MovingCylinder::setMode(int m) {
  // generate random veocity
  if (m == 0) {
    ;
  } else if (m == 1) {
    _set_vy = 0;
  } else if (m == 2) {
    _set_vx = 0;
  } else {
    _set_vx = 0;
    _set_vy = 0;
  }
}


//////////////////////////////////////////////////start 4.0/////////////////////////////////////////////////////////////////////////////////////
  
  vector<Vector2d> MovingCylinder::obs_static_position_exchange(double matrix_static_position[6][2]) //静态坐标变换
  {
      std::vector<Vector2d> obstacles;
        for (int i = 1; i <= static_number; i++)
        {
            Vector2d obstacle(matrix_static_position[i - 1][0], matrix_static_position[i - 1][1]);
            obstacles.push_back(obstacle);

            // 打印每个静态障碍物的坐标
            ROS_INFO("Static Obstacle %d: (%f, %f)", i, obstacle(0), obstacle(1));
        }
        
        return obstacles;
  }

  // vector<Vector2d> MovingCylinder::obs_static_position_exchange(double matrix_static_position[6][2]) //静态坐标变换
  // {
  //   vector<Vector2d> obstacles;
  //     for(int i=1;i<=static_number;i++)
  //     {
  //       obstacles.push_back(Vector2d(matrix_static_position[i-1][0],matrix_static_position[i-1][1]));
  //     }
  //     //打印观看内容
  //      for (size_t i = 0; i < obstacles.size(); ++i) {
  //       std::cout << "(" << obstacles[i](0) << ", " << obstacles[i](1) << ")" << std::endl;
  //       ROS_INFO("静态障碍物的坐标信息x: %f", obstacles[i](0));  // 输出当前x坐标
  //       ROS_INFO("静态障碍物的坐标信息y: %f", obstacles[i](1));  // 输出当前y坐标
  //   }

  //   return obstacles;
  // }
// 计算吸引力
Vector2d MovingCylinder::calculate_attraction(const Vector2d& position, const Vector2d& goal, double attraction_coefficient) {
    double distance = (position - goal).norm();  // 计算当前位置到目标位置的欧几里得距离
    Vector2d attraction = attraction_coefficient * (goal - position) / distance;  // 根据人工势场法公式计算吸引力向量
    return attraction;  // 返回计算得到的吸引力向量
}

// 计算斥力   初始的简洁版斥力函数
Vector2d MovingCylinder::calculate_repulsion(const Vector2d& position, const Vector2d& goal,const vector<Vector2d>& obstacles, double repulsion_coefficient, double repulsion_threshold) {
    Vector2d repulsion_force = Vector2d::Zero();
    Vector2d rep1 = Vector2d::Zero();
    Vector2d rep2 = Vector2d::Zero();
    int q=0;
    for (const auto& obstacle : obstacles) {
        double distance = (position - obstacle).norm();  // 计算当前位置到障碍物的欧几里得距离
        double distance_p_g = (position - goal).norm();  // 计算当前位置到目标位置的欧几里得距离
        if((position - goal).norm() <= 0.5)//如果物体到达目标点附近，则障碍物斥力为0
        {
             q=0;//标志位系数
        }
        else q=1;

        if (distance < repulsion_threshold) {  // 如果距离小于斥力作用范围
            Vector2d obstacle_vec = position - obstacle;
            repulsion_force += repulsion_coefficient * (1 / repulsion_threshold - 1 / distance) * (obstacle_vec / pow(distance, 3))*q;  // 计算当前位置到障碍物的向量
            
        }
    }
    return repulsion_force;  // 返回计算得到的斥力向量
}

// // 计算斥力  优化后的斥力函数
// Vector2d MovingCylinder::calculate_repulsion(const Vector2d& position, const Vector2d& goal,const vector<Vector2d>& obstacles, double repulsion_coefficient, double repulsion_threshold) {
//     Vector2d repulsion_force = Vector2d::Zero();
    
//     int q=0;
//     for (const auto& obstacle : obstacles) {
//         double distance = (position - obstacle).norm();  // 计算当前位置到障碍物的欧几里得距离
//         double distance_p_g = (position - goal).norm();  // 计算当前位置到目标位置的欧几里得距离
//         if((position - goal).norm() <= 0.5)//如果物体到达目标点附近，则障碍物斥力为0
//         {
//              q=0;//标志位系数
//         }
//         else q=1;

//         if (distance < repulsion_threshold) {  // 如果距离小于斥力作用范围
//             Vector2d obstacle_vec = position - obstacle;
//             Vector2d rep1 = repulsion_coefficient * (1 / repulsion_threshold - 1 / distance) * (obstacle_vec / pow(distance, 3))*q*distance_p_g;  // 计算当前位置到障碍物的向量
//             Vector2d rep2 = 1/2*repulsion_coefficient * pow((1 / repulsion_threshold - 1 / distance),2)* (obstacle_vec / distance)*q;  // 计算当前位置到障碍物的向量
//             repulsion_force+=rep1+rep2;
//         }
//     }
//     return repulsion_force;  // 返回计算得到的斥力向量
// }


Vector2d MovingCylinder::update(double delta) {
   
    a=a+1;//进入函数的次数
    // ROS_INFO("测试a: %d", a);  // 输出当前x坐标

    /**
     * 目标点更迭 
     */
    Vector2d goal_position(0.0,0.0);
    switch (count_position)
    {
      case 1:goal_position.x()=-12;goal_position.y()=-15;  // 目标位置 
          break;
      case 2:goal_position.x()=12;goal_position.y()=-15;  // 目标位置 
          break;
      case 3:goal_position.x()=-12;goal_position.y()=15;  // 目标位置 
          break;
      case 4:goal_position.x()=12;goal_position.y()=-15;  // 目标位置 
          break;
      default: count_position=1; goal_position.x()=-12;goal_position.y()=-15;  // 目标位置 
          break;
    }

    Vector2d start_position(10.0, 10.0);  // 初始位置
    // Vector2d goal_position(-12.0, -15.0);  // 目标位置
    vector<Vector2d> obstacles = {Vector2d(-8.0, 0.0), Vector2d(0.0, 0.0), Vector2d(7.0, -3.0),Vector2d(-5.0, 10.0), Vector2d(-8.0, -9.5)};  // 障碍物位置

    
    vector<Vector2d> path;  // 路径记录
    Vector2d position = start_position;  // 初始化机器人当前位置为起始位置
 
    // 路径规划
    if(a>1) {
      position.x()=start_position_last[0];
      position.y()=start_position_last[1];
    }

    path.push_back(start_position);  // 添加初始位置到路径记录
  
    // 计算吸引力：从当前位置到目标位置的吸引力
    Vector2d attraction = calculate_attraction(position, goal_position, attraction_coefficient);

    // 计算斥力：从当前位置到所有障碍物的斥力
    // Vector2d repulsion = calculate_repulsion(position, obstacles, repulsion_coefficient, repulsion_threshold);
    Vector2d repulsion =calculate_repulsion(position, goal_position, obstacles, repulsion_coefficient, repulsion_threshold) ;

    // 合成总力：吸引力减去斥力
    Vector2d force = attraction - repulsion;

    // 根据合力更新位置：步长乘以归一化后的合力
    position += step_size * force.normalized();

     start_position_last[0]=position.x();
     start_position_last[1]=position.y();
    //  ROS_INFO("position坐标x: %f", position.x());  // 输出当前x坐标
    //  ROS_INFO("position坐标y: %f", position.y());  // 输出当前y坐标

    //  ROS_INFO("start_position_last坐标x: %f", start_position_last.x());  // 输出当前x坐标
    //  ROS_INFO("start_position_last坐标y: %f", start_position_last.y());  // 输出当前y坐标
      
    if ((position - goal_position).norm() <= 0.5) 
    {
      ROS_INFO("抵达目标点%d",count_position);
      count_position+=1;
    }
    
    // 计算点云的变换矩阵，用于将点云从上一位置变换到当前位置
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << start_position_last[0] -last_x, start_position_last[1] - last_y, 0;
    
    // 对点云进行变换
    pcl::transformPointCloud(_cloud, _cloud, transform);

    // // 更新上一位置信息
    last_x = start_position_last[0];
    last_y = start_position_last[1];
    

    // ros::Time cur_time = ros::Time::now();  // 获取当前时间，该行代码被注释掉了
    ROS_INFO("坐标x: %f", x);  // 输出当前x坐标
    ROS_INFO("坐标y: %f", y);  // 输出当前y坐标
    ROS_INFO("delta: %f", delta);  // 
    ROS_INFO("前往目标点%d ing",count_position);
    ROS_INFO("-----------------------------------------------------");

    return start_position_last;
  }
/**
 * 1号动态障碍物的更新函数
 */
Vector2d MovingCylinder::update1(double delta) {
   
    a=a+1;//进入函数的次数
    // ROS_INFO("测试a: %d", a);  // 输出当前x坐标

    /**
     * 目标点更迭 
     */
    Vector2d goal_position(0.0,0.0);
    switch (count_position)
    {
      case 1:goal_position.x()=-12;goal_position.y()=-15;  // 目标位置 
          break;
      case 2:goal_position.x()=3;goal_position.y()=-5;  // 目标位置 
          break;
      case 3:goal_position.x()=-2;goal_position.y()=15;  // 目标位置 
          break;
      case 4:goal_position.x()=12;goal_position.y()=-9;  // 目标位置 
          break;
      default: count_position=1; goal_position.x()=-12;goal_position.y()=-15;  // 目标位置 
          break;
    }

    Vector2d start_position(-10.0, 10.0);  // 初始位置
    // vector<Vector2d> obstacles;
    /**
     * 判断动态障碍物的个数，设置合理个数的障碍物坐标
     * obs_static_positong_exchange
     */
////////////////////////////////////////////////start_xin/////////////////////////////////////////////////////////////////
    std::vector<Vector2d> obstacles ;
    std::vector<Vector2d> obstacles1 ;
    for (int i = 1; i <= static_number; i++)
    {
        Vector2d obstacle(matrix_static_position[i - 1][0], matrix_static_position[i - 1][1]);
        obstacles1.push_back(obstacle);

        // 打印每个静态障碍物的坐标
        // ROS_INFO("xin_Static Obstacle %d: (%f, %f)", i, obstacle(0), obstacle(1));
    }
//////////////////////////////////////////////// end_xin /////////////////////////////////////////////////////////////////

    switch (dynamic_number)
    {
      case 1:
        obstacles = {};// 除动态障碍物本身外，还有0个动态障碍物,不执行
        obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      case 2:
        obstacles = {
          Vector2d(matrix[1][0],matrix[1][1])};  // 除动态障碍物本身外，还有1个动态障碍物
         obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      case 3:
        obstacles = {
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[2][0],matrix[2][1])};  // 除动态障碍物本身外，还有3个动态障碍物
         obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
       case 4:
        obstacles = {
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[2][0],matrix[2][1]),
          Vector2d(matrix[3][0],matrix[3][1])
          };  // 除动态障碍物本身外，还有3个动态障碍物
       obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      case 5:
        obstacles = {
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[2][0],matrix[2][1]),
          Vector2d(matrix[3][0],matrix[3][1]),
          Vector2d(matrix[4][0],matrix[4][1])
          };  // 除动态障碍物本身外，还有4个动态障碍物
        obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      case 6:
        obstacles = {
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[2][0],matrix[2][1]),
          Vector2d(matrix[3][0],matrix[3][1]),
          Vector2d(matrix[4][0],matrix[4][1]),
          Vector2d(matrix[5][0],matrix[5][1])
          };  // 除动态障碍物本身外，还有4个动态障碍物
        obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      default:break;
    }
  
    vector<Vector2d> path;  // 路径记录
    Vector2d position = start_position;  // 初始化机器人当前位置为起始位置
 
    // 路径规划
    if(a>1) {
      position.x()=start_position_last[0];
      position.y()=start_position_last[1];
    }
    // 根据合力更新位置：步长乘以归一化后的合力

    path.push_back(start_position);  // 添加初始位置到路径记录
  
    // 计算吸引力：从当前位置到目标位置的吸引力
    Vector2d attraction = calculate_attraction(position, goal_position, attraction_coefficient);

    // 计算斥力：从当前位置到所有障碍物的斥力
    Vector2d repulsion=calculate_repulsion(position, goal_position, obstacles, repulsion_coefficient, repulsion_threshold) ;  

    // 合成总力：吸引力减去斥力
    Vector2d force = attraction - repulsion;

    // 根据合力更新位置：步长乘以归一化后的合力
  
    /**
     * 生成随机数
     */
    srand(time(NULL));
    double ax = ((double)rand() / RAND_MAX * 0.03 - 1.0)*0.01;
    double ay = ((double)rand() / RAND_MAX * 0.03 - 1.0)*0.01;
    Vector2d perturbation(ax, ay);
    // printf("随机数%f",ax);
    // printf("随机数%f\n",ay);

    position += step_size * (force.normalized()+Vector2d(ax, ay));

     start_position_last[0]=position.x();
     start_position_last[1]=position.y();
    //  ROS_INFO("position坐标x: %f", position.x());  // 输出当前x坐标
    //  ROS_INFO("position坐标y: %f", position.y());  // 输出当前y坐标

    //  ROS_INFO("start_position_last坐标x: %f", start_position_last.x());  // 输出当前x坐标
    //  ROS_INFO("start_position_last坐标y: %f", start_position_last.y());  // 输出当前y坐标
      
    if ((position - goal_position).norm() <= 0.5) 
    {
      ROS_INFO("抵达目标点%d",count_position);
      count_position+=1;
    }
    

    // 计算点云的变换矩阵，用于将点云从上一位置变换到当前位置
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << start_position_last[0] -last_x, start_position_last[1] - last_y, 0;

    //////////////////////箭头/////////////////////////////////////////////////////
    _set_vx=start_position_last[0] -last_x;
    _set_vy=start_position_last[1] -last_y;
    //////////////////////箭头/////////////////////////////////////////////////////


    // 对点云进行变换
    pcl::transformPointCloud(_cloud, _cloud, transform);

    // // 更新上一位置信息
    last_x = start_position_last[0];
    last_y = start_position_last[1];
  

    // ros::Time cur_time = ros::Time::now();  // 获取当前时间，该行代码被注释掉了
    // ROS_INFO("坐标x: %f", x);  // 输出当前x坐标
    // ROS_INFO("坐标y: %f", y);  // 输出当前y坐标
    // ROS_INFO("delta: %f", delta);  // 
    // ROS_INFO("前往目标点%d ing",count_position);
    // ROS_INFO("-----------------------------------------------------");

    return start_position_last;
  }

  /**
   * 2号动态障碍物的更新函数
   */
  Vector2d MovingCylinder::update2(double delta) {
   
    a=a+1;//进入函数的次数
    // ROS_INFO("测试a: %d", a);  // 输出当前x坐标

    /**
     * 目标点更迭 
     */
    Vector2d goal_position(10.0,0.0);
    switch (count_position)
    {
      case 1:goal_position.x()=-12;goal_position.y()=-15;  // 目标位置 
          break;
      case 2:goal_position.x()=-3;goal_position.y()=-15;  // 目标位置 
          break;
      case 3:goal_position.x()=-3;goal_position.y()=10;  // 目标位置 
          break;
      case 4:goal_position.x()=4;goal_position.y()=6;  // 目标位置 
          break;
      default: count_position=1; goal_position.x()=-12;goal_position.y()=-15;  // 目标位置 
          break;
    }

    Vector2d start_position(8.0, 10.0);  // 初始位置


////////////////////////////////////////////////start_xin/////////////////////////////////////////////////////////////////
    std::vector<Vector2d> obstacles ;
    std::vector<Vector2d> obstacles1 ;
    for (int i = 1; i <= static_number; i++)
    {
        Vector2d obstacle(matrix_static_position[i - 1][0], matrix_static_position[i - 1][1]);
        obstacles1.push_back(obstacle);

        // 打印每个静态障碍物的坐标
        // ROS_INFO("xin_Static Obstacle %d: (%f, %f)", i, obstacle(0), obstacle(1));
    }
//////////////////////////////////////////////// end_xin /////////////////////////////////////////////////////////////////



    /**
     * 判断动态障碍物的个数，设置合理个数的障碍物坐标
     */
    switch (dynamic_number)
    {
      case 1:
        obstacles = {};// 除动态障碍物本身外，还有0个动态障碍物,不执行
        obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      case 2:
        obstacles = {
        Vector2d(matrix[0][0],matrix[0][1])};  // 除动态障碍物本身外，还有1个动态障碍物
         obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      case 3:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[2][0],matrix[2][1])       
          };  // 除动态障碍物本身外，还有2个动态障碍物
           obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      case 4:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[2][0],matrix[2][1]),
          Vector2d(matrix[3][0],matrix[3][1])
          };  // 除动态障碍物本身外，还有3个动态障碍物
           obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      case 5:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[2][0],matrix[2][1]),
          Vector2d(matrix[3][0],matrix[3][1]),
          Vector2d(matrix[4][0],matrix[4][1])
          };  // 除动态障碍物本身外，还有4个动态障碍物
           obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      case 6:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[2][0],matrix[2][1]),
          Vector2d(matrix[3][0],matrix[3][1]),
          Vector2d(matrix[4][0],matrix[4][1]),
          Vector2d(matrix[5][0],matrix[5][1])
          };  // 除动态障碍物本身外，还有4个动态障碍物
           obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      default:break;
    }
   
    // ROS_INFO("障碍物的数量%d", obstacles.size());

    vector<Vector2d> path;  // 路径记录
    Vector2d position = start_position;  // 初始化机器人当前位置为起始位置
 
    // 路径规划
    if(a>1) {
      position.x()=start_position_last[0];
      position.y()=start_position_last[1];
    }

    path.push_back(start_position);  // 添加初始位置到路径记录
  
    // 计算吸引力：从当前位置到目标位置的吸引力
    Vector2d attraction = calculate_attraction(position, goal_position, attraction_coefficient);

    // 计算斥力：从当前位置到所有障碍物的斥力
    // Vector2d repulsion = calculate_repulsion(position, obstacles, repulsion_coefficient, repulsion_threshold);
    Vector2d repulsion=calculate_repulsion(position, goal_position, obstacles, repulsion_coefficient, repulsion_threshold) ;
    // 合成总力：吸引力减去斥力
    Vector2d force = attraction - repulsion;

    // 根据合力更新位置：步长乘以归一化后的合力
  
    /**
     * 生成随机数
     */
    srand(time(NULL));
    double ax = ((double)rand() / RAND_MAX * 0.03 - 1.0)*0.01;
    double ay = ((double)rand() / RAND_MAX * 0.03 - 1.0)*0.01;
    Vector2d perturbation(ax, ay);
    // printf("随机数%f",ax);
    // printf("随机数%f\n",ay);

    position += step_size * (force.normalized()+Vector2d(ax, ay));

     start_position_last[0]=position.x();
     start_position_last[1]=position.y();
    //  ROS_INFO("position坐标x: %f", position.x());  // 输出当前x坐标
    //  ROS_INFO("position坐标y: %f", position.y());  // 输出当前y坐标

    //  ROS_INFO("start_position_last坐标x: %f", start_position_last.x());  // 输出当前x坐标
    //  ROS_INFO("start_position_last坐标y: %f", start_position_last.y());  // 输出当前y坐标
      
    if ((position - goal_position).norm() <= 0.5) 
    {
      ROS_INFO("抵达目标点%d",count_position);
      count_position+=1;
    }
    
    // 计算点云的变换矩阵，用于将点云从上一位置变换到当前位置
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << start_position_last[0] -last_x, start_position_last[1] - last_y, 0;
    
      //////////////////////箭头/////////////////////////////////////////////////////
    _set_vx=start_position_last[0] -last_x;
    _set_vy=start_position_last[1] -last_y;
    //////////////////////箭头/////////////////////////////////////////////////////


    // 对点云进行变换
    pcl::transformPointCloud(_cloud, _cloud, transform);

    // // 更新上一位置信息
    last_x = start_position_last[0];
    last_y = start_position_last[1];
    

    // ros::Time cur_time = ros::Time::now();  // 获取当前时间，该行代码被注释掉了
    // ROS_INFO("坐标x: %f", x);  // 输出当前x坐标
    // ROS_INFO("坐标y: %f", y);  // 输出当前y坐标
    // ROS_INFO("delta: %f", delta);  // 
    // ROS_INFO("前往目标点%d ing",count_position);
    // ROS_INFO("-----------------------------------------------------");

    return start_position_last;
  }
   /**
   * 3号动态障碍物的更新函数
   */
  Vector2d MovingCylinder::update3(double delta) {
   
    a=a+1;//进入函数的次数
    // ROS_INFO("测试a: %d", a);  // 输出当前x坐标

    /**
     * 目标点更迭 
     */
    Vector2d goal_position(0.0,10.0);
    switch (count_position)
    {
      case 1:goal_position.x()=-1;goal_position.y()=-15;  // 目标位置 
          break;
      case 2:goal_position.x()=7;goal_position.y()=3;  // 目标位置 
          break;
      case 3:goal_position.x()=-3;goal_position.y()=5;  // 目标位置 
          break;
      case 4:goal_position.x()=6;goal_position.y()=-7;  // 目标位置 
          break;
      default: count_position=1; goal_position.x()=-12;goal_position.y()=-15;  // 目标位置 
          break;
    }

    Vector2d start_position(10.0, 10.0);  // 初始位置
    ////////////////////////////////////////////////start_xin/////////////////////////////////////////////////////////////////
    std::vector<Vector2d> obstacles ;
    std::vector<Vector2d> obstacles1 ;
    for (int i = 1; i <= static_number; i++)
    {
        Vector2d obstacle(matrix_static_position[i - 1][0], matrix_static_position[i - 1][1]);
        obstacles1.push_back(obstacle);

        // 打印每个静态障碍物的坐标
        // ROS_INFO("xin_Static Obstacle %d: (%f, %f)", i, obstacle(0), obstacle(1));
    }
  /////////////////////////////////////////////// end_xin /////////////////////////////////////////////////////////////////



    switch (dynamic_number)
    {
      case 1:
        obstacles = {};// 除动态障碍物本身外，还有0个动态障碍物,不执行
        obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      case 2:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1])};  // 不执行
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      case 3:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1])};  // 除动态障碍物本身外，还有2个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      case 4:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[3][0],matrix[3][1])
          };  // 除动态障碍物本身外，还有3个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      case 5:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[3][0],matrix[3][1]),
          Vector2d(matrix[4][0],matrix[4][1])
          };  // 除动态障碍物本身外，还有4个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
       case 6:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[3][0],matrix[3][1]),
          Vector2d(matrix[4][0],matrix[4][1]),
          Vector2d(matrix[5][0],matrix[5][1])
          };  // 除动态障碍物本身外，还有4个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break;
      default:break;
    }
 
    
    vector<Vector2d> path;  // 路径记录
    Vector2d position = start_position;  // 初始化机器人当前位置为起始位置
 
    // 路径规划
    if(a>1) {
      position.x()=start_position_last[0];
      position.y()=start_position_last[1];
    }

    path.push_back(start_position);  // 添加初始位置到路径记录
  
    // 计算吸引力：从当前位置到目标位置的吸引力
    Vector2d attraction = calculate_attraction(position, goal_position, attraction_coefficient);

    // 计算斥力：从当前位置到所有障碍物的斥力
    // Vector2d repulsion = calculate_repulsion(position, obstacles, repulsion_coefficient, repulsion_threshold);
    Vector2d repulsion=calculate_repulsion(position, goal_position, obstacles, repulsion_coefficient, repulsion_threshold) ;

    // 合成总力：吸引力减去斥力
    Vector2d force = attraction - repulsion;

  
    /**
     * 生成随机数
     */
    srand(time(NULL));
    double ax = ((double)rand() / RAND_MAX * 0.03 - 1.0)*0.01;
    double ay = ((double)rand() / RAND_MAX * 0.03 - 1.0)*0.01;
    Vector2d perturbation(ax, ay);
    // printf("随机数%f",ax);
    // printf("随机数%f\n",ay);

    position += step_size * (force.normalized()+Vector2d(ax, ay));

     start_position_last[0]=position.x();
     start_position_last[1]=position.y();
    //  ROS_INFO("position坐标x: %f", position.x());  // 输出当前x坐标
    //  ROS_INFO("position坐标y: %f", position.y());  // 输出当前y坐标

    //  ROS_INFO("start_position_last坐标x: %f", start_position_last.x());  // 输出当前x坐标
    //  ROS_INFO("start_position_last坐标y: %f", start_position_last.y());  // 输出当前y坐标
      
    if ((position - goal_position).norm() <= 0.5) 
    {
      ROS_INFO("抵达目标点%d",count_position);
      count_position+=1;
    }
    
    // 计算点云的变换矩阵，用于将点云从上一位置变换到当前位置
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << start_position_last[0] -last_x, start_position_last[1] - last_y, 0;
    
    //////////////////////箭头/////////////////////////////////////////////////////
    _set_vx=start_position_last[0] -last_x;
    _set_vy=start_position_last[1] -last_y;
    //////////////////////箭头/////////////////////////////////////////////////////

    // 对点云进行变换
    pcl::transformPointCloud(_cloud, _cloud, transform);

    // // 更新上一位置信息
    last_x = start_position_last[0];
    last_y = start_position_last[1];
    

    // ros::Time cur_time = ros::Time::now();  // 获取当前时间，该行代码被注释掉了
    // ROS_INFO("坐标x: %f", x);  // 输出当前x坐标
    // ROS_INFO("坐标y: %f", y);  // 输出当前y坐标
    // ROS_INFO("delta: %f", delta);  // 
    // ROS_INFO("前往目标点%d ing",count_position);
    // ROS_INFO("-----------------------------------------------------");

    return start_position_last;
  }
   /**
   * 4号动态障碍物的更新函数
   */
  Vector2d MovingCylinder::update4(double delta) {
   
    a=a+1;//进入函数的次数
    // ROS_INFO("测试a: %d", a);  // 输出当前x坐标

    /**
     * 目标点更迭 
     */
    Vector2d goal_position(0.0,10.0);
    switch (count_position)
    {
      case 1:goal_position.x()=-0;goal_position.y()=-7;  // 目标位置 
          break;
      case 2:goal_position.x()=12;goal_position.y()=-15;  // 目标位置 
          break;
      case 3:goal_position.x()=2;goal_position.y()=-5;  // 目标位置 
          break;
      case 4:goal_position.x()=9;goal_position.y()=15;  // 目标位置 
          break;
      default: count_position=1; goal_position.x()=-12;goal_position.y()=-15;  // 目标位置 
          break;
    }

    Vector2d start_position(-5.0, -5.0);  // 初始位置
    ////////////////////////////////////////////////start_xin/////////////////////////////////////////////////////////////////
    std::vector<Vector2d> obstacles ;
    std::vector<Vector2d> obstacles1 ;
    for (int i = 1; i <= static_number; i++)
    {
        Vector2d obstacle(matrix_static_position[i - 1][0], matrix_static_position[i - 1][1]);
        obstacles1.push_back(obstacle);

        // 打印每个静态障碍物的坐标
        // ROS_INFO("xin_Static Obstacle %d: (%f, %f)", i, obstacle(0), obstacle(1));
    }
  /////////////////////////////////////////////// end_xin /////////////////////////////////////////////////////////////////

    switch (dynamic_number)
    {
      case 1:
        obstacles = {};
          // 不执行
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
      
        break;
      case 2:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1])};  // 不执行
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
          
        break;
      case 3:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1])};  // 除动态障碍物本身外，还有2个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
          
        break;
      case 4:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[2][0],matrix[2][1])
          };  // 除动态障碍物本身外，还有3个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
      
        break;
      case 5:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[2][0],matrix[2][1]),
          Vector2d(matrix[4][0],matrix[4][1])
          };  // 除动态障碍物本身外，还有4个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        
        break;
      case 6:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[2][0],matrix[2][1]),
          Vector2d(matrix[4][0],matrix[4][1]),
          Vector2d(matrix[5][0],matrix[5][1])
          };  // 除动态障碍物本身外，还有5个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
          
        break;
      default:break;
    }
 
    
    vector<Vector2d> path;  // 路径记录
    Vector2d position = start_position;  // 初始化机器人当前位置为起始位置
 
    // 路径规划
    if(a>1) {
      position.x()=start_position_last[0];
      position.y()=start_position_last[1];
    }

    path.push_back(start_position);  // 添加初始位置到路径记录
  
    // 计算吸引力：从当前位置到目标位置的吸引力
    Vector2d attraction = calculate_attraction(position, goal_position, attraction_coefficient);

    // 计算斥力：从当前位置到所有障碍物的斥力
    // Vector2d repulsion = calculate_repulsion(position, obstacles, repulsion_coefficient, repulsion_threshold);
    Vector2d repulsion=calculate_repulsion(position, goal_position, obstacles, repulsion_coefficient, repulsion_threshold) ;
    // 合成总力：吸引力减去斥力
    Vector2d force = attraction - repulsion;

  
    /**
     * 生成随机数
     */
    srand(time(NULL));
    double ax = ((double)rand() / RAND_MAX * 0.03 - 1.0)*0.01;
    double ay = ((double)rand() / RAND_MAX * 0.03 - 1.0)*0.01;
    Vector2d perturbation(ax, ay);
    // printf("随机数%f",ax);
    // printf("随机数%f\n",ay);

    position += step_size * (force.normalized()+Vector2d(ax, ay));

     start_position_last[0]=position.x();
     start_position_last[1]=position.y();
    //  ROS_INFO("position坐标x: %f", position.x());  // 输出当前x坐标
    //  ROS_INFO("position坐标y: %f", position.y());  // 输出当前y坐标

    //  ROS_INFO("start_position_last坐标x: %f", start_position_last.x());  // 输出当前x坐标
    //  ROS_INFO("start_position_last坐标y: %f", start_position_last.y());  // 输出当前y坐标
      
    if ((position - goal_position).norm() <= 0.5) 
    {
      ROS_INFO("抵达目标点%d",count_position);
      count_position+=1;
    }
    
    // 计算点云的变换矩阵，用于将点云从上一位置变换到当前位置
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << start_position_last[0] -last_x, start_position_last[1] - last_y, 0;
    //////////////////////箭头/////////////////////////////////////////////////////
    _set_vx=start_position_last[0] -last_x;
    _set_vy=start_position_last[1] -last_y;
    //////////////////////箭头/////////////////////////////////////////////////////
    // 对点云进行变换
    pcl::transformPointCloud(_cloud, _cloud, transform);

    // // 更新上一位置信息
    last_x = start_position_last[0];
    last_y = start_position_last[1];
    

    // ros::Time cur_time = ros::Time::now();  // 获取当前时间，该行代码被注释掉了
    // ROS_INFO("坐标x: %f", x);  // 输出当前x坐标
    // ROS_INFO("坐标y: %f", y);  // 输出当前y坐标
    // ROS_INFO("delta: %f", delta);  // 
    // ROS_INFO("前往目标点%d ing",count_position);
    // ROS_INFO("-----------------------------------------------------");

    return start_position_last;
  }


   /**
   * 5号动态障碍物的更新函数
   */
  Vector2d MovingCylinder::update5(double delta) {
   
    a=a+1;//进入函数的次数
    // ROS_INFO("测试a: %d", a);  // 输出当前x坐标

    /**
     * 目标点更迭 
     */
    Vector2d goal_position(0.0,10.0);
    switch (count_position)
    {
      case 1:goal_position.x()=-12;goal_position.y()=5;  // 目标位置 
          break;
      case 2:goal_position.x()=12;goal_position.y()=-15;  // 目标位置 
          break;
      case 3:goal_position.x()=1;goal_position.y()=-4;  // 目标位置 
          break;
      case 4:goal_position.x()=12;goal_position.y()=-5;  // 目标位置 
          break;
      default: count_position=1; goal_position.x()=-12;goal_position.y()=-15;  // 目标位置 
          break;
    }

    Vector2d start_position(-1.0, -5.0);  // 初始位置

    ////////////////////////////////////////////////start_xin/////////////////////////////////////////////////////////////////
    std::vector<Vector2d> obstacles ;
    std::vector<Vector2d> obstacles1 ;
    for (int i = 1; i <= static_number; i++)
    {
        Vector2d obstacle(matrix_static_position[i - 1][0], matrix_static_position[i - 1][1]);
        obstacles1.push_back(obstacle);

        // 打印每个静态障碍物的坐标
        // ROS_INFO("xin_Static Obstacle %d: (%f, %f)", i, obstacle(0), obstacle(1));
    }
  /////////////////////////////////////////////// end_xin /////////////////////////////////////////////////////////////////


    switch (dynamic_number)
    {
      case 1:
        obstacles = {};  // 不执行
        obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());

        break;
      case 2:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1])};  // 不执行
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());

        break;
      case 3:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1])};  // 除动态障碍物本身外，还有2个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());

        break;
      case 4:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[2][0],matrix[2][1])
          };  // 除动态障碍物本身外，还有4个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());

        break;
        case 5:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[2][0],matrix[2][1]),
          Vector2d(matrix[3][0],matrix[3][1])
          };  // 除动态障碍物本身外，还有3个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());


        break;
      case 6:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[2][0],matrix[2][1]),
          Vector2d(matrix[3][0],matrix[3][1]),
          Vector2d(matrix[5][0],matrix[5][1])
          };  // 除动态障碍物本身外，还有3个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());


        break; 
      default:break;
    }
 
    
    vector<Vector2d> path;  // 路径记录
    Vector2d position = start_position;  // 初始化机器人当前位置为起始位置
 
    // 路径规划
    if(a>1) {
      position.x()=start_position_last[0];
      position.y()=start_position_last[1];
    }

    path.push_back(start_position);  // 添加初始位置到路径记录
  
    // 计算吸引力：从当前位置到目标位置的吸引力
    Vector2d attraction = calculate_attraction(position, goal_position, attraction_coefficient);

    // 计算斥力：从当前位置到所有障碍物的斥力
    // Vector2d repulsion = calculate_repulsion(position, obstacles, repulsion_coefficient, repulsion_threshold);
    Vector2d repulsion=calculate_repulsion(position, goal_position, obstacles, repulsion_coefficient, repulsion_threshold) ;
    // 合成总力：吸引力减去斥力
    Vector2d force = attraction - repulsion;

  
    /**
     * 生成随机数
     */
    srand(time(NULL));
    double ax = ((double)rand() / RAND_MAX * 0.03 - 1.0)*0.01;
    double ay = ((double)rand() / RAND_MAX * 0.03 - 1.0)*0.01;
    Vector2d perturbation(ax, ay);
    // printf("随机数%f",ax);
    // printf("随机数%f\n",ay);

    position += step_size * (force.normalized()+Vector2d(ax, ay));

     start_position_last[0]=position.x();
     start_position_last[1]=position.y();
    //  ROS_INFO("position坐标x: %f", position.x());  // 输出当前x坐标
    //  ROS_INFO("position坐标y: %f", position.y());  // 输出当前y坐标

    //  ROS_INFO("start_position_last坐标x: %f", start_position_last.x());  // 输出当前x坐标
    //  ROS_INFO("start_position_last坐标y: %f", start_position_last.y());  // 输出当前y坐标
      
    if ((position - goal_position).norm() <= 0.5) 
    {
      ROS_INFO("抵达目标点%d",count_position);
      count_position+=1;
    }
    
    // 计算点云的变换矩阵，用于将点云从上一位置变换到当前位置
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << start_position_last[0] -last_x, start_position_last[1] - last_y, 0;

    //////////////////////箭头/////////////////////////////////////////////////////
    _set_vx=start_position_last[0] -last_x;
    _set_vy=start_position_last[1] -last_y;
    //////////////////////箭头/////////////////////////////////////////////////////

    // 对点云进行变换
    pcl::transformPointCloud(_cloud, _cloud, transform);

    // // 更新上一位置信息
    last_x = start_position_last[0];
    last_y = start_position_last[1];
    

    // ros::Time cur_time = ros::Time::now();  // 获取当前时间，该行代码被注释掉了
    // ROS_INFO("坐标x: %f", x);  // 输出当前x坐标
    // ROS_INFO("坐标y: %f", y);  // 输出当前y坐标
    // ROS_INFO("delta: %f", delta);  // 
    // ROS_INFO("前往目标点%d ing",count_position);
    // ROS_INFO("-----------------------------------------------------");

    return start_position_last;
  }

  

    /**
   * 6号动态障碍物的更新函数
   */
  Vector2d MovingCylinder::update6(double delta) {
   
    a=a+1;//进入函数的次数
    // ROS_INFO("测试a: %d", a);  // 输出当前x坐标

    /**
     * 目标点更迭 
     */
    Vector2d goal_position(0.0,10.0);
    switch (count_position)
    {
      case 1:goal_position.x()=-12;goal_position.y()=-15;  // 目标位置 
          break;
      case 2:goal_position.x()=7;goal_position.y()=-15;  // 目标位置 
          break;
      case 3:goal_position.x()=-12;goal_position.y()=2;  // 目标位置 
          break;
      case 4:goal_position.x()=-3;goal_position.y()=-15;  // 目标位置 
          break;
      default: count_position=1; goal_position.x()=-12;goal_position.y()=-15;  // 目标位置 
          break;
    }

    Vector2d start_position(-1.0, -5.0);  // 初始位置
        ////////////////////////////////////////////////start_xin/////////////////////////////////////////////////////////////////
    std::vector<Vector2d> obstacles ;
    std::vector<Vector2d> obstacles1 ;
    for (int i = 1; i <= static_number; i++)
    {
        Vector2d obstacle(matrix_static_position[i - 1][0], matrix_static_position[i - 1][1]);
        obstacles1.push_back(obstacle);

        // 打印每个静态障碍物的坐标
        // ROS_INFO("xin_Static Obstacle %d: (%f, %f)", i, obstacle(0), obstacle(1));
    }
  /////////////////////////////////////////////// end_xin /////////////////////////////////////////////////////////////////


    switch (dynamic_number)
    {
      case 1:
        obstacles = {};  // 不执行
        obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());

        break;
      case 2:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1])};  // 不执行
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());


        break;
      case 3:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1])};  // 除动态障碍物本身外，还有2个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());


        break;
      case 4:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[2][0],matrix[2][1])
          };  // 除动态障碍物本身外，还有4个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());


        break;
        case 5:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[2][0],matrix[2][1]),
          Vector2d(matrix[3][0],matrix[3][1])
          };  // 除动态障碍物本身外，还有3个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());

        break;
      case 6:
        obstacles = {
          Vector2d(matrix[0][0],matrix[0][1]),
          Vector2d(matrix[1][0],matrix[1][1]),
          Vector2d(matrix[2][0],matrix[2][1]),
          Vector2d(matrix[3][0],matrix[3][1]),
          Vector2d(matrix[4][0],matrix[4][1])
          };  // 除动态障碍物本身外，还有3个动态障碍物
          obstacles.insert(obstacles.end(), obstacles1.begin(), obstacles1.end());
        break; 
      default:break;
    }
 
    
    vector<Vector2d> path;  // 路径记录
    Vector2d position = start_position;  // 初始化机器人当前位置为起始位置
 
    // 路径规划
    if(a>1) {
      position.x()=start_position_last[0];
      position.y()=start_position_last[1];
    }

    path.push_back(start_position);  // 添加初始位置到路径记录
  
    // 计算吸引力：从当前位置到目标位置的吸引力
    Vector2d attraction = calculate_attraction(position, goal_position, attraction_coefficient);

    // 计算斥力：从当前位置到所有障碍物的斥力
    // Vector2d repulsion = calculate_repulsion(position, obstacles, repulsion_coefficient, repulsion_threshold);
    Vector2d repulsion=calculate_repulsion(position, goal_position, obstacles, repulsion_coefficient, repulsion_threshold) ;
    // 合成总力：吸引力减去斥力
    Vector2d force = attraction - repulsion;

  
    /**
     * 生成随机数
     */
    srand(time(NULL));
    double ax = ((double)rand() / RAND_MAX * 0.03 - 1.0)*0.01;
    double ay = ((double)rand() / RAND_MAX * 0.03 - 1.0)*0.01;
    Vector2d perturbation(ax, ay);
    // printf("随机数%f",ax);
    // printf("随机数%f\n",ay);

    position += step_size * (force.normalized()+Vector2d(ax, ay));

     start_position_last[0]=position.x();
     start_position_last[1]=position.y();
    //  ROS_INFO("position坐标x: %f", position.x());  // 输出当前x坐标
    //  ROS_INFO("position坐标y: %f", position.y());  // 输出当前y坐标

    //  ROS_INFO("start_position_last坐标x: %f", start_position_last.x());  // 输出当前x坐标
    //  ROS_INFO("start_position_last坐标y: %f", start_position_last.y());  // 输出当前y坐标
      
    if ((position - goal_position).norm() <= 0.5) 
    {
      ROS_INFO("抵达目标点%d",count_position);
      count_position+=1;
    }
    
    // 计算点云的变换矩阵，用于将点云从上一位置变换到当前位置
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << start_position_last[0] -last_x, start_position_last[1] - last_y, 0;

    //////////////////////箭头/////////////////////////////////////////////////////
    _set_vx=start_position_last[0] -last_x;
    _set_vy=start_position_last[1] -last_y;
    //////////////////////箭头/////////////////////////////////////////////////////

    // 对点云进行变换
    pcl::transformPointCloud(_cloud, _cloud, transform);

    // // 更新上一位置信息
    last_x = start_position_last[0];
    last_y = start_position_last[1];
    

    // ros::Time cur_time = ros::Time::now();  // 获取当前时间，该行代码被注释掉了
    // ROS_INFO("坐标x: %f", x);  // 输出当前x坐标
    // ROS_INFO("坐标y: %f", y);  // 输出当前y坐标
    // ROS_INFO("delta: %f", delta);  // 
    // ROS_INFO("前往目标点%d ing",count_position);
    // ROS_INFO("-----------------------------------------------------");

    return start_position_last;
  }
////////////////////////////////////////////////////end 4.0///////////////////////////////////////////////////////////////////////////////////

}  // namespace dynamic_map_objects

#endif  // __MOVING_CYLINDER_H__

