/**
 * @file moving_circle.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-07-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __MOVING_CIRCLE_H__
#define __MOVING_CIRCLE_H__

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <random>





namespace dynamic_map_objects {
class MovingCircle {
 private:
  std::uniform_real_distribution<double> _rand_x, _rand_y, _rand_z, _rand_r, _rand_theta, _rand_v, _rand_a,
      _rand_omega;
  std::default_random_engine _generator;
  double _x_l, _x_h, _y_l, _y_h, _r_l, _r_h, _r_2, _v_h, _a_h, _omega_h;
  double _resolution;

  std::mt19937 _gen;
  std::normal_distribution<double> _vel_error;
  std::normal_distribution<double> _pos_error;

  double _de_acc;

 public:
  pcl::PointCloud<pcl::PointXYZ> _cloud;

  double             x;       // x coordinate of the circle center
  double             y;       // y coordinate of the circle center
  double             last_x;
  double             last_y;
  double             last_z;
  double             z;       // y coordinate of the circle center
  double             r1, r2;  // outer radius
  double             dr;      // thickness of the circle
  double             vx;
  double             vy;
  double             _set_vx, _set_vy;
  double             _init_vx, _init_vy;
  double             omega;  // angular velocity
  double             theta;  // angle of the circle center
  Eigen::Quaterniond q;
  MovingCircle(double                      x_l,
               double                      x_h,
               double                      y_l,
               double                      y_h,
               double                      z_l,
               double                      z_h,
               double                      r_l,
               double                      r_h,
               double                      dr,
               double                      v_h,
               double                      a_h,
               std::default_random_engine &eng,
               double                      resolution)
      : _rand_x(x_l, x_h)
      , _rand_y(y_l, y_h)
      , _rand_z(z_l, z_h)
      , _rand_r(r_l, r_h)
      , _rand_theta(0, 2 * M_PI)
      , _rand_v(-v_h, v_h)
      , _rand_a(-a_h, a_h)
      , _x_l(x_l)
      , _x_h(x_h)
      , _y_l(y_l)
      , _y_h(y_h)
      , _r_2(dr)
      , _v_h(v_h)
      , _a_h(a_h)
      , _resolution(resolution) {
    x        = _rand_x(eng);
    y        = _rand_y(eng);
    z        = _rand_z(eng);
    // vx       = _rand_v(eng);
    // vy       = _rand_v(eng);
    theta    = _rand_theta(eng);
    double r = _rand_r(eng);
    // random point in grid scale
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;
    last_x = x;
    last_y = y;
    last_z = z;

    _de_acc = 0.5;

  _set_vx = random() % 2 == 0 ? _v_h : -_v_h;
  _set_vy = random() % 2 == 0 ? _v_h : -_v_h;
  _init_vx = _set_vx > 0 ? _set_vx : - _set_vx;
  _init_vy = _set_vy > 0 ? _set_vy : - _set_vy;


    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
    q = Eigen::Quaterniond(rotate);
    q = q * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());

    r1 = r + _r_2 / 2;
    r2 = r - _r_2 / 2;

    // draw a circle centered at (x,y,z)
    pcl::PointXYZ   pt;
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 2 * M_PI; angle += _resolution / 2) {
      cpt(0) = 0.0;
      cpt(1) = r1 * cos(angle);
      cpt(2) = r2 * sin(angle);
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution, ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt.x   = cpt_if(0);
            pt.y   = cpt_if(1);
            pt.z   = cpt_if(2);
            _cloud.push_back(pt);
          }
    }
    _cloud.width    = _cloud.points.size();
    _cloud.height   = 1;
    _cloud.is_dense = true;
  };

  ~MovingCircle() {}

  void noiseInit(std::mt19937 gen, std::normal_distribution<double> vel_error, std::normal_distribution<double> pos_error){
    _gen = gen;
    _vel_error = vel_error;
    _pos_error = pos_error;
  }

  /**
   * MovingCircle 类的 update 方法负责更新动态圆形障碍物的位置和速度
   */
  /*添加部分*/



  void update(double delta) {
    /**
     * 1.随机加速度
     *   生成随机的加速度值，用于模拟障碍物加速度的变化
     */
    double new_acc_x = _rand_a(_generator);
    double new_acc_y = _rand_a(_generator);
    /**
     * 2.调整速度
     *   根据障碍物与边界的距离调整速度，以模拟障碍物在接近边界时减速的效果
     */
    if(x - _x_l <= pow(_init_vx, 2) / (2.0 * _de_acc)){
      _set_vx += _de_acc * delta;
    }
    else if(_x_h - x <= pow(_init_vx, 2) / (2.0 * _de_acc)){
      _set_vx -= _de_acc * delta;
    }
    if(y - _y_l <= pow(_init_vy, 2) / (2.0 * _de_acc)){
      _set_vy += _de_acc * delta;
    }
    else if(_y_h - y <= pow(_init_vy, 2) / (2.0 * _de_acc)){
      _set_vy -= _de_acc * delta;
    }

    /**
     * 3.更新速度
     *   随机加速度与时间差 delta 相乘，然后加到当前速度上，得到新的速度
     */
    double s_vx = _set_vx + new_acc_x * delta;    
    double s_vy = _set_vy + new_acc_y * delta;

    /**
     * 4.限制速度
     *   限制速度不超过最大速度 _v_h
     */
    // _set_vx += new_acc_x * delta;
    // _set_vy += new_acc_y * delta;
    _set_vx = _set_vx > 2 * _v_h ? 2 * _v_h : (_set_vx < -2 * _v_h ? -2 * _v_h : _set_vx);
    _set_vy = _set_vy > 2 * _v_h ? 2 * _v_h : (_set_vy < -2 * _v_h ? -2 * _v_h : _set_vy);

    /**
     * 5.添加噪声
     *   向速度中添加正态分布的噪声，以模拟实际运动中的不确定性
     */
    vx = _set_vx + _vel_error(_gen);
    vy = _set_vy + _vel_error(_gen);

    /**
     * 6.更新位置
     *   根据速度和时间差 delta 计算位置的变化，并更新障碍物的位置
     */
    double delta_x = vx * delta;
    double delta_y = vy * delta;

    /**
     * 7.添加位置噪声
     *   向位置中添加正态分布的噪声
     */
    x += delta_x;
    y += delta_y;
    x += _pos_error(_gen);
    y += _pos_error(_gen);

    /**
     * 8.边界检查
     *  检查障碍物是否超出边界，如果超出，则反向速度并返回
     */
    if(x < _x_l || x > _x_h){
      x -= delta_x;
      _set_vx = -_set_vx;
      return;
    }

    if(y < _y_l || y > _y_h){
      y -= delta_y;
      _set_vy = -_set_vy;
      return;
    }

    /**
     * 9.更新点云
     *   创建一个变换矩阵，将点云中的每个点根据障碍物的新位置进行更新
     */
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << x - last_x, y - last_y, 0;
    pcl::transformPointCloud(_cloud, _cloud, transform);

    /**
     * 10.更新最后位置
     *  更新障碍物的最后位置
     */
    last_x = x;
    last_y = y;
  }
};
}  // namespace dynamic_map_objects
#endif  // __MOVING_CIRCLE_H__
