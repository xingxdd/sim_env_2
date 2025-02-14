

#ifndef __MOVING_COLUMN_H__
#define __MOVING_COLUMN_H__

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <random>

namespace dynamic_map_objects {

class MovingColumn {
 private:
  std::uniform_real_distribution<double> _rand_x, _rand_y, _rand_w, _rand_h, _rand_v, _rand_a;
  std::default_random_engine _generator;
  double                                 _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _v_h, _a_h;
  double                                 _resolution;
  int                                    _mode = 0;
  bool                                   _obs_type;

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
  double                         _set_vx, _set_vy;
  double                         _init_vx, _init_vy;
  MovingColumn(double                      x_l,
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
                 double                      resolution,
                 bool                        use_static);
  ~MovingColumn() {}
  void setMode(int m);
  void setPos(Eigen::Vector2d set_pos);
  void update(double delta);
  void noiseInit(std::mt19937 gen, std::normal_distribution<double> vel_error, std::normal_distribution<double> pos_error);
};

MovingColumn::MovingColumn(double                          x_l,
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
                               double                      resolution,
                               bool                        use_static)
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
  _obs_type   = use_static;

  // genrate random 2D position, width, height
  x = _rand_x(eng);
  y = _rand_y(eng);
  h = _rand_h(eng);
  w = _rand_w(eng);

  _de_acc = 0.5;

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

void MovingColumn::noiseInit(std::mt19937 gen, std::normal_distribution<double> vel_error, std::normal_distribution<double> pos_error){
  _gen = gen;
  _vel_error = vel_error;
  _pos_error = pos_error;
}

void MovingColumn::setMode(int m) {
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

void MovingColumn::setPos(Eigen::Vector2d set_pos){
  x = set_pos(0);
  y = set_pos(1);
}

void MovingColumn::update(double delta) {
    if(!_obs_type){
      double new_acc_x = _rand_a(_generator);
      double new_acc_y = _rand_a(_generator);

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
      double s_vx = _set_vx + new_acc_x * delta;
      double s_vy = _set_vy + new_acc_y * delta;

      // _set_vx += new_acc_x * delta;
      // _set_vy += new_acc_y * delta;
      
      _set_vx = _set_vx > 2 * _v_h ? 2 * _v_h : (_set_vx < -2 * _v_h ? -2 * _v_h : _set_vx);
      _set_vy = _set_vy > 2 * _v_h ? 2 * _v_h : (_set_vy < -2 * _v_h ? -2 * _v_h : _set_vy);
      vx = _set_vx + _vel_error(_gen);
      vy = _set_vy + _vel_error(_gen);
      // ROS_INFO("RAW VEL: %f, %f", vx, vy);
      double delta_x = vx * delta;
      double delta_y = vy * delta;
      x += delta_x;
      y += delta_y;
      x += _pos_error(_gen);
      y += _pos_error(_gen);

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
    }else{
      // x += _pos_error(_gen);
      // y += _pos_error(_gen);
    }

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << x - last_x, y - last_y, 0;
    pcl::transformPointCloud(_cloud, _cloud, transform);

    last_x = x;
    last_y = y;
}

}  // namespace dynamic_map_objects

#endif  // __MOVING_COLUMN_H__