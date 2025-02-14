// #include <Eigen/Dense>
// #include <vector>
// #include <ros/ros.h>

// using namespace Eigen;

// class MovingCylinder
// {
// public:
//     int static_number; // 静态障碍物的数量
//     double matrix_static_position[10][2]; // 存储静态障碍物的坐标，假设最多有10个静态障碍物

//     MovingCylinder()
//     {
//         // 初始化静态障碍物的数量和坐标
//         static_number = 3;
//         matrix_static_position[0][0] = 1.0; matrix_static_position[0][1] = 2.0;
//         matrix_static_position[1][0] = 3.0; matrix_static_position[1][1] = 4.0;
//         matrix_static_position[2][0] = 5.0; matrix_static_position[2][1] = 6.0;
//     }

//     std::vector<Vector2d> obs_static_position_exchange() //静态坐标变换
//     {
//         std::vector<Vector2d> obstacles;
//         for (int i = 1; i <= static_number; i++)
//         {
//             Vector2d obstacle(matrix_static_position[i - 1][0], matrix_static_position[i - 1][1]);
//             obstacles.push_back(obstacle);

//             // 打印每个静态障碍物的坐标
//             ROS_INFO("Static Obstacle %d: (%f, %f)", i, obstacle(0), obstacle(1));
//         }
        
//         return obstacles;
//     }
// };

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "moving_cylinder_node");
//     ros::NodeHandle nh;

//     MovingCylinder mc;
//     std::vector<Vector2d> obstacles = mc.obs_static_position_exchange();
//     std::vector<Vector2d> obstacles1 ={Vector2d(-8.0, 0.0), Vector2d(0.0, 0.0), Vector2d(4.0, 4.0)};
//     obstacles1.insert(obstacles1.end(),obstacles.begin(),obstacles.end());
//     for (size_t i = 0; i < obstacles1.size(); ++i) {
//         std::cout << "(" << obstacles1[i](0) << ", " << obstacles1[i](1) << ")" << std::endl;
//     }
//     ros::spin();
//     return 0;
// }