#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include <time.h>
using namespace std;
using namespace Eigen;

// 计算吸引力
Vector2d calculate_attraction(const Vector2d& position, const Vector2d& goal, double attraction_coefficient) {
    double distance = (position - goal).norm();  // 计算当前位置到目标位置的欧几里得距离
    Vector2d attraction = attraction_coefficient * (goal - position) / distance;  // 根据人工势场法公式计算吸引力向量
    return attraction;  // 返回计算得到的吸引力向量
}

// 计算斥力
Vector2d calculate_repulsion(const Vector2d& position,const Vector2d& goal, const vector<Vector2d>& obstacles, double repulsion_coefficient, double repulsion_threshold) {
    Vector2d repulsion_force = Vector2d::Zero();
    int q=1;
    for (const auto& obstacle : obstacles) {
        double distance = (position - obstacle).norm();  // 计算当前位置到障碍物的欧几里得距离
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

int main() {
    // 环境参数
    Vector2d start_position(0.0, 0.0);  // 初始位置
    Vector2d goal_position(5.0, 10.0);  // 目标位置
    vector<Vector2d> obstacles = {Vector2d(4.0, 5.0), Vector2d(7.0, 8.0), Vector2d(10.0, 3.0)};  // 障碍物位置

    // 势场参数
    double attraction_coefficient = 1.0;  // 吸引力系数
    double repulsion_coefficient = 100.0;  // 斥力系数
    double repulsion_threshold = 2.0;  // 斥力作用范围

    // 路径规划参数
    double step_size = 0.1;  // 步长
    int max_iterations = 1000;  // 最大迭代次数
    vector<Vector2d> path;  // 路径记录
    path.push_back(start_position);  // 添加初始位置到路径记录
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    srand(time(NULL));

    // 生成随机数
    double ax = (double)rand() / RAND_MAX * 0.03 - 1.0;
    double ay = (double)rand() / RAND_MAX * 0.03 - 1.0;
    Vector2d perturbation(ax, ay);
    printf("随机数%f",ax);
    printf("随机数%f\n",ay);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // 路径规划
    Vector2d position = start_position;  // 初始化机器人当前位置为起始位置
    for (int i = 0; i < max_iterations; ++i) {  // 开始循环，最多执行 max_iterations 次
        // 计算吸引力：从当前位置到目标位置的吸引力
        Vector2d attraction = calculate_attraction(position, goal_position, attraction_coefficient);

        // 计算斥力：从当前位置到所有障碍物的斥力
        Vector2d repulsion = calculate_repulsion(position, goal_position,obstacles, repulsion_coefficient, repulsion_threshold);

        // 合成总力：吸引力减去斥力
        Vector2d force = attraction - repulsion;

        // 根据合力更新位置：步长乘以归一化后的合力
        position += step_size * force.normalized();

        // 记录当前步骤的位置到路径列表中
        path.push_back(position);

        // 检查机器人是否到达目标位置：如果当前位置与目标位置的距离小于等于0.5，则认为到达目标
        if ((position - goal_position).norm() <= 0.5) {
            break;  // 跳出循环，结束路径规划
        }
    }

    // 输出路径
    cout << "Path:" << endl;
    for (const auto& p : path) {
        cout << "(" << p.x() << ", " << p.y() << ")" << endl;
    }

    return 0;
}


// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <Eigen/Dense>
// #include <Eigen/Geometry>
// #include <random>
// #include <time.h>
// using namespace std;
// using namespace Eigen;

// // 计算吸引力
// Vector2d calculate_attraction(const Vector2d& position, const Vector2d& goal, double attraction_coefficient) {
//     double distance = (position - goal).norm();  // 计算当前位置到目标位置的欧几里得距离
//     Vector2d attraction = attraction_coefficient * (goal - position) / distance;  // 根据人工势场法公式计算吸引力向量
//     return attraction;  // 返回计算得到的吸引力向量
// }

// // 计算斥力
// Vector2d calculate_repulsion(const Vector2d& position, const vector<Vector2d>& obstacles, double repulsion_coefficient, double repulsion_threshold) {
//     Vector2d repulsion_force = Vector2d::Zero();
//     for (const auto& obstacle : obstacles) {
//         double distance = (position - obstacle).norm();  // 计算当前位置到障碍物的欧几里得距离
//         if (distance < repulsion_threshold) {  // 如果距离小于斥力作用范围
//             Vector2d obstacle_vec = position - obstacle;
//             repulsion_force += repulsion_coefficient * (1 / repulsion_threshold - 1 / distance) * (obstacle_vec / pow(distance, 3));  // 计算当前位置到障碍物的向量
//         }
//     }
//     return repulsion_force;  // 返回计算得到的斥力向量
// }

// int main() {
//     // 环境参数
//     Vector2d start_position(0.0, 0.0);  // 初始位置
//     Vector2d goal_position(5.0, 10.0);  // 目标位置
//     vector<Vector2d> obstacles = {Vector2d(4.0, 5.0), Vector2d(7.0, 8.0), Vector2d(10.0, 3.0)};  // 障碍物位置

//     // 势场参数
//     double attraction_coefficient = 1.0;  // 吸引力系数
//     double repulsion_coefficient = 100.0;  // 斥力系数
//     double repulsion_threshold = 2.0;  // 斥力作用范围

//     // 路径规划参数
//     double step_size = 0.1;  // 步长
//     int max_iterations = 1000;  // 最大迭代次数
//     vector<Vector2d> path;  // 路径记录
//     path.push_back(start_position);  // 添加初始位置到路径记录
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     srand(time(NULL));

//     // 生成随机数
//     double ax = (double)rand() / RAND_MAX * 0.03 - 1.0;
//     double ay = (double)rand() / RAND_MAX * 0.03 - 1.0;
//     Vector2d perturbation(ax, ay);
//     printf("随机数%f",ax);
//     printf("随机数%f\n",ay);

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//     // 路径规划
//     Vector2d position = start_position;  // 初始化机器人当前位置为起始位置
//     for (int i = 0; i < max_iterations; ++i) {  // 开始循环，最多执行 max_iterations 次
//         // 计算吸引力：从当前位置到目标位置的吸引力
//         Vector2d attraction = calculate_attraction(position, goal_position, attraction_coefficient);

//         // 计算斥力：从当前位置到所有障碍物的斥力
//         Vector2d repulsion = calculate_repulsion(position, obstacles, repulsion_coefficient, repulsion_threshold);

//         // 合成总力：吸引力减去斥力
//         Vector2d force = attraction - repulsion;

//         // 根据合力更新位置：步长乘以归一化后的合力
//         position += step_size * force.normalized();

//         // 记录当前步骤的位置到路径列表中
//         path.push_back(position);

//         // 检查机器人是否到达目标位置：如果当前位置与目标位置的距离小于等于0.5，则认为到达目标
//         if ((position - goal_position).norm() <= 0.5) {
//             break;  // 跳出循环，结束路径规划
//         }
//     }

//     // 输出路径
//     cout << "Path:" << endl;
//     for (const auto& p : path) {
//         cout << "(" << p.x() << ", " << p.y() << ")" << endl;
//     }

//     return 0;
// }