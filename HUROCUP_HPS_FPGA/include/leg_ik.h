#ifndef LEG_IK_H_
#define LEG_IK_H_
/******************* Include libarary*********************/
#include <stdio.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <map>
#include <vector>
#include <cstring>
#include <cstdio>
#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
/********************* Define ***************************/
#define PI                  3.1415926535897932384626433832795
#define PI_2                1.5707963267948966192313216916398
#define DEGREE_2_PI         PI / 180
/******************* Leg Parameter **********************/
#define LEG_L1              WAIST_WIDTH/2
#define LEG_L2              3.914   // cm
#define LEG_L3              12.5    // 大腿長度
#define LEG_L4              12.5    // 小腿長度
#define LEG_L5              3.52
#define LEG_LENGTH          -32.434     // 總長度
/***************** Robot Parameter **********************/
#define WAIST_WIDTH         9.0     // cm
#define COM_HEIGHT          29.5
#define LEG_HEITHT          23.5    // 12&14號馬達主轉距離
#define STANDING_HEIGHT     -30.934    // 站立高度 
/********************************************************/
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 12, 1> Vector12d;

class LegTrajectory
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegTrajectory();
    ~LegTrajectory();

    Vector3d position, rpy_radians;
    bool set_speed, get_position, start;

    void initial();
    void set_parameter(bool left_hand_);
    void set_walk_parameter(Vector3d position_, Vector3d rpy_radian_, bool left_);
    void walking();

private:
    int name_cont_;
    Affine3d left_endpoint, right_endpoint;
    Matrix3d walk_rm;
};

class LegInverseKinematic
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegInverseKinematic();
    ~LegInverseKinematic();

    struct Endpoint{
        Affine3d init_endpoint, now_endpoint, stand_endpoint, cal_endpoint, base;
        Vector6d output_rad, save_rad;
        bool ik_error;
    };

    struct Endpoint left_leg, right_leg;
    double now_com_height, now_standing_height, now_waist_width;
    bool change_flag;
    
    void initial_ik();
    void after_initial_ik();
    Vector12d run();
    bool limit_check(Vector6d theta);
    // bool activities_area_check(Affine3d end_point);
    Vector6d calculate_leg_ik(Affine3d end_point, Affine3d base);
    Affine3d calculate_endpoint(Vector3d rpy, Vector3d pos);
    void stand();
    void set_robot_parameter(double com_height, double standing_height, double waist_width);

    inline Matrix3d get_rotation_matrix(Vector3d radians_) {
    Matrix3d res;
    res = AngleAxisd(radians_(2), Vector3d::UnitZ())
        * AngleAxisd(radians_(1), Vector3d::UnitY())
        * AngleAxisd(radians_(0), Vector3d::UnitX());
    return res;
    }

    string DtoS(double value);
    void saveData();
    void pushData();
    std::map<std::string, std::vector<float>> map_lik;

private:
    int name_cont_;
    Vector6d theta_min, theta_max;
};

#endif