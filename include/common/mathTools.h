
#ifndef MATHTOOLS_H
#define MATHTOOLS_H

#include <stdio.h>
#include <iostream>
#include "common/mathTypes.h"



template<typename T1, typename T2>
inline T1 max(const T1 a, const T2 b){
	return (a > b ? a : b);
}

template<typename T1, typename T2>
inline T1 min(const T1 a, const T2 b){
	return (a < b ? a : b);
}

template<typename T>
inline T saturation(const T a, Vec2 limits){
    T lowLim, highLim;
    if(limits(0) > limits(1)){
        lowLim = limits(1);
        highLim= limits(0);
    }else{
        lowLim = limits(0);
        highLim= limits(1);
    }

    if(a < lowLim){
        return lowLim;
    }
    else if(a > highLim){
        return highLim;
    }
    else{
        return a;
    }
}

template<typename T0, typename T1>
inline T0 killZeroOffset(T0 a, const T1 limit){
    if((a > -limit) && (a < limit)){
        a = 0;
    }
    return a;
}

template<typename T0, typename T1, typename T2>
inline T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1){
	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
}

template<typename T>
inline T windowFunc(const T x, const T windowRatio, const T xRange=1.0, const T yRange=1.0){
    if((x < 0)||(x > xRange)){
        std::cout << "[ERROR][windowFunc] The x=" << x << ", which should between [0, xRange]" << std::endl;
    }
    if((windowRatio <= 0)||(windowRatio >= 0.5)){
        std::cout << "[ERROR][windowFunc] The windowRatio=" << windowRatio << ", which should between [0, 0.5]" << std::endl;
    }

    if(x/xRange < windowRatio){
        return x * yRange / (xRange * windowRatio);
    }
    else if(x/xRange > 1 - windowRatio){
        return yRange * (xRange - x)/(xRange * windowRatio);
    }
    else{
        return yRange;
    }
}

template<typename T1, typename T2>
inline void updateAverage(T1 &exp, T2 newValue, double n){
    if(exp.rows()!=newValue.rows()){
        std::cout << "The size of updateAverage is error" << std::endl;
        exit(-1);
    }
    if(fabs(n - 1) < 0.001){
        exp = newValue;
    }else{
        exp = exp + (newValue - exp)/n;
    }
}

template<typename T1, typename T2, typename T3>
inline void updateCovariance(T1 &cov, T2 expPast, T3 newValue, double n){
    if( (cov.rows()!=cov.cols()) || (cov.rows() != expPast.rows()) || (expPast.rows()!=newValue.rows())){
        std::cout << "The size of updateCovariance is error" << std::endl;
        exit(-1);
    }
    if(fabs(n - 1) < 0.1){
        cov.setZero();
    }else{
        cov = cov*(n-1)/n + (newValue-expPast)*(newValue-expPast).transpose()*(n-1)/(n*n);
    }
}

template<typename T1, typename T2, typename T3>
inline void updateAvgCov(T1 &cov, T2 &exp, T3 newValue, double n){
    // The order matters!!! covariance first!!!
    updateCovariance(cov, exp, newValue, n);
    updateAverage(exp, newValue, n);
}

inline RotMat rotx(const double &theta) {
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat R;
    R << 1, 0, 0, 0, c, -s, 0, s, c;
    return R;
}

inline RotMat roty(const double &theta) {
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat R;
    R << c, 0, s, 0, 1, 0, -s, 0, c;
    return R;
}

inline RotMat rotz(const double &theta) {
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat R;
    R << c, -s, 0, s, c, 0, 0, 0, 1;
    return R;
}

inline Mat2 skew(const double& w){
    Mat2 mat; mat.setZero();
    mat(0, 1) = -w;
    mat(1, 0) =  w;
    return mat;
}

inline Mat3 skew(const Vec3& v) {
    Mat3 m;
    m << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return m;
}

inline RotMat rpyToRotMat(const double& row, const double& pitch, const double& yaw) {
    RotMat m = rotz(yaw) * roty(pitch) * rotx(row);
    return m;
}

inline Vec3 rotMatToRPY(const Mat3& R) {
    Vec3 rpy;
    rpy(0) = atan2(R(2,1),R(2,2));
    rpy(1) = asin(-R(2,0));
    rpy(2) = atan2(R(1,0),R(0,0));
    return rpy;
}

inline RotMat quatToRotMat(const Quat& q) {
    double e0 = q(0);
    double e1 = q(1);
    double e2 = q(2);
    double e3 = q(3);

    RotMat R;
    R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
            2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
            1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
            2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
            1 - 2 * (e1 * e1 + e2 * e2);
    return R;
}

inline Vec3 rotMatToExp(const RotMat& rm){
    double cosValue = rm.trace()/2.0-1/2.0;
    if(cosValue > 1.0f){
        cosValue = 1.0f;
    }else if(cosValue < -1.0f){
        cosValue = -1.0f;
    }

    double angle = acos(cosValue);
    Vec3 exp;
    if (fabs(angle) < 1e-5){
        exp=Vec3(0,0,0);
    }
    else if (fabs(angle - M_PI) < 1e-5){
        exp = angle * Vec3(rm(0,0)+1, rm(0,1), rm(0,2)) / sqrt(2*(1+rm(0, 0)));
    }
    else{
        exp=angle/(2.0f*sin(angle))*Vec3(rm(2,1)-rm(1,2),rm(0,2)-rm(2,0),rm(1,0)-rm(0,1));
    }
    return exp;
}

inline HomoMat homoMatrix(Vec3 p, RotMat m){
    HomoMat homoM;
    homoM.setZero();
    homoM.topLeftCorner(3, 3) = m;
    homoM.topRightCorner(3, 1) = p;
    homoM(3, 3) = 1;
    return homoM;
}

inline HomoMat homoMatrix(Vec3 p, Quat q){
    HomoMat homoM;
    homoM.setZero();
    homoM.topLeftCorner(3, 3) = quatToRotMat(q);
    homoM.topRightCorner(3, 1) = p;
    homoM(3, 3) = 1;
    return homoM;
}

inline HomoMat homoMatrixInverse(HomoMat homoM){
    HomoMat homoInv;
    homoInv.setZero();
    homoInv.topLeftCorner(3, 3) = homoM.topLeftCorner(3, 3).transpose();
    homoInv.topRightCorner(3, 1) = -homoM.topLeftCorner(3, 3).transpose() * homoM.topRightCorner(3, 1);
    homoInv(3, 3) = 1;
    return homoInv;
}

//  add 1 at the end of Vec3
inline Vec4 homoVec(Vec3 v3){
    Vec4 v4;
    v4.block(0, 0, 3, 1) = v3;
    v4(3) = 1;
    return v4;
}

//  remove 1 at the end of Vec4
inline Vec3 noHomoVec(Vec4 v4){
    Vec3 v3;
    v3 = v4.block(0, 0, 3, 1);
    return v3;
}

// Calculate average value and covariance
class AvgCov{
public:
    AvgCov(unsigned int size, std::string name, bool avgOnly=false, unsigned int showPeriod=1000, unsigned int waitCount=5000, double zoomFactor=10000)
            :_size(size), _showPeriod(showPeriod), _waitCount(waitCount), _zoomFactor(zoomFactor), _valueName(name), _avgOnly(avgOnly) {
        _exp.resize(size);
        _cov.resize(size, size);
        _defaultWeight.resize(size, size);
        _defaultWeight.setIdentity();
        _measureCount = 0;
    }
    void measure(VecX newValue){
        ++_measureCount;

        if(_measureCount > _waitCount){
            updateAvgCov(_cov, _exp, newValue, _measureCount-_waitCount);
            if(_measureCount % _showPeriod == 0){
                std::cout << "******" << _valueName << " measured count: " << _measureCount-_waitCount << "******" << std::endl;
                std::cout << _zoomFactor << " Times Average of " << _valueName << std::endl << (_zoomFactor*_exp).transpose() << std::endl;
                if(!_avgOnly){
                    std::cout << _zoomFactor << " Times Covariance of " << _valueName << std::endl << _zoomFactor*_cov << std::endl;
                }
            }
        }
    }
private:
    VecX _exp;
    MatX _cov;
    MatX _defaultWeight;
    bool _avgOnly;
    unsigned int _size;
    unsigned int _measureCount;
    unsigned int _showPeriod;
    unsigned int _waitCount;
    double _zoomFactor;
    std::string _valueName;
};

inline std::vector<float> QuatRotateInverse(const std::vector<float> &q, const std::vector<float> &v)
{
    float q_w = q[0];
    Eigen::Vector3f v_vec = Eigen::Vector3f(v[0], v[1], v[2]);
    Eigen::Vector3f q_vec = Eigen::Vector3f(q[1], q[2], q[3]);
    Eigen::Vector3f a = v_vec * (2.0f * q_w * q_w - 1.0f);
    Eigen::Vector3f b = q_vec.cross(v_vec) * q_w * 2.0f;
    Eigen::Vector3f c = q_vec * (q_vec.dot(v_vec) * 2.0f);
    Eigen::Vector3f projected_gravity_vec = (a - b + c).eval();
    std::vector<float> projected_gravity(projected_gravity_vec.data(), projected_gravity_vec.data() + 3);
    return projected_gravity;
}

/**
 * @brief 将四元数转换为旋转矩阵
 * 
 * @param quaternion 四元数 (w, x, y, z) 格式
 * @return Eigen::Matrix3f 3x3 旋转矩阵
 * 
 */
inline Eigen::Matrix3f matrix_from_quat(const std::vector<float> &quaternion)
{
    if (quaternion.size() != 4) {
        throw std::invalid_argument("Quaternion must have 4 elements (w, x, y, z)");
    }

    float w = quaternion[0];  // r in PyTorch version
    float x = quaternion[1];  // i in PyTorch version
    float y = quaternion[2];  // j in PyTorch version
    float z = quaternion[3];  // k in PyTorch version

    // 计算四元数的平方和
    float quat_norm_sq = w * w + x * x + y * y + z * z;
    float two_s = 2.0f / quat_norm_sq;

    // 构建旋转矩阵的9个元素
    float m00 = 1.0f - two_s * (y * y + z * z);
    float m01 = two_s * (x * y - z * w);
    float m02 = two_s * (x * z + y * w);
    
    float m10 = two_s * (x * y + z * w);
    float m11 = 1.0f - two_s * (x * x + z * z);
    float m12 = two_s * (y * z - x * w);
    
    float m20 = two_s * (x * z - y * w);
    float m21 = two_s * (y * z + x * w);
    float m22 = 1.0f - two_s * (x * x + y * y);

    // 构建 Eigen 矩阵
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix << m00, m01, m02,
                       m10, m11, m12,
                       m20, m21, m22;

    return rotation_matrix;
}

/**
 * @brief 将四元数转换为旋转矩阵 (Eigen::Vector4f 版本)
 * 
 * @param quaternion 四元数 (w, x, y, z) 格式
 * @return Eigen::Matrix3f 3x3 旋转矩阵
 */
inline Eigen::Matrix3f matrix_from_quat(const Eigen::Vector4f &quaternion)
{
    float w = quaternion(0);
    float x = quaternion(1);
    float y = quaternion(2);
    float z = quaternion(3);

    float quat_norm_sq = quaternion.squaredNorm();
    float two_s = 2.0f / quat_norm_sq;

    float m00 = 1.0f - two_s * (y * y + z * z);
    float m01 = two_s * (x * y - z * w);
    float m02 = two_s * (x * z + y * w);
    
    float m10 = two_s * (x * y + z * w);
    float m11 = 1.0f - two_s * (x * x + z * z);
    float m12 = two_s * (y * z - x * w);
    
    float m20 = two_s * (x * z - y * w);
    float m21 = two_s * (y * z + x * w);
    float m22 = 1.0f - two_s * (x * x + y * y);

    Eigen::Matrix3f rotation_matrix;
    rotation_matrix << m00, m01, m02,
                       m10, m11, m12,
                       m20, m21, m22;

    return rotation_matrix;
}

/**
 * @brief 计算四元数的共轭
 * 
 * @param q 四元数 (w, x, y, z) 格式
 * @return std::vector<float> 共轭四元数 (w, -x, -y, -z)
 */
inline std::vector<float> quat_conjugate(const std::vector<float> &q)
{
    if (q.size() != 4) {
        throw std::invalid_argument("Quaternion must have 4 elements (w, x, y, z)");
    }
    return {q[0], -q[1], -q[2], -q[3]};
}

/**
 * @brief 计算四元数的共轭 (Eigen::Vector4f 版本)
 * 
 * @param q 四元数 (w, x, y, z) 格式
 * @return Eigen::Vector4f 共轭四元数 (w, -x, -y, -z)
 */
inline Eigen::Vector4f quat_conjugate(const Eigen::Vector4f &q)
{
    return Eigen::Vector4f(q(0), -q(1), -q(2), -q(3));
}

/**
 * @brief 绕X轴旋转的旋转矩阵
 * 
 * @param angle 旋转角度（弧度）
 * @return Eigen::Matrix3f 3x3 旋转矩阵
 */
inline Eigen::Matrix3f rotx(float angle)
{
    float c = std::cos(angle);
    float s = std::sin(angle);
    Eigen::Matrix3f R;
    R << 1,  0,  0,
         0,  c, -s,
         0,  s,  c;
    return R;
}

/**
 * @brief 绕Y轴旋转的旋转矩阵
 * 
 * @param angle 旋转角度（弧度）
 * @return Eigen::Matrix3f 3x3 旋转矩阵
 */
inline Eigen::Matrix3f roty(float angle)
{
    float c = std::cos(angle);
    float s = std::sin(angle);
    Eigen::Matrix3f R;
    R <<  c,  0,  s,
          0,  1,  0,
         -s,  0,  c;
    return R;
}

/**
 * @brief 绕Z轴旋转的旋转矩阵
 * 
 * @param angle 旋转角度（弧度）
 * @return Eigen::Matrix3f 3x3 旋转矩阵
 */
inline Eigen::Matrix3f rotz(float angle)
{
    float c = std::cos(angle);
    float s = std::sin(angle);
    Eigen::Matrix3f R;
    R << c, -s,  0,
         s,  c,  0,
         0,  0,  1;
    return R;
}

/**
 * @brief 将旋转矩阵转换为四元数
 * 
 * @param R 3x3 旋转矩阵
 * @return std::vector<float> 四元数 (w, x, y, z) 格式
 */
inline std::vector<float> quat_from_matrix(const Eigen::Matrix3f &R)
{
    float trace = R.trace();
    std::vector<float> q(4);
    
    if (trace > 0) {
        float s = 0.5f / std::sqrt(trace + 1.0f);
        q[0] = 0.25f / s;  // w
        q[1] = (R(2,1) - R(1,2)) * s;  // x
        q[2] = (R(0,2) - R(2,0)) * s;  // y
        q[3] = (R(1,0) - R(0,1)) * s;  // z
    } else if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
        float s = 2.0f * std::sqrt(1.0f + R(0,0) - R(1,1) - R(2,2));
        q[0] = (R(2,1) - R(1,2)) / s;  // w
        q[1] = 0.25f * s;  // x
        q[2] = (R(0,1) + R(1,0)) / s;  // y
        q[3] = (R(0,2) + R(2,0)) / s;  // z
    } else if (R(1,1) > R(2,2)) {
        float s = 2.0f * std::sqrt(1.0f + R(1,1) - R(0,0) - R(2,2));
        q[0] = (R(0,2) - R(2,0)) / s;  // w
        q[1] = (R(0,1) + R(1,0)) / s;  // x
        q[2] = 0.25f * s;  // y
        q[3] = (R(1,2) + R(2,1)) / s;  // z
    } else {
        float s = 2.0f * std::sqrt(1.0f + R(2,2) - R(0,0) - R(1,1));
        q[0] = (R(1,0) - R(0,1)) / s;  // w
        q[1] = (R(0,2) + R(2,0)) / s;  // x
        q[2] = (R(1,2) + R(2,1)) / s;  // y
        q[3] = 0.25f * s;  // z
    }
    
    // 归一化
    float norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (int i = 0; i < 4; i++) {
        q[i] /= norm;
    }
    
    return q;
}

/**
 * @brief 从四元数提取欧拉角 (roll, pitch, yaw)
 * 
 * @param q 四元数 (w, x, y, z) 格式
 * @return std::vector<float> 欧拉角 [roll, pitch, yaw] (弧度)
 */
inline std::vector<float> quat_to_euler(const std::vector<float> &q)
{
    if (q.size() != 4) {
        throw std::invalid_argument("Quaternion must have 4 elements (w, x, y, z)");
    }
    
    float w = q[0], x = q[1], y = q[2], z = q[3];
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    float roll = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (w * y - z * x);
    float pitch;
    if (std::abs(sinp) >= 1.0f)
        pitch = std::copysign(M_PI / 2.0f, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    float yaw = std::atan2(siny_cosp, cosy_cosp);
    
    return {roll, pitch, yaw};
}

/**
 * @brief 从欧拉角构建四元数
 * 
 * @param roll 绕x轴旋转角度 (弧度)
 * @param pitch 绕y轴旋转角度 (弧度)
 * @param yaw 绕z轴旋转角度 (弧度)
 * @return std::vector<float> 四元数 (w, x, y, z) 格式
 */
inline std::vector<float> euler_to_quat(float roll, float pitch, float yaw)
{
    float cy = std::cos(yaw * 0.5f);
    float sy = std::sin(yaw * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);
    
    std::vector<float> q(4);
    q[0] = cr * cp * cy + sr * sp * sy;  // w
    q[1] = sr * cp * cy - cr * sp * sy;  // x
    q[2] = cr * sp * cy + sr * cp * sy;  // y
    q[3] = cr * cp * sy - sr * sp * cy;  // z
    
    return q;
}

/**
 * @brief 提取四元数的yaw分量
 * 
 * @param quat 四元数 (w, x, y, z) 格式
 * @return std::vector<float> 只包含yaw旋转的四元数 (w, 0, 0, z)
 */
inline std::vector<float> yaw_quat(const std::vector<float> &quat)
{
    if (quat.size() != 4) {
        throw std::invalid_argument("Quaternion must have 4 elements (w, x, y, z)");
    }
    
    float qw = quat[0];
    float qx = quat[1];
    float qy = quat[2];
    float qz = quat[3];
    
    // 计算yaw角
    float yaw = std::atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
    
    // 构建只包含yaw的四元数
    std::vector<float> quat_yaw(4);
    quat_yaw[0] = std::cos(yaw / 2.0f);  // w
    quat_yaw[1] = 0.0f;                   // x
    quat_yaw[2] = 0.0f;                   // y
    quat_yaw[3] = std::sin(yaw / 2.0f);  // z
    
    // 归一化
    float norm = std::sqrt(quat_yaw[0] * quat_yaw[0] + quat_yaw[3] * quat_yaw[3]);
    quat_yaw[0] /= norm;
    quat_yaw[3] /= norm;
    
    return quat_yaw;
}

/**
 * @brief 四元数乘法
 * 
 * @param q1 第一个四元数 (w, x, y, z) 格式
 * @param q2 第二个四元数 (w, x, y, z) 格式
 * @return std::vector<float> 结果四元数 q1 * q2
 */
inline std::vector<float> quat_multiply(const std::vector<float> &q1, const std::vector<float> &q2)
{
    if (q1.size() != 4 || q2.size() != 4) {
        throw std::invalid_argument("Quaternions must have 4 elements (w, x, y, z)");
    }
    
    float w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    float w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
    
    std::vector<float> result(4);
    result[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;  // w
    result[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;  // x
    result[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;  // y
    result[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;  // z
    
    return result;
}

/**
 * @brief 计算四元数的逆
 * 
 * @param q 四元数 (w, x, y, z) 格式
 * @param eps 避免除零的小值，默认 1e-9
 * @return std::vector<float> 逆四元数
 * 
 * 公式: q_inv = conjugate(q) / ||q||^2
 */
inline std::vector<float> quat_inv(const std::vector<float> &q, float eps = 1e-9f)
{
    if (q.size() != 4) {
        throw std::invalid_argument("Quaternion must have 4 elements (w, x, y, z)");
    }
    
    // 计算四元数模的平方: w^2 + x^2 + y^2 + z^2
    float norm_squared = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    norm_squared = std::max(norm_squared, eps);  // 避免除零
    
    // 计算共轭
    std::vector<float> q_conj = quat_conjugate(q);
    
    // 除以模的平方
    return {
        q_conj[0] / norm_squared,
        q_conj[1] / norm_squared,
        q_conj[2] / norm_squared,
        q_conj[3] / norm_squared
    };
}

/**
 * @brief 计算四元数的逆 (Eigen::Vector4f 版本)
 * 
 * @param q 四元数 (w, x, y, z) 格式
 * @param eps 避免除零的小值，默认 1e-9
 * @return Eigen::Vector4f 逆四元数
 */
inline Eigen::Vector4f quat_inv(const Eigen::Vector4f &q, float eps = 1e-9f)
{
    float norm_squared = q.squaredNorm();
    norm_squared = std::max(norm_squared, eps);
    
    Eigen::Vector4f q_conj = quat_conjugate(q);
    
    return q_conj / norm_squared;
}

/**
 * @brief 两个四元数相乘
 * 
 * @param q1 第一个四元数 (w, x, y, z) 格式
 * @param q2 第二个四元数 (w, x, y, z) 格式
 * @return std::vector<float> 乘积四元数 (w, x, y, z)
 * 
 * 使用优化算法减少乘法次数
 */
inline std::vector<float> quat_mul(const std::vector<float> &q1, const std::vector<float> &q2)
{
    if (q1.size() != 4 || q2.size() != 4) {
        throw std::invalid_argument("Both quaternions must have 4 elements (w, x, y, z)");
    }
    
    // 提取四元数分量
    float w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    float w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
    
    // 使用优化的乘法算法
    float ww = (z1 + x1) * (x2 + y2);
    float yy = (w1 - y1) * (w2 + z2);
    float zz = (w1 + y1) * (w2 - z2);
    float xx = ww + yy + zz;
    float qq = 0.5f * (xx + (z1 - x1) * (x2 - y2));
    
    float w = qq - ww + (z1 - y1) * (y2 - z2);
    float x = qq - xx + (x1 + w1) * (x2 + w2);
    float y = qq - yy + (w1 - x1) * (y2 + z2);
    float z = qq - zz + (z1 + y1) * (w2 - x2);
    
    return {w, x, y, z};
}

/**
 * @brief 两个四元数相乘 (Eigen::Vector4f 版本)
 * 
 * @param q1 第一个四元数 (w, x, y, z) 格式
 * @param q2 第二个四元数 (w, x, y, z) 格式
 * @return Eigen::Vector4f 乘积四元数 (w, x, y, z)
 */
inline Eigen::Vector4f quat_mul(const Eigen::Vector4f &q1, const Eigen::Vector4f &q2)
{
    float w1 = q1(0), x1 = q1(1), y1 = q1(2), z1 = q1(3);
    float w2 = q2(0), x2 = q2(1), y2 = q2(2), z2 = q2(3);
    
    float ww = (z1 + x1) * (x2 + y2);
    float yy = (w1 - y1) * (w2 + z2);
    float zz = (w1 + y1) * (w2 - z2);
    float xx = ww + yy + zz;
    float qq = 0.5f * (xx + (z1 - x1) * (x2 - y2));
    
    float w = qq - ww + (z1 - y1) * (y2 - z2);
    float x = qq - xx + (x1 + w1) * (x2 + w2);
    float y = qq - yy + (w1 - x1) * (y2 + z2);
    float z = qq - zz + (z1 + y1) * (w2 - x2);
    
    return Eigen::Vector4f(w, x, y, z);
}

/**
 * @brief 使用四元数对向量进行旋转
 * 
 * @param quat 四元数 (w, x, y, z) 格式
 * @param vec 三维向量 (x, y, z)
 * @return std::vector<float> 旋转后的向量 (x, y, z)
 * 
 * 公式: v' = v + 2w(q_xyz × v) + 2(q_xyz × (q_xyz × v))
 */
inline std::vector<float> quat_apply(const std::vector<float> &quat, const std::vector<float> &vec)
{
    if (quat.size() != 4) {
        throw std::invalid_argument("Quaternion must have 4 elements (w, x, y, z)");
    }
    if (vec.size() != 3) {
        throw std::invalid_argument("Vector must have 3 elements (x, y, z)");
    }
    
    float w = quat[0];
    float qx = quat[1], qy = quat[2], qz = quat[3];
    float vx = vec[0], vy = vec[1], vz = vec[2];
    
    // t = xyz × vec × 2
    float tx = 2.0f * (qy * vz - qz * vy);
    float ty = 2.0f * (qz * vx - qx * vz);
    float tz = 2.0f * (qx * vy - qy * vx);
    
    // xyz × t
    float cross_x = qy * tz - qz * ty;
    float cross_y = qz * tx - qx * tz;
    float cross_z = qx * ty - qy * tx;
    
    // vec + w * t + xyz × t
    return {
        vx + w * tx + cross_x,
        vy + w * ty + cross_y,
        vz + w * tz + cross_z
    };
}

/**
 * @brief 使用四元数对向量进行旋转 (Eigen 版本)
 * 
 * @param quat 四元数 (w, x, y, z) 格式
 * @param vec 三维向量 (x, y, z)
 * @return Eigen::Vector3f 旋转后的向量 (x, y, z)
 */
inline Eigen::Vector3f quat_apply(const Eigen::Vector4f &quat, const Eigen::Vector3f &vec)
{
    float w = quat(0);
    Eigen::Vector3f xyz(quat(1), quat(2), quat(3));
    
    Eigen::Vector3f t = xyz.cross(vec) * 2.0f;
    
    return vec + w * t + xyz.cross(t);
}

/**
 * @brief 使用四元数对向量进行旋转 (std::vector 四元数 + Eigen::Vector3f 向量)
 * 
 * @param quat 四元数 (w, x, y, z) 格式
 * @param vec 三维向量
 * @return Eigen::Vector3f 旋转后的向量
 */
inline Eigen::Vector3f quat_apply(const std::vector<float> &quat, const Eigen::Vector3f &vec)
{
    if (quat.size() != 4) {
        throw std::invalid_argument("Quaternion must have 4 elements (w, x, y, z)");
    }
    
    Eigen::Vector4f q(quat[0], quat[1], quat[2], quat[3]);
    return quat_apply(q, vec);
}

/**
 * @brief 计算两个参考坐标系之间的相对变换
 * 
 * 执行变换操作: T_12 = T_01^(-1) × T_02
 * 其中 T_AB 是从坐标系 A 到坐标系 B 的齐次变换矩阵
 * 
 * @param t01 坐标系1相对于坐标系0的位置 (x, y, z)
 * @param q01 坐标系1相对于坐标系0的四元数 (w, x, y, z)
 * @param t02 坐标系2相对于坐标系0的位置 (x, y, z)，nullptr 表示零位置
 * @param q02 坐标系2相对于坐标系0的四元数 (w, x, y, z)，nullptr 表示单位四元数
 * @return std::pair<std::vector<float>, std::vector<float>> 坐标系2相对于坐标系1的位置和四元数
 */
inline std::pair<std::vector<float>, std::vector<float>> subtract_frame_transforms(
    const std::vector<float> &t01,
    const std::vector<float> &q01,
    const std::vector<float> *t02 = nullptr,
    const std::vector<float> *q02 = nullptr)
{
    if (t01.size() != 3 || q01.size() != 4) {
        throw std::invalid_argument("t01 must be size 3, q01 must be size 4");
    }
    if (t02 && t02->size() != 3) {
        throw std::invalid_argument("t02 must be size 3 if provided");
    }
    if (q02 && q02->size() != 4) {
        throw std::invalid_argument("q02 must be size 4 if provided");
    }
    
    // 计算方向: q10 = quat_inv(q01)
    std::vector<float> q10 = quat_inv(q01);
    
    // 计算 q12
    std::vector<float> q12;
    if (q02 != nullptr) {
        q12 = quat_mul(q10, *q02);
    } else {
        q12 = q10;
    }
    
    // 计算平移
    std::vector<float> t12;
    if (t02 != nullptr) {
        // t12 = quat_apply(q10, t02 - t01)
        std::vector<float> diff = {
            (*t02)[0] - t01[0],
            (*t02)[1] - t01[1],
            (*t02)[2] - t01[2]
        };
        t12 = quat_apply(q10, diff);
    } else {
        // t12 = quat_apply(q10, -t01)
        std::vector<float> neg_t01 = {-t01[0], -t01[1], -t01[2]};
        t12 = quat_apply(q10, neg_t01);
    }
    
    return {t12, q12};
}

/**
 * @brief 计算两个参考坐标系之间的相对变换 (Eigen 版本)
 * 
 * @param t01 坐标系1相对于坐标系0的位置
 * @param q01 坐标系1相对于坐标系0的四元数 (w, x, y, z)
 * @param t02 坐标系2相对于坐标系0的位置，nullptr 表示零位置
 * @param q02 坐标系2相对于坐标系0的四元数，nullptr 表示单位四元数
 * @return std::pair<Eigen::Vector3f, Eigen::Vector4f> 坐标系2相对于坐标系1的位置和四元数
 */
inline std::pair<Eigen::Vector3f, Eigen::Vector4f> subtract_frame_transforms(
    const Eigen::Vector3f &t01,
    const Eigen::Vector4f &q01,
    const Eigen::Vector3f *t02 = nullptr,
    const Eigen::Vector4f *q02 = nullptr)
{
    // 计算方向
    Eigen::Vector4f q10 = quat_inv(q01);
    
    Eigen::Vector4f q12;
    if (q02 != nullptr) {
        q12 = quat_mul(q10, *q02);
    } else {
        q12 = q10;
    }
    
    // 计算平移
    Eigen::Vector3f t12;
    if (t02 != nullptr) {
        t12 = quat_apply(q10, *t02 - t01);
    } else {
        t12 = quat_apply(q10, -t01);
    }
    
    return {t12, q12};
}

// 四元数球面线性插值 (SLERP)
// q1, q2: 输入四元数 (w,x,y,z) 格式
// t: 插值参数 [0, 1]
// 返回: 插值后的四元数 (w,x,y,z)
inline std::vector<float> quat_slerp(const std::vector<float> &q1, const std::vector<float> &q2, float t)
{
    // 计算点积
    float dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
    
    // 如果点积为负，取反其中一个四元数以获得最短路径
    std::vector<float> q2_adjusted = q2;
    if (dot < 0.0f) {
        q2_adjusted[0] = -q2[0];
        q2_adjusted[1] = -q2[1];
        q2_adjusted[2] = -q2[2];
        q2_adjusted[3] = -q2[3];
        dot = -dot;
    }
    
    const float DOT_THRESHOLD = 0.9995f;
    std::vector<float> result(4);
    
    if (dot > DOT_THRESHOLD) {
        // 四元数非常接近，使用线性插值
        result[0] = q1[0] + t * (q2_adjusted[0] - q1[0]);
        result[1] = q1[1] + t * (q2_adjusted[1] - q1[1]);
        result[2] = q1[2] + t * (q2_adjusted[2] - q1[2]);
        result[3] = q1[3] + t * (q2_adjusted[3] - q1[3]);
        
        // 归一化
        float norm = std::sqrt(result[0] * result[0] + result[1] * result[1] + 
                              result[2] * result[2] + result[3] * result[3]);
        result[0] /= norm;
        result[1] /= norm;
        result[2] /= norm;
        result[3] /= norm;
    } else {
        // 使用球面线性插值
        float theta_0 = std::acos(dot);  // 四元数之间的角度
        float theta = theta_0 * t;       // 插值角度
        float sin_theta = std::sin(theta);
        float sin_theta_0 = std::sin(theta_0);
        
        float s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
        float s1 = sin_theta / sin_theta_0;
        
        result[0] = s0 * q1[0] + s1 * q2_adjusted[0];
        result[1] = s0 * q1[1] + s1 * q2_adjusted[1];
        result[2] = s0 * q1[2] + s1 * q2_adjusted[2];
        result[3] = s0 * q1[3] + s1 * q2_adjusted[3];
    }
    
    return result;
}

// 应用四元数的逆旋转到向量
// quat: 四元数 (w, x, y, z) 格式
// vec: 向量 (x, y, z) 格式
// 返回: 旋转后的向量 (x, y, z)
inline std::vector<float> quat_apply_inverse(const std::vector<float> &quat, const std::vector<float> &vec)
{
    // 提取四元数的虚部 (x, y, z)
    std::vector<float> xyz = {quat[1], quat[2], quat[3]};
    
    // 计算 t = xyz.cross(vec) * 2
    std::vector<float> cross1(3);
    cross1[0] = xyz[1] * vec[2] - xyz[2] * vec[1];
    cross1[1] = xyz[2] * vec[0] - xyz[0] * vec[2];
    cross1[2] = xyz[0] * vec[1] - xyz[1] * vec[0];
    
    std::vector<float> t(3);
    t[0] = cross1[0] * 2.0f;
    t[1] = cross1[1] * 2.0f;
    t[2] = cross1[2] * 2.0f;
    
    // 计算 xyz.cross(t)
    std::vector<float> cross2(3);
    cross2[0] = xyz[1] * t[2] - xyz[2] * t[1];
    cross2[1] = xyz[2] * t[0] - xyz[0] * t[2];
    cross2[2] = xyz[0] * t[1] - xyz[1] * t[0];
    
    // 计算最终结果: vec - quat_w * t + xyz.cross(t)
    std::vector<float> result(3);
    result[0] = vec[0] - quat[0] * t[0] + cross2[0];
    result[1] = vec[1] - quat[0] * t[1] + cross2[1];
    result[2] = vec[2] - quat[0] * t[2] + cross2[2];
    
    return result;
}

#endif  // MATHTOOLS_H