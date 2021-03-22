//
// Created by vlad on 22.03.2021.
//

#ifndef LAB_WS_UTILS_H
#define LAB_WS_UTILS_H

double yaw(geometry_msgs::Quaternion q){
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

#endif //LAB_WS_UTILS_H
