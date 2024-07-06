/*
 * @file JointSwingTrajectory.cpp
 * @brief 
 */

#include "Controllers/JointSwingTrajectory.h"

template<typename T>
Vec3<T> JointSwingTrajectory<T>::inverseKinematics(Vec3<T> pDes, int leg) {
    T x = std::abs(pDes(0));
    T y = std::abs(pDes(1));
    T z = std::abs(pDes(2));

    T h = 0.062;
    T hu = 0.209;
    T hl = 0.195;

    T dyz = std::sqrt(y * y + z * z);
    T lyz = std::sqrt(dyz * dyz - h * h);

    T gamma_yz = -std::atan2(y, z);
    if (gamma_yz < -M_PI / 2) {
        gamma_yz += M_PI;
    } else if (gamma_yz > M_PI / 2) {
        gamma_yz -= M_PI;
    }
    T gamma_h_offset = -std::atan2(h, lyz);
    if (gamma_h_offset < -M_PI / 2) {
        gamma_h_offset += M_PI;
    } else if (gamma_h_offset > M_PI / 2) {
        gamma_h_offset -= M_PI;
    }
    T gamma = gamma_yz - gamma_h_offset;

    T lxzp = std::sqrt(lyz * lyz + x * x);
    T n = (lxzp * lxzp - hl * hl - hu * hu) / (2 * hu);
    T beta = -std::acos(n / hl);

    T alfa_xzp = -std::atan2(x, lyz);
    if (alfa_xzp < -M_PI / 2) {
        alfa_xzp += M_PI;
    } else if (alfa_xzp > M_PI / 2) {
        alfa_xzp -= M_PI;
    }
    T alfa_off = std::acos((hu + n) / lxzp);
    T alfa = alfa_xzp + alfa_off;

    Vec3<T> output;
    const float sideSigns[4] = {-1, 1, -1, 1};
    output(0) = -gamma * sideSigns[leg];
    output(1) = -alfa;
    output(2) = -beta;

    return output;
}

template<typename T>
void JointSwingTrajectory<T>::computeSwingTrajectoryBezier(T phase, T swingTime) {
    if (phase < T(0.5)) {
        _q = Interpolate::cubicBezier<Vec3<T>>(_q0, _qm, phase * 2);
        _qd = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_q0, _qm, phase * 2) * 2 / swingTime;
        _qdd = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_q0, _qm, phase * 2) * 4 / (swingTime * swingTime);
    } else {
        _q = Interpolate::cubicBezier<Vec3<T>>(_qm, _qf, phase * 2 - 1);
        _qd = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_qm, _qf, phase * 2 - 1) * 2 / swingTime;
        _qdd = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_qm, _qf, phase * 2 - 1) * 4 / (swingTime * swingTime);
    }
}

template
class JointSwingTrajectory<float>;

template
class JointSwingTrajectory<double>;