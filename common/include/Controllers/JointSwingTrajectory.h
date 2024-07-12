/*
 * @file JointSwingTrajectory.h
 * @brief 
 */

#ifndef CHEETAH_SOFTWARE_JOINTSWINGTRAJECTORY_H
#define CHEETAH_SOFTWARE_JOINTSWINGTRAJECTORY_H

#include "cppTypes.h"
#include "Math/Interpolation.h"

template<typename T>
class JointSwingTrajectory {
public:
    JointSwingTrajectory(){
        _p0.setZero();
        _pf.setZero();
        _q0.setZero();
        _qf.setZero();
        _qm.setZero();
        _q.setZero();
        _qd.setZero();
        _qdd.setZero();
    }

    static Vec3<T> inverseKinematics(Vec3<T> pDes, int leg);

    void computeSwingTrajectoryBezier(T phase, T swingTime);

    void computeSwingTrajectoryBezierWithoutMid(T phase, T swingTime);

    void setInitialJointPosition(Vec3<T> p0, int leg) {
        _p0 = p0;
        _q0 = inverseKinematics(p0, leg);
    }

    void setFinalJointPosition(Vec3<T> pf, int leg) {
        _pf = pf;
        _qf = inverseKinematics(pf, leg);
    }

    void setMiddleJointPosition(Vec3<T> pm, int leg) {
        _qm = inverseKinematics(pm, leg);
    }

    void setHeight(T h, int leg) {
        T _pmx = (_p0(0) + _pf(0)) / 2;
        T _pmy = (_p0(1) + _pf(1)) / 2;
        _qm = inverseKinematics(Vec3<T>(_pmx, _pmy, h), leg);
    }

    Vec3<T> getJointPosition() {
        return _q;
    }

    Vec3<T> getJointVelocity() {
        return _qd;
    }

    Vec3<T> getJointAcceleration() {
        return _qdd;
    }

private:
    Vec3<T> _p0;
    Vec3<T> _pf;
    Vec3<T> _q0;
    Vec3<T> _qf;
    Vec3<T> _qm;

    Vec3<T> _q;
    Vec3<T> _qd;
    Vec3<T> _qdd;

};

#endif //CHEETAH_SOFTWARE_JOINTSWINGTRAJECTORY_H
