//
// Created by mebius on 24-5-12.
//

#include "FSM_State_SitDown.h"
#include "Math/Interpolation.h"

template<typename T>
FSM_State_SitDown<T>::FSM_State_SitDown(ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::SIT_DOWN, "SIT_DOWN"), _ini_joint_pos(4),
          _end_joint_pos(4) {
    //  回复状态是确定的,不需要安全检查
    this->turnOffAllSafetyChecks();
}

template<typename T>
void FSM_State_SitDown<T>::onEnter() {
    printf("[FSM_State_SitDown] onEnter...\n");

    //  重置迭代器
    iter = 0;

//    float theta_abad = this->_delta_abad;
//    float theta_hip = this->_delta_hip;
//    float theta_knee = this->_delta_knee;

    for (size_t leg(0); leg < 4; ++leg) {
        _ini_joint_pos[leg] = this->_data->_legController->datas[leg].q;
//        _ini_joint_pos[leg][0] = this->_delta_abad;
//        _ini_joint_pos[leg][1] = this->_delta_hip;
//        _ini_joint_pos[leg][2] = this->_delta_knee;

        _end_joint_pos[leg][0] = 0.f;    // abad
        _end_joint_pos[leg][1] = 0.f;    // hip
        _end_joint_pos[leg][2] = 0.f;    // knee
    }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template<typename T>
void FSM_State_SitDown<T>::run() {
    //  迭代器，进入状态后迭代了多少个dt周期
    iter++;
    T recovery_passive_time = this->_data->userParameters->stand_up_time;
    T t = (iter * this->_data->controlParameters->controller_dt) / recovery_passive_time;

    if (t > 1.) { t = 1.; }

    Vec3<T> kp(0.8, 0.6, 1.8);
    Vec3<T> kd(0.008, 0.003, 0.02);

    for (size_t leg(0); leg < 4; ++leg) {
        this->_data->_legController->commands[leg].kpJoint = kp.asDiagonal();
        this->_data->_legController->commands[leg].kdJoint = kd.asDiagonal();

        this->_data->_legController->commands[leg].qDes
                = Interpolate::cubicBezier<Vec3<T>>(_ini_joint_pos[leg], _end_joint_pos[leg], t);
        this->_data->_legController->commands[leg].qdDes
                = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_ini_joint_pos[leg], _end_joint_pos[leg], t) /
                  recovery_passive_time;
    }
}

/**
 * 判断动作是否完成,Busy状态下不允许切换状态
 */
template<typename T>
bool FSM_State_SitDown<T>::isBusy() {
    T t = (iter * this->_data->controlParameters->controller_dt) /
          (this->_data->userParameters->stand_up_time);

    if (t < 1.0f)
        return true;
    else
        return false;
}

// template class FSM_State_StandUp<double>;
template
class FSM_State_SitDown<float>;