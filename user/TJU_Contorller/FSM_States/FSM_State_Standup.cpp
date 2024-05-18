/*============================= Stand Up ==============================*/
/** 执行之后，机器人保持站立状态。
 */
#include "FSM_State_StandUp.h"
#include "Math/Interpolation.h"

#define PI (3.141592654f)

/**
 * 构造函数
 *
 * @param _controlFSMData 这里面保存了相关参数
 */
template<typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"),
          _ini_joint_pos(4), _end_joint_pos(4) {
    //站立状态是确定的,不需要安全检查
    this->turnOffAllSafetyChecks();
}

template<typename T>
void FSM_State_StandUp<T>::onEnter() {

    printf("[FSM_State_StandUp] onEnter...\n");
    // Reset iteration counter
    iter = 0;

//    float l1 = this->_data->_quadruped->_hipLinkLength; // 0.209
//    float l2 = this->_data->_quadruped->_kneeLinkLength; // 0.195
//    float h  = this->_data->userParameters->stand_up_height;   // 0.31
//
//    float theta1 = -acosf((l1*l1+h*h-l2*l2)/(2*l1*h));  // -0.667655
//    float theta2 = PI - acosf((l1*l1+l2*l2-h*h)/(2*l1*l2)); // 1.393274
//    printf("theta1: %f, theta2: %f\n", theta1, theta2);

    float theta_abad = this->_delta_abad;
    float theta_hip = this->_delta_hip;
    float theta_knee = this->_delta_knee;

    for (size_t leg(0); leg < 4; ++leg) {
        _ini_joint_pos[leg] = this->_data->_legController->datas[leg].q;

        _end_joint_pos[leg][0] = theta_abad; // abad
        _end_joint_pos[leg][1] = theta_hip;    // hip
        _end_joint_pos[leg][2] = theta_knee;    // knee
    }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template<typename T>
void FSM_State_StandUp<T>::run() {

    iter++;
    T stand_up_time = this->_data->userParameters->stand_up_time;
    T t = (iter * this->_data->controlParameters->controller_dt) / stand_up_time;

    if (t > 1.) { t = 1.; }

    Vec3<T> kp(this->_data->userParameters->Kp_stand[0],
               this->_data->userParameters->Kp_stand[1],
               this->_data->userParameters->Kp_stand[2]);
    Vec3<T> kd(this->_data->userParameters->Kd_stand[0],
               this->_data->userParameters->Kd_stand[1],
               this->_data->userParameters->Kd_stand[2]);
    for (int leg = 0; leg < 4; leg++) {
        this->_data->_legController->commands[leg].kpJoint = kp.asDiagonal();
        this->_data->_legController->commands[leg].kdJoint = kd.asDiagonal();

        this->_data->_legController->commands[leg].qDes
                = Interpolate::cubicBezier<Vec3<T>>(_ini_joint_pos[leg], _end_joint_pos[leg], t);
        this->_data->_legController->commands[leg].qdDes
                = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_ini_joint_pos[leg], _end_joint_pos[leg], t) /
                  stand_up_time;
//        printf("leg: %d, abad_qDes: %f\n", leg, this->_data->_legController->commands[leg].qDes[0]);
//        printf("leg: %d, hip_qDes: %f\n", leg, this->_data->_legController->commands[leg].qDes[1]);
//        printf("leg: %d, knee_qDes: %f\n\n", leg, this->_data->_legController->commands[leg].qDes[2]);
//        printf("leg: %d, abad_qdDes: %f\n", leg, this->_data->_legController->commands[leg].qdDes[0]);
//        printf("leg: %d, hip_qdDes: %f\n", leg, this->_data->_legController->commands[leg].qdDes[1]);
//        printf("leg: %d, knee_qdDes: %f\n\n", leg, this->_data->_legController->commands[leg].qdDes[2]);
//        printf("leg: %d, kp: %f, %f, %f\n", leg,
//               this->_data->_legController->commands[leg].kpJoint(0,0),
//               this->_data->_legController->commands[leg].kpJoint(1,1),
//               this->_data->_legController->commands[leg].kpJoint(2,2));
//        printf("leg: %d, kd: %f, %f, %f\n\n", leg,
//               this->_data->_legController->commands[leg].kdJoint(0,0),
//               this->_data->_legController->commands[leg].kdJoint(1,1),
//               this->_data->_legController->commands[leg].kdJoint(2,2));
    }
}

/**
 * 判断动作是否完成,Busy状态下不允许切换状态
*/
template<typename T>
bool FSM_State_StandUp<T>::isBusy() {

    T t = (iter * this->_data->controlParameters->controller_dt) /
          (this->_data->userParameters->stand_up_time);

    if (t < 1.0f)
        return true;
    else
        return false;
}

// template class FSM_State_StandUp<double>;
template
class FSM_State_StandUp<float>;
