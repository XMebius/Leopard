/*
 * @file FSM_State_RecoveryStand.cpp
 * @brief 
 */

#include "FSM_State_RecoveryStand.h"

template<typename T>
FSM_State_RecoveryStand<T>::FSM_State_RecoveryStand(ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::RECOVERY_STAND, "RECOVERY_STAND") {
    this->turnOffAllSafetyChecks();

    zero_vec3.setZero();
    /********** 目标位置配置 **********/

    // Folding
    fold_jpos[0] << -0.0f, -1.4f, 2.7f;
    fold_jpos[1] << 0.0f, -1.4f, 2.7f;
    fold_jpos[2] << -0.0f, -1.4f, 2.7f;
    fold_jpos[3] << 0.0f, -1.4f, 2.7f;

    // Stand Up
    stand_jpos[0] << 0.f, -.65f, 1.35f;
    stand_jpos[1] << 0.f, -.65f, 1.35f;
    stand_jpos[2] << 0.f, -.60f, 1.30f;
    stand_jpos[3] << 0.f, -.60f, 1.30f;

    // Rolling
    rolling_jpos[0] << 1.5f, -1.6f, 2.77f;
    rolling_jpos[1] << 1.3f, -3.1f, 2.77f;
    rolling_jpos[2] << 1.5f, -1.6f, 2.77f;
    rolling_jpos[3] << 1.3f, -3.1f, 2.77f;
}

template<typename T>
void FSM_State_RecoveryStand<T>::onEnter() {
    printf("[FSM_State_RecoveryStand] onEnter...\n");

    // 重置迭代器
    iter = 0;
    _state_iter = 0;
    _motion_start_iter = 0;

    /********** 初始位置配置 **********/
    for (size_t i(0); i < 4; ++i) {
        initial_jpos[i] = this->_data->_legController->datas[i].q;
    }
    T body_height =
            this->_data->_stateEstimator->getResult().position[2];

    // 初始状态为内展腿
    _flag = FoldLegs;
    // 验证初始姿态并进入退出FoldLegs状态
    if (!_UpsideDown()) { // Proper orientation
        if ((0.2 < body_height) && (body_height < 0.45)) {
            printf("[Recovery Balance] body height is %f; Stand Up \n", body_height);
            _flag = StandUp;
        } else {
            printf("[Recovery Balance] body height is %f; Folding legs \n", body_height);
        }
    } else {
        printf("[Recovery Balance] UpsideDown (%d) \n", _UpsideDown());
    }
}

template<typename T>
bool FSM_State_RecoveryStand<T>::_UpsideDown() {
    if (this->_data->_stateEstimator->getResult().rBody(2, 2) < 0) {
        return true;
    }
    return false;
}

template<typename T>
void FSM_State_RecoveryStand<T>::run() {
    switch (_flag) {
        case StandUp:
            _StandUp(_state_iter - _motion_start_iter);
            break;
        case FoldLegs:
            _FoldLegs(_state_iter - _motion_start_iter);
            break;
        case RollOver:
            _RollOver(_state_iter - _motion_start_iter);
            break;
    }
    ++_state_iter;
}

template<typename T>
void FSM_State_RecoveryStand<T>::_FoldLegs(const int &curr_iter) {

    for (size_t i(0); i < 4; ++i) {
        _SetJPosInterPts(curr_iter, fold_ramp_iter, i,
                         initial_jpos[i], fold_jpos[i]);
    }

    // 过了settle_iter之后，进入下一个状态
    if (curr_iter >= fold_ramp_iter + fold_settle_iter) {
//        if(_UpsideDown()){  // 如果身体发生了翻转
//            _flag = RollOver;
//            for(size_t i(0); i<4; ++i) initial_jpos[i] = fold_jpos[i];
//        }else{
        _flag = StandUp;
        for (size_t i(0); i < 4; ++i) initial_jpos[i] = fold_jpos[i];
//        }
        _motion_start_iter = _state_iter + 1;
    }
}

template<typename T>
void FSM_State_RecoveryStand<T>::_RollOver(const int &curr_iter) {

    for (size_t i(0); i < 4; ++i) {
        _SetJPosInterPts(curr_iter, rollover_ramp_iter, i,
                         initial_jpos[i], rolling_jpos[i]);
    }

    if (curr_iter > rollover_ramp_iter + rollover_settle_iter) {
        _flag = FoldLegs;
        for (size_t i(0); i < 4; ++i) initial_jpos[i] = rolling_jpos[i];
        _motion_start_iter = _state_iter + 1;
    }
}

template<typename T>
void FSM_State_RecoveryStand<T>::_StandUp(const int &curr_iter) {
    T body_height = this->_data->_stateEstimator->getResult().position[2];
    bool something_wrong(false);

//    if( _UpsideDown() || (body_height < 0.1 ) ) {
//        something_wrong = true;
//    }

    if ((curr_iter > floor(standup_ramp_iter * 0.7)) && something_wrong) {
        // If body height is too low because of some reason
        // even after the stand up motion is almost over
        // (Can happen when E-Stop is engaged in the middle of Other state)
        for (size_t i(0); i < 4; ++i) {
            initial_jpos[i] = this->_data->_legController->datas[i].q;
        }
        _flag = FoldLegs;
        _motion_start_iter = _state_iter + 1;

        printf("[Recovery Balance - Warning] body height is still too low (%f) or UpsideDown (%d); Folding legs \n",
               body_height, _UpsideDown());

    } else {  // 无问题，继续站立

        int curr_iter_backup = curr_iter;
        if (curr_iter >= standup_ramp_iter)
            curr_iter_backup = standup_ramp_iter;

        for (size_t leg(0); leg < 4; ++leg) {
            this->_data->_legController->commands[leg].qDes =
                    Interpolate::cubicBezier<Vec3<T>>(initial_jpos[leg],
                                                      stand_jpos[leg],
                                                      curr_iter_backup * 1. / standup_ramp_iter * 1.);

            this->_data->_legController->commands[leg].qdDes =
                    Interpolate::cubicBezierFirstDerivative<Vec3<T>>(initial_jpos[leg],
                                                                     stand_jpos[leg],
                                                                     curr_iter_backup * 1. / standup_ramp_iter * 1.);

            this->_data->_legController->commands[leg].kpJoint = Vec3<T>(1.8, 1.8, 1.8).asDiagonal();
            this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.02, 0.02, 0.02).asDiagonal();
        }
    }
    Vec4<T> se_contactState(0.5, 0.5, 0.5, 0.5);
    this->_data->_stateEstimator->setContactPhase(se_contactState);
}

template<typename T>
void FSM_State_RecoveryStand<T>::_SetJPosInterPts(
        const size_t &curr_iter, size_t max_iter, int leg,
        const Vec3<T> &ini, const Vec3<T> &fin) {

    // 权重
    float a(0.f);
    float b(1.f);

    // if we're done interpolating
    if (curr_iter <= max_iter) {
        b = (float) curr_iter / (float) max_iter;
        a = 1.f - b;
    }

    // 加权和
    Vec3<T> inter_pos = a * ini + b * fin;

    // do control
    this->jointPDControl(leg, inter_pos, zero_vec3);
}

template<typename T>
bool FSM_State_RecoveryStand<T>::isBusy() {
    return false;
}

template
class FSM_State_RecoveryStand<float>;

