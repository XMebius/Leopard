//
// Created by mebius on 24-5-13.
//

#include "FSM_State_Locomotion.h"

template<typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION"),
          trotting(10, Vec4<int>(0, 5, 5, 0), Vec4<int>(5, 5, 5, 5), "Trotting") {
    this->turnOnAllSafetyChecks();
    // Turn off Foot pos command since it is set in WBC as operational task
    this->checkPDesFoot = false;
    dt = this->_data->controlParameters->controller_dt;
    dtMPC = 27 / 1000.;
    iterationsBetweenMPC = 27 / (1000. * dt);
    default_iterations_between_mpc = iterationsBetweenMPC;

    for (int i = 0; i < 4; i++) firstSwing[i] = true;
    firstRun = true;
}

template<typename T>
void FSM_State_Locomotion<T>::onEnter() {
    printf("[FSM_State_Locomotion] onEnter...\n");
    for (int leg = 0; leg < 4; leg++) {
        standJointPos[leg] = this->_data->_legController->datas[leg].q;
    }
}

template<typename T>
void FSM_State_Locomotion<T>::recompute_timing(int iterations_per_mpc) {
    iterationsBetweenMPC = iterations_per_mpc;
    dtMPC = iterations_per_mpc * dt;
}

template<typename T>
void FSM_State_Locomotion<T>::setupCommand(ControlFSMData<float> &data) {

    _body_height = 0.32;

    float x_vel_cmd, y_vel_cmd;
    float filter(0.1);
    if (data.controlParameters->use_rc) {
        const rc_control_settings *rc_cmd = data._desiredStateCommand->rcCommand;
        data.userParameters->cmpc_gait = rc_cmd->variable[0];
        _yaw_turn_rate = -rc_cmd->omega_des[2];
        x_vel_cmd = rc_cmd->v_des[0];
        y_vel_cmd = rc_cmd->v_des[1] * 0.5;
        _body_height += rc_cmd->height_variation * 0.08;
    } else {
        _yaw_turn_rate = data._desiredStateCommand->rightAnalogStick[0];
        x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1];
        y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0];

        // debug
        _yaw_turn_rate = 0.f;
        x_vel_cmd = 1.f;
        y_vel_cmd = 0.f;
    }
    float _x_vel_multiplier = 1.f;  // 可以根据需要调整
    _x_vel_des = _x_vel_des * (1 - filter) + (x_vel_cmd * _x_vel_multiplier) * filter;
//    _x_vel_des = _x_vel_des * (1 - filter) + x_vel_cmd * filter;
    _y_vel_des = _y_vel_des * (1 - filter) + y_vel_cmd * filter;

    _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
    _roll_des = 0.;
    _pitch_des = 0.;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template<typename T>
void FSM_State_Locomotion<T>::run() {
    // 通过遥感设置xy速度和yaw角速度，以及设置身体高度和不被使用的roll和pitch
    setupCommand(*this->_data);
    auto &seResult = this->_data->_stateEstimator->getResult();

    Gait *gait = &trotting;
    gait->setIterations(iterationsBetweenMPC, iterationCounter);

    recompute_timing(default_iterations_between_mpc);

    Vec3<float> v_robot = seResult.vWorld;  // 当前值
    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0); // 身体坐标系下的期望值
    Vec3<float> v_des_world = seResult.rBody.transpose() * v_des_robot; // 世界坐标系下的期望值

    if (fabs(v_robot[0]) > .2)   //avoid dividing by zero
        rpy_int[1] += dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
    if (fabs(v_robot[1]) > 0.1)
        rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];

    rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
    rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
    rpy_comp[1] = v_robot[0] * rpy_int[1];
    rpy_comp[0] = v_robot[1] * rpy_int[0];

    for (int i = 0; i < 4; i++) {
//        pFoot[i] = seResult.position +
//                   seResult.rBody.transpose() * (this->_data->_quadruped->getHipLocation(i) +
//                                                 this->_data->_legController->datas[i].p);
        // 直接使用身体坐标系
        pFoot[i] = this->_data->_legController->datas[i].p;
    }

    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);

    if (firstRun) {
        world_position_desired[0] = seResult.position[0];
        world_position_desired[1] = seResult.position[1];
        world_position_desired[2] = seResult.rpy[2];

        for (int i = 0; i < 4; i++) {
            _jointSwingTrajectories[i].setHeight(0.22, i);
            _jointSwingTrajectories[i].setInitialJointPosition(pFoot[i], i);
            _jointSwingTrajectories[i].setFinalJointPosition(pFoot[i], i);
        }
        firstRun = false;
    }

    // foot placement
    for (int l = 0; l < 4; l++)
        swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

    // 设置swingTime和stanceTime
    float side_sign[4] = {-1, 1, -1, 1};
    float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
    float interleave_gain = -0.2;
    float v_abs = std::fabs(v_des_robot[0]);

//    float _step_length_multiplier = 1.3f;  // 增加步长
//    float _foot_placement_factor = 1.2f;   // 增加足端位置的前向偏移
//    float _prediction_horizon = 3;         // 预测未来几个时间步

    for (int i = 0; i < 4; i++) {
        if (firstSwing[i]) {
            swingTimeRemaining[i] = swingTimes[i];
        } else {
            swingTimeRemaining[i] -= dt;
        }
        if (gait->getSwingState()[i] > 0) {
            _jointSwingTrajectories[i].setHeight(0.22, i);
        } else {
            _jointSwingTrajectories[i].setHeight(_body_height, i);

        }
        Vec3<float> offset(0, side_sign[i] * .065, 0);
        Vec3<float> pRobotFrame = (this->_data->_quadruped->getHipLocation(i) + offset);
        pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
        float stance_time = gait->getCurrentStanceTime(dtMPC, i);
        Vec3<float> pYawCorrected =
                coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate * stance_time / 2) * pRobotFrame;

        Vec3<float> des_vel;
        des_vel[0] = _x_vel_des;
        des_vel[1] = _y_vel_des;
        des_vel[2] = 0.0;

        Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected
                                                                           + des_vel * swingTimeRemaining[i]);

        float p_rel_max = 0.3f;
//        float p_rel_max = 0.3f * _step_length_multiplier;

        float pfx_rel = seResult.vWorld[0] * .5 * stance_time +
                        .03f * (seResult.vWorld[0] - v_des_world[0]) +
                        (0.5f * seResult.position[2] / 9.81f) * (seResult.vWorld[1] * _yaw_turn_rate);
//        float pfx_rel = predicted_vel[0] * .5 * stance_time +
//                        .03f * (predicted_vel[0] - v_des_world[0]) +
//                        (0.5f * predicted_pos[2] / 9.81f) * (predicted_vel[1] * _yaw_turn_rate);

        float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +
                        .03f * (seResult.vWorld[1] - v_des_world[1]) +
                        (0.5f * seResult.position[2] / 9.81f) * (-seResult.vWorld[0] * _yaw_turn_rate);
//        float pfy_rel = predicted_vel[1] * .5 * stance_time * dtMPC +
//                        .03f * (predicted_vel[1] - v_des_world[1]) +
//                        (0.5f * predicted_pos[2] / 9.81f) * (-predicted_vel[0] * _yaw_turn_rate);

        pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
        pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
        Pf[0] += pfx_rel;
        Pf[1] += pfy_rel;
        Pf[2] = -0.003;
        //Pf[2] = 0.0;
        // 转换为身体坐标系
        Vec3<float> pDesLeg = seResult.rBody * (Pf - seResult.position)
                              - this->_data->_quadruped->getHipLocation(i);
//        // 平滑足端位置变化
//        if (!firstSwing[i]) {
//            pDesLeg = 0.7f * pDesLeg + 0.3f * _previous_pDesLeg[i];
//        }
//        _previous_pDesLeg[i] = pDesLeg;

        if (i == 0) {  // 对一条腿进行分析
            printf("swingState: %f\n", gait->getSwingState()[i]);
            printf("pDesLeg: %f, %f, %f\n\n", pDesLeg[0], pDesLeg[1], pDesLeg[2]);
        }

        _jointSwingTrajectories[i].setFinalJointPosition(pDesLeg, i);
    }

    iterationCounter++;
    Vec4<float> swingStates = gait->getSwingState();
//    Vec4<float> se_contactState(0, 0, 0, 0);

    for (int foot = 0; foot < 4; foot++) {
        float swingState = swingStates[foot];
        if (swingState > 0) {   // swing
            if (firstSwing[foot]) {
                firstSwing[foot] = false;
                _jointSwingTrajectories[foot].setInitialJointPosition(pFoot[foot], foot);
            }
            _jointSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

            Vec3<float> qJoint = _jointSwingTrajectories[foot].getJointPosition();
            Vec3<float> qdJoint = _jointSwingTrajectories[foot].getJointVelocity();
            this->_data->_legController->commands[foot].qDes = qJoint;
            this->_data->_legController->commands[foot].qdDes = qdJoint;
            this->_data->_legController->commands[foot].kpJoint = Vec3<T>(2.2, 2.2, 2.2).asDiagonal();
            this->_data->_legController->commands[foot].kdJoint = Vec3<T>(0.022, 0.022, 0.022).asDiagonal();
        } else {
            firstSwing[foot] = true;

            this->_data->_legController->commands[foot].qDes = standJointPos[foot];
            this->_data->_legController->commands[foot].qdDes = Vec3<T>(0.f, 0.f, 0.f);
//            Vec3<float> qJoint = _jointSwingTrajectories[foot].getJointPosition();
//            Vec3<float> qdJoint = _jointSwingTrajectories[foot].getJointVelocity();
//            this->_data->_legController->commands[foot].qDes = qJoint;
//            this->_data->_legController->commands[foot].qdDes = qdJoint;
            this->_data->_legController->commands[foot].kpJoint = Vec3<T>(3.6, 3.6, 3.6).asDiagonal();
            this->_data->_legController->commands[foot].kdJoint = Vec3<T>(0.032, 0.032, 0.032).asDiagonal();

        }
    }
//    this->_data->_stateEstimator->setContactPhase(se_contactState);
//    contact_state = gait->getContactState();
}

template<typename T>
bool FSM_State_Locomotion<T>::isBusy() {
    return false;
}

template
class FSM_State_Locomotion<float>;