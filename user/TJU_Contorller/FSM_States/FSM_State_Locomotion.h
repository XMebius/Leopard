//
// Created by mebius on 24-5-13.
//

#ifndef LEOPARD_FSM_STATE_LOCOMOTION_H
#define LEOPARD_FSM_STATE_LOCOMOTION_H

#include "FSM_State.h"
#include <Controllers/JointSwingTrajectory.h>
#include <Controllers/convexMPC/Gait.h>


template<typename T>
class FSM_State_Locomotion :public FSM_State<T>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData);

    void onEnter() override;

    void run() override;

    void setupCommand(ControlFSMData<float> &data);

    void recompute_timing(int iterations_per_mpc);

    bool isBusy() override;
private:
    // Keep track of the control iterations
    int iter = 0;

    Vec3<T> standJointPos[4];

    int iterationsBetweenMPC;
    int iterationCounter = 0;
    float dt;
    float dtMPC;
    int default_iterations_between_mpc;

    float _yaw_turn_rate;
    float _yaw_des;
    float _roll_des;
    float _pitch_des;
    float _body_height;
    float _x_vel_des = 0.;
    float _y_vel_des = 0.;

    Vec3<float> world_position_desired;
    Vec3<float> rpy_int;
    Vec3<float> rpy_comp;

    Vec3<float> pFoot[4];

    bool firstRun;
    Vec4<float> swingTimes;
    bool firstSwing[4];
    float swingTimeRemaining[4];

    Vec3<float> _previous_pDesLeg[4];

    JointSwingTrajectory<float> _jointSwingTrajectories[4];

    OffsetDurationGait trotting;
};


#endif //LEOPARD_FSM_STATE_LOCOMOTION_H
