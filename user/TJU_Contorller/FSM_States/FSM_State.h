#ifndef FSM_State_H
#define FSM_State_H

#include <stdio.h>

#include "ControlFSMData.h"
#include "Controllers/GaitScheduler.h"

// Normal robot states
//#define K_PASSIVE         (0)
//#define K_STAND_UP        (1)
//#define K_SIT_DOWN        (2)
//#define K_LOCOMOTION      (3)
//#define K_BALANCE_STAND   (4)

/**
 * Enumerate all of the FSM states so we can keep track of them.
 */
enum class FSM_StateName {
    INVALID,
    PASSIVE,
    STAND_UP,
    SIT_DOWN,
    LOCOMOTION,
    RECOVERY_STAND,
    BALANCE_STAND,
};

template<typename T>
class FSM_State {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Generic constructor for all states
    FSM_State(ControlFSMData<T> *_controlFSMData, FSM_StateName stateNameIn,
              std::string stateStringNameI);

    // Behavior to be carried out when entering a state
    virtual void onEnter() = 0;// {}

    // Run the normal behavior for the state
    virtual void run() = 0; //{}

    //在busy状态下,不允许进行状态切换
    virtual bool isBusy() = 0;

    void jointPDControl(int leg, Vec3<T> qDes, Vec3<T> qdDes);

    void turnOnAllSafetyChecks();

    void turnOffAllSafetyChecks();

    // Holds all of the relevant control data
    ControlFSMData<T> *_data;

    // FSM State info
    FSM_StateName stateName;          // enumerated name of the current state
    std::string stateString;

    // Pre controls safety checks
    bool checkSafeOrientation = false;  // check roll and pitch

    // Post control safety checks
    bool checkPDesFoot = false;          // do not command footsetps too far
    bool checkForceFeedForward = false;  // do not command huge forces
    bool checkLegSingularity = false;    // do not let leg 奇异位置

    // Leg controller command placeholders for the whole robot (3x4 matrices)
    Mat34<T> jointFeedForwardTorques;  // feed forward joint torques
    Mat34<T> jointPositions;           // joint angle positions
    Mat34<T> jointVelocities;          // joint angular velocities
    Mat34<T> footFeedForwardForces;    // feedforward forces at the feet
    Mat34<T> footPositions;            // cartesian foot positions
    Mat34<T> footVelocities;           // cartesian foot velocities

protected:
////    float _delta_abad = 0.026977;
//    float _delta_abad = 0.106977;
////    float _delta_hip = 0.52602;
//    float _delta_hip = 0.62602;
////    float _delta_knee = 2.0202;
//    float _delta_knee = 1.4907074;
////    float _delta_knee = 2.6907074;

    // Create the cartesian P gain matrix
    Mat3<float> kpMat;

    // Create the cartesian D gain matrix
    Mat3<float> kdMat;
};

#endif  // FSM_State_H
