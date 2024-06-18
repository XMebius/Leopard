/*============================ Control FSM ============================*/
/**
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 */

#include "ControlFSM.h"
#include "FSM_State.h"
//#include "gamepad/Gamepad.hpp"
#include "string.h"

/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 *
 * @param _quadruped the quadruped information
 * @param _stateEstimator contains the estimated states
 * @param _legController interface to the leg controllers
 * @param _gaitScheduler controls scheduled foot contact modes
 * @param _desiredStateCommand gets the desired COM state trajectories
 * @param controlParameters passes in the control parameters from the GUI
 */
template<typename T>
ControlFSM<T>::ControlFSM(Quadruped<T> *_quadruped,
                          StateEstimatorContainer<T> *_stateEstimator,
                          LegController<T> *_legController,
                          GaitScheduler<T> *_gaitScheduler,
                          DesiredStateCommand<T> *_desiredStateCommand,
                          RobotControlParameters *controlParameters,
                          VisualizationData *visualizationData,
                          TJU_UserParameters *userParameters) {
    // Add the pointers to the ControlFSMData struct
    data._quadruped = _quadruped;
    data._stateEstimator = _stateEstimator;
    data._legController = _legController;
    data._gaitScheduler = _gaitScheduler;
    data._desiredStateCommand = _desiredStateCommand;
    data.controlParameters = controlParameters;
    data.visualizationData = visualizationData;
    data.userParameters = userParameters;

    // Initialize and add all of the FSM States to the state list
    statesList.invalid = nullptr;
    statesList.passive = new FSM_State_Passive<T>(&data);
//    statesList.sitDown = new FSM_State_SitDown<T>(&data);
    statesList.standUp = new FSM_State_StandUp<T>(&data);
    statesList.sitDown = new FSM_State_SitDown<T>(&data);
    statesList.locomotion = new FSM_State_Locomotion<T>(&data);
    statesList.recoveryStand = new FSM_State_RecoveryStand<T>(&data);
    statesList.balanceStand = new FSM_State_BalanceStand<T>(&data);

    safetyChecker = new SafetyChecker<T>(&data);

    // Initialize the FSM with the Passive FSM State
    initialize();
}

/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state.
 */
template<typename T>
void ControlFSM<T>::initialize() {
    // 上电后,passive
    printf("controlFSM initialized\n");
    currentState = statesList.passive;
    currentState->onEnter();
    nextState = currentState;
}

/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template<typename T>
void ControlFSM<T>::runFSM() {
    isPreSafe = safetyPreCheck();
    if (nextState == currentState && eStop) {
        switch (currentState->stateName) {
            case FSM_StateName::PASSIVE:
                /*if (data._desiredStateCommand->beiTong->leftBumper) { // LB pressed
//                    nextState = statesList.standUp;
                } else*/
                this->data._legController->isPassive = true;
                if (data._desiredStateCommand->beiTong->b) {
                    nextState = statesList.passive;
                } else if (data._desiredStateCommand->beiTong->rightBumper) {
                    nextState = statesList.recoveryStand;
                } else if (data._desiredStateCommand->beiTong->rightTriggerButton) {
                    nextState = statesList.standUp;
                }
                break;
            case FSM_StateName::STAND_UP:
                if (currentState->isBusy()) break;
                if (data._desiredStateCommand->beiTong->b) {
                    nextState = statesList.passive;
                } else if (data._desiredStateCommand->beiTong->x) {
                    nextState = statesList.locomotion;
                }
                break;
                /*case FSM_StateName::SIT_DOWN:
                    if (currentState->isBusy()) break;  // wait until the action is done
                    nextState = statesList.passive;
                    break;*/
            case FSM_StateName::RECOVERY_STAND:
                this->data._legController->isRecoveryStand = true;
                if (data._desiredStateCommand->beiTong->b) {
                    nextState = statesList.passive;
                } else if (data._desiredStateCommand->beiTong->y) {
                    nextState = statesList.balanceStand;
                } else if (data._desiredStateCommand->beiTong->x) {
                    nextState = statesList.locomotion;
                }
                break;
            case FSM_StateName::BALANCE_STAND:
                this->data._legController->isRecoveryStand = false;
                if (data._desiredStateCommand->beiTong->b) {
                    nextState = statesList.passive;
                } else if (data._desiredStateCommand->beiTong->x) {
                    nextState = statesList.locomotion;
                }
                break;
            case FSM_StateName::LOCOMOTION:
                this->data._legController->isLocomotion = true;
                if (currentState->isBusy()) break;
                if (data._desiredStateCommand->beiTong->b) { // LB pressed
                    nextState = statesList.passive;
                } else if (data._desiredStateCommand->beiTong->rightTriggerButton) {
                    nextState = statesList.standUp;
                }
                break;
            default:
                break;
        }
    }

    // 安全检查
    if (!isPreSafe || !isPostSafe) { // 任何一个检查不通过,则进入passive状态
        printf("not safe, enter passive state\n");
        nextState = statesList.passive;
        eStop = false;
    } else {
        eStop = true;
    }

    //状态切换
    if (nextState != currentState) {
        if (!currentState->isBusy()) {
            currentState = nextState;
            currentState->onEnter();
        }
    }

    currentState->run();
    printInfo();

    isPostSafe = safetyPostCheck();
}

/**
 * Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 * 安全预检
 * @return the appropriate operating mode
 */
template<typename T>
bool ControlFSM<T>::safetyPreCheck() {
    // Check for safe orientation if the current state requires it
    if (currentState->checkSafeOrientation) {
        if (!safetyChecker->checkSafeOrientation()) {
            std::cout << "broken: Orientation Safety Check FAIL" << std::endl;
            return false;
        }
    }
    return true;
}

/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 *
 * @return the appropriate operating mode
 */
template<typename T>
bool ControlFSM<T>::safetyPostCheck() {
    // Check for safe desired foot positions
    // 检测足底期望位置是否安全
    if (currentState->checkPDesFoot) {
        safetyChecker->checkPDesFoot();
    }
    // Check for safe desired feedforward forces
    // 检测期望足底前馈力是否安全
    if (currentState->checkForceFeedForward) {
        safetyChecker->checkForceFeedForward();
    }
    return true;
}

template<typename T>
void ControlFSM<T>::printInfo() {

    printIter++;

    // Print at commanded frequency
    if (printIter == printNum) {
        std::cout << "[CONTROL FSM] Printing FSM Info...\n";
        std::cout
                << "---------------------------------------------------------\n";
        std::cout << "Iteration: " << printIter << "\n";

        std::cout << "Operating Mode: NORMAL in " << currentState->stateString
                  << "\n";

        // Reset iteration counter
        printIter = 0;
    }

}

