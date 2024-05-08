//
// Created by mebius on 24-4-23.
//

#include "TJU_Controller.hpp"

TJU_Controller::TJU_Controller() : RobotController() {}

void TJU_Controller::initializeController() {
    std::cout << "Initializing FSM Controller" << std::endl;
    _controlFSM = new ControlFSM<float>(
            _quadruped,
            _stateEstimator,
            _legController,
            _desiredStateCommand,
            _controlParameters,
            _visualizationData,
            &userParameters);

}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void TJU_Controller::runController() {

    _controlFSM->runFSM();
}