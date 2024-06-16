#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "ControlParameters/RobotParameters.h"
#include "Controllers/DesiredStateCommand.h"
#include "TJU_UserParameters.hpp"
#include "Controllers/GaitScheduler.h"
#include "Controllers/LegController.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/Quadruped.h"

/**
 *
 */
template<typename T>
struct ControlFSMData {
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Quadruped<T> *_quadruped;
    StateEstimatorContainer<T> *_stateEstimator;
    LegController<T> *_legController;
    GaitScheduler<T> *_gaitScheduler;
    DesiredStateCommand<T> *_desiredStateCommand;
    RobotControlParameters *controlParameters;
    TJU_UserParameters *userParameters;
    VisualizationData *visualizationData;
};

template
struct ControlFSMData<double>;
template
struct ControlFSMData<float>;

#endif  // CONTROLFSM_H