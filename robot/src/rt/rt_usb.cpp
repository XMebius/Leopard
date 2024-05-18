//
// Created by mebius on 24-4-22.
//

#ifdef USE_MOTOR

#include "rt/rt_usb.h"

//#define MOTOR_CHEATER   // initial passive

MotorCmd abad_cmd, hip_cmd, knee_cmd;
std::vector<MotorCmd> spine_leg_cmd{abad_cmd, hip_cmd, knee_cmd};

MotorData abad_data, hip_data, knee_data;
std::vector<MotorData> spine_leg_data{abad_data, hip_data, knee_data};
bool first_initial[4] = {true, true, true, true};

usb_command_t usb_command_drv;
usb_data_t usb_data_drv;
usb_torque_t usb_torque_drv;

//SerialPort *serial_0 = new SerialPort("/dev/ttyACM0");
//SerialPort *serial_1 = new SerialPort("/dev/ttyACM1");
//SerialPort *serial_2 = new SerialPort("/dev/ttyACM2");
//SerialPort *serial_3 = new SerialPort("/dev/ttyACM3");
SerialPort *serial_0;
SerialPort *serial_1;
SerialPort *serial_2;
SerialPort *serial_3;

std::vector<SerialPort *> serial = {serial_0, serial_1, serial_2, serial_3};

pthread_mutex_t usb_mutex;

const float max_torque[3] = {17.f, 17.f, 26.f};// TODO CHECK WITH BEN
const float wimp_torque[3] = {6.f, 6.f, 6.f};  // TODO CHECK WITH BEN
const float disabled_torque[3] = {0.f, 0.f, 0.f};

// only used for actual robot
//const float abad_side_sign[4] = {-1.f, -1.f, 1.f, 1.f};
const float abad_side_sign[4] = {6.33f, -6.33f, -6.33f, 6.33f};
const float hip_side_sign[4] = {6.33f, -6.33f, 6.33f, -6.33f};
//const float knee_side_sign[4] = {-.6429f, .6429f, -.6429f, .6429f};
const float knee_side_sign[4] = {-6.33f, 6.33f, -6.33f, 6.33f};

// only used for actual robot
float abad_offset[4] = {0.f, 0.f, 0.f, 0.f};
float hip_offset[4] = {0.f, 0.f, 0.f, 0.f};
float knee_offset[4] = {0.f, 0.f, 0.f, 0.f};   // 4.35
//float abad_offset[4] = {0.266933f, -0.208075f, -0.311219f, 0.226977f};
//float hip_offset[4] = {0.549557f, -0.459771f, 0.526293f, -0.568459f};
//float knee_offset[4] = {-2.111409f, 2.058308f, -1.974702f, 1.96925f};

void init_usb() {

    serial_0 = new SerialPort("/dev/ttyACM0");
    serial_1 = new SerialPort("/dev/ttyACM1");
    serial_2 = new SerialPort("/dev/ttyACM2");
    serial_3 = new SerialPort("/dev/ttyACM3");

    // check size
    size_t command_size = sizeof(usb_command_t);
    size_t data_size = sizeof(usb_data_t);

    memset(&usb_command_drv, 0, sizeof(usb_command_drv));
    memset(&usb_data_drv, 0, sizeof(usb_data_drv));

    if (pthread_mutex_init(&usb_mutex, NULL) != 0) {
        printf("Error initializing usb mutex\n");
    }

    if (command_size != K_EXPECTED_COMMAND_SIZE) {
        printf("usb_command_t size is %lu, expected %d\n", command_size, K_EXPECTED_COMMAND_SIZE);
    } else {
        printf("[RT USB] command size good\n");
    }

    if (data_size != K_EXPECTED_DATA_SIZE) {
        printf("usb_data_t size is %lu, expected %d\n", data_size, K_EXPECTED_DATA_SIZE);
    } else {
        printf("[RT USB] data size good\n");
    }

    printf("[RT USB] open\n");
}

void printCmd(usb_command_t *cmd, uint8_t leg) {
    if (leg == 2
        && std::abs(cmd->q_des_abad[leg]) > 0.0
        && std::abs(cmd->q_des_hip[leg]) > 0.0
        && std::abs(cmd->q_des_knee[leg]) > 0.0) {

        printf("---------------------------------------------\n");

        printf("original_abad--> q_des: %f, qd_des: %f, kp:%f, kd:%f, tau:%f\n",
               cmd->q_des_abad[leg], cmd->qd_des_abad[leg], cmd->kp_abad[leg], cmd->kd_abad[leg],
               cmd->tau_abad_ff[leg]);

        printf("original_hip--> q_des: %f, qd_des: %f, kp:%f, kd:%f, tau:%f\n",
               cmd->q_des_hip[leg], cmd->qd_des_hip[leg], cmd->kp_hip[leg], cmd->kd_hip[leg], cmd->tau_hip_ff[leg]);

        printf("original_knee--> q_des: %f, qd_des: %f, kp:%f, kd:%f, tau:%f\n",
               cmd->q_des_knee[leg], cmd->qd_des_knee[leg], cmd->kp_knee[leg], cmd->kd_knee[leg],
               cmd->tau_knee_ff[leg]);

        printf("abad--> q_des: %f, qd_des: %f, kp:%f, kd:%f, tau:%f\n",
               (cmd->q_des_abad[leg] * abad_side_sign[leg]) + abad_offset[leg],
               cmd->qd_des_abad[leg] * abad_side_sign[leg],
               cmd->kp_abad[leg],
               cmd->kd_abad[leg],
               cmd->tau_abad_ff[leg] * abad_side_sign[leg]);

        printf("hip--> q_des: %f, qd_des: %f, kp:%f, kd:%f, tau:%f\n",
               (cmd->q_des_hip[leg] * hip_side_sign[leg]) + hip_offset[leg],
               cmd->qd_des_hip[leg] * hip_side_sign[leg],
               cmd->kp_hip[leg],
               cmd->kd_hip[leg],
               cmd->tau_hip_ff[leg] * hip_side_sign[leg]);

        printf("knee--> q_des: %f, qd_des: %f, kp:%f, kd:%f, tau:%f\n",
               (cmd->q_des_knee[leg] * knee_side_sign[leg]) + knee_offset[leg],
               cmd->qd_des_knee[leg] * knee_side_sign[leg],
               cmd->kp_knee[leg],
               cmd->kd_knee[leg],
               cmd->tau_knee_ff[leg] * knee_side_sign[leg]);
    }

}

void usb_to_spine(usb_command_t *cmd, std::vector<MotorCmd> &SPine_leg_cmd, uint8_t leg) {

    assert(leg >= 0 && leg <= 3);
    // todo: mode settings and mode
    SPine_leg_cmd[0].id = leg * 3 + 0;// abad 6
    SPine_leg_cmd[1].id = leg * 3 + 1;// hip 7
    SPine_leg_cmd[2].id = leg * 3 + 2;// knee 8

    for (auto &i: SPine_leg_cmd) {
        i.mode = 1;// foc control
    }

//    printf("leg: %d q_des_abad: %f, q_des_hip: %f, q_des_knee: %f\n", leg, cmd->q_des_abad[leg], cmd->q_des_hip[leg], cmd->q_des_knee[leg]);
//    printf("leg: %d qd_des_abad: %f, qd_des_hip: %f, qd_des_knee: %f\n", leg, cmd->qd_des_abad[leg], cmd->qd_des_hip[leg], cmd->qd_des_knee[leg]);
//    printf("leg: %d kp_abad: %f, kp_hip: %f, kp_knee: %f\n", leg, cmd->kp_abad[leg], cmd->kp_hip[leg], cmd->kp_knee[leg]);
//    printf("leg: %d kd_abad: %f, kd_hip: %f, kd_knee: %f\n", leg, cmd->kd_abad[leg], cmd->kd_hip[leg], cmd->kd_knee[leg]);
//    printf("leg: %d tau_abad_ff: %f, tau_hip_ff: %f, tau_knee_ff: %f\n", leg, cmd->tau_abad_ff[leg], cmd->tau_hip_ff[leg], cmd->tau_knee_ff[leg]);

#ifndef MOTOR_CHEATER

    // abad hip knee
    SPine_leg_cmd[0].Pos = (cmd->q_des_abad[leg] * abad_side_sign[leg]) + abad_offset[leg];
    SPine_leg_cmd[1].Pos = (cmd->q_des_hip[leg] * hip_side_sign[leg]) + hip_offset[leg];
    SPine_leg_cmd[2].Pos = (cmd->q_des_knee[leg] * knee_side_sign[leg]) + knee_offset[leg];

    SPine_leg_cmd[0].W = cmd->qd_des_abad[leg] * abad_side_sign[leg];
    SPine_leg_cmd[1].W = cmd->qd_des_hip[leg] * hip_side_sign[leg];
    SPine_leg_cmd[2].W = cmd->qd_des_knee[leg] * knee_side_sign[leg];

    SPine_leg_cmd[0].K_P = cmd->kp_abad[leg];
    SPine_leg_cmd[1].K_P = cmd->kp_hip[leg];
    SPine_leg_cmd[2].K_P = cmd->kp_knee[leg];

    SPine_leg_cmd[0].K_W = cmd->kd_abad[leg];
    SPine_leg_cmd[1].K_W = cmd->kd_hip[leg];
    SPine_leg_cmd[2].K_W = cmd->kd_knee[leg];

    SPine_leg_cmd[0].T = cmd->tau_abad_ff[leg] * abad_side_sign[leg];
    SPine_leg_cmd[1].T = cmd->tau_hip_ff[leg] * hip_side_sign[leg];
    SPine_leg_cmd[2].T = cmd->tau_knee_ff[leg] * knee_side_sign[leg];

//    printf("leg: %d abad: %f, hip: %f, knee: %f\n", leg, SPine_leg_cmd[0].Pos, SPine_leg_cmd[1].Pos, SPine_leg_cmd[2].Pos);
//    printf("leg: %d abad_offset: %f, hip_offset: %f, knee_offset: %f\n", leg, abad_offset[leg], hip_offset[leg], knee_offset[leg]);
//    printf("leg: %d abad_kp: %f, hip_kp: %f, knee_kp: %f\n", leg, SPine_leg_cmd[0].K_P, SPine_leg_cmd[1].K_P, SPine_leg_cmd[2].K_P);
//    printf("leg: %d abad_kd: %f, hip_kd: %f, knee_kd: %f\n", leg, SPine_leg_cmd[0].K_W, SPine_leg_cmd[1].K_W, SPine_leg_cmd[2].K_W);
//    printf("leg: %d abad_tau: %f, hip_tau: %f, knee_tau: %f\n\n", leg, SPine_leg_cmd[0].T, SPine_leg_cmd[1].T, SPine_leg_cmd[2].T);
#else
    (void) cmd;
    SPine_leg_cmd[0].Pos = 0.0 + abad_offset[leg];// abad
    SPine_leg_cmd[1].Pos = 0.0 + hip_offset[leg];// hip
    SPine_leg_cmd[2].Pos = 0.0 + knee_offset[leg];// knee

    SPine_leg_cmd[0].W = 0.0;// abad
    SPine_leg_cmd[1].W = 0.0;// hip
    SPine_leg_cmd[2].W = 0.0;// knee

    SPine_leg_cmd[0].K_P = 0.05;
    SPine_leg_cmd[1].K_P = 0.05;
    SPine_leg_cmd[2].K_P = 0.05;

    SPine_leg_cmd[0].K_W = 0.0;// abad
    SPine_leg_cmd[1].K_W = 0.0;// hipclear
    SPine_leg_cmd[2].K_W = 0.0;// knee

    SPine_leg_cmd[0].T = 0.0;// abad
    SPine_leg_cmd[1].T = 0.0;// hip
    SPine_leg_cmd[2].T = 0.0;// knee

#endif
}

void spine_to_usb(usb_data_t *data, std::vector<MotorData> &SPine_leg_data, uint8_t leg) {

    if (first_initial[leg]) {  // 保存offset为初始位置
        abad_offset[leg] = SPine_leg_data[0].Pos;
        hip_offset[leg] = SPine_leg_data[1].Pos;
        knee_offset[leg] = SPine_leg_data[2].Pos;
        first_initial[leg] = false;
    }

    data->q_abad[leg] = (SPine_leg_data[0].Pos - abad_offset[leg]) / abad_side_sign[leg]; //abad
    data->q_hip[leg] = (SPine_leg_data[1].Pos - hip_offset[leg]) / hip_side_sign[leg];   // hip
    data->q_knee[leg] = (SPine_leg_data[2].Pos - knee_offset[leg]) / knee_side_sign[leg];// knee

    data->qd_abad[leg] = SPine_leg_data[0].W / abad_side_sign[leg];// abad
    data->qd_hip[leg] = SPine_leg_data[1].W / hip_side_sign[leg];  // hip
    data->qd_knee[leg] = SPine_leg_data[2].W / knee_side_sign[leg];// knee

//    data->q_abad[leg] = SPine_leg_data[0].Pos / 6.33f; //abad
//    data->q_hip[leg] = SPine_leg_data[1].Pos / 6.33f;   // hip
//    data->q_knee[leg] = SPine_leg_data[2].Pos / 6.33f;// knee
//
//    data->qd_abad[leg] = SPine_leg_data[0].W * abad_side_sign[leg];// abad
//    data->qd_hip[leg] = SPine_leg_data[1].W * hip_side_sign[leg];  // hip
//    data->qd_knee[leg] = SPine_leg_data[2].W * knee_side_sign[leg];// knee

//    printf("q_abad: %f, q_hip: %f, q_knee: %f\n", SPine_leg_data[0].Pos, SPine_leg_data[1].Pos, SPine_leg_data[2].Pos);
//    printf("qd_abad: %f, qd_hip: %f, qd_knee: %f\n", SPine_leg_data[0].W, SPine_leg_data[1].W, SPine_leg_data[2].W);

//    printf("data_abad--> q: %f, qd: %f\n", data->q_abad[leg], data->qd_abad[leg]);
//    printf("data_hip--> q: %f, qd: %f\n", data->q_hip[leg], data->qd_hip[leg]);
//    printf("data_knee--> q: %f, qd: %f\n", data->q_knee[leg], data->qd_knee[leg]);
    // no check
}

int usb_driver_iterations = 0;

void usb_send_receive(usb_command_t *command, usb_data_t *data) {
    usb_driver_iterations++;
    data->spi_driver_status = usb_driver_iterations << 16;

    // each leg TODO: send 3 motors at the same time???
    for (uint8_t leg = 0; leg < 4; leg++) {
        usb_to_spine(command, spine_leg_cmd, leg);
        serial[leg]->sendRecv(spine_leg_cmd, spine_leg_data);
        spine_to_usb(data, spine_leg_data, leg);
    }
}

void usb_driver_run() {
    // lock mutex
    pthread_mutex_lock(&usb_mutex);
    usb_send_receive(&usb_command_drv, &usb_data_drv);
    pthread_mutex_unlock(&usb_mutex);
}

usb_command_t *get_usb_command() {
    return &usb_command_drv;
}

usb_data_t *get_usb_data() {
    return &usb_data_drv;
}

#endif // USE_MOTOR