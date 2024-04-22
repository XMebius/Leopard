//
// Created by mebius on 24-4-22.
//

#ifndef LEOPARD_RT_WHEELTEC_H
#define LEOPARD_RT_WHEELTEC_H

#include <cserialport.h>
#include <iostream>
#include <cstring>
#include "SimUtilities/IMUTypes.h"

#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295

#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
#define IMU_LEN 0x38
#define AHRS_LEN 0x30
#define INSGPS_LEN 0x54

bool init_port(void *);
void read_imu(void *, VectorNavData&);

#endif //LEOPARD_RT_WHEELTEC_H
