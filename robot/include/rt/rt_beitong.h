//
// Created by mebius on 24-5-9.
//

#ifndef LEOPARD_RT_BEITONG_H
#define LEOPARD_RT_BEITONG_H

#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <map>

#include "Utilities/BeiTong.h"

bool init_gamepad();
void read_gamepad(BeiTong &beiTong);    // read once

#endif //LEOPARD_RT_BEITONG_H
