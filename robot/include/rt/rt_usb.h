//
// Created by mebius on 24-4-22.
//

#ifndef LEOPARD_RT_USB_H
#define LEOPARD_RT_USB_H

#include <serialPort/SerialPort.h>
#include <unistd.h>
#include <usb_command_t.hpp>
#include <usb_data_t.hpp>
#include <usb_torque_t.hpp>
#include <vector>
#include <math.h>
#include <cassert>

#define K_EXPECTED_COMMAND_SIZE 256
#define K_WORDS_PER_MESSAGE 66
#define K_EXPECTED_DATA_SIZE 116
#define K_KNEE_OFFSET_POS 4.35f

void init_usb();

void usb_driver_run();
usb_command_t* get_usb_command();
usb_data_t* get_usb_data();

#endif //LEOPARD_RT_USB_H
