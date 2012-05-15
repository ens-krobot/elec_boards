/*
 * can_monitor.h
 * -------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#ifndef __CAN_MONITOR_H
#define __CAN_MONITOR_H

#include <drv/can.h>
#include <drv/timer.h>

#include "encoder.h"
#include "motor_controller.h"
#include "trajectory_controller.h"
#include "odometry.h"
#include "differential_drive.h"
#include "can_messages.h"

void canMonitorInit(void);
void can_send_error(uint8_t err1, uint8_t err2);

#endif /* __CAN_MONITOR_H */
