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
#include "can_messages/can_messages.h"
#include "can_messages/can_messages_sensors.h"
#include "lift_controller.h"

void canMonitorInit(void);

#endif /* __CAN_MONITOR_H */
