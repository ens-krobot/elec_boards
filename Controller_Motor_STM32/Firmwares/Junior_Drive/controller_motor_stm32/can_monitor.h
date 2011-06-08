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
#include "asservissement.h"
#include "odometry.h"
#include "can_messages.h"
#include "can_messages_sensors.h"
#include "AX12.h"

adc_values adc1_msg, adc2_msg;

void canMonitorInit(void);

#endif /* __CAN_MONITOR_H */
