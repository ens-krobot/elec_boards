/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _BUZZER_H_
#define _BUZZER_H_

#ifdef __cplusplus
extern "C" {
#endif
  void buzzInit(void);
  void buzzPlay(uint32_t freq, systime_t duration);
  void buzzPlayWait(uint32_t freq, systime_t duration);
#ifdef __cplusplus
}
#endif

extern EventSource BuzzerSilentEventSource;

#endif /* _BUZZER_H_ */
