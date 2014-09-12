/*
 * Copyright (C) 2014 OpenUAS
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "obc2014.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/autopilot.h"

int obc_flight_mode = SCRUTENEERING_FLIGHT;

float set_airspeed_nominal = NOMINAL_AIRSPEED;
float set_airspeed_tracking = TRACKING_AIRSPEED;
float set_airspeed_glide = GLIDE_AIRSPEED;
float set_takeoff_pitch = TAKEOFF_PITCH_ANGLE;
float set_flare_pitch = FLARE_PITCH_ANGLE;


#ifdef SITL
float v_ctl_auto_throttle_nominal_cruise_pitch = 0;
float v_ctl_auto_airspeed_setpoint = 0;
float airspeed_quadratic_scale = 0;
float airspeed_offset = 0;
#endif

void periodic_obc(void)
{
  // Copy Radio commands in AUTO1
  if (pprz_mode == PPRZ_MODE_AUTO1)
  {
    ap_state->commands[COMMAND_HATCH] = fbw_state->channels[RADIO_HATCH];
    ap_state->commands[COMMAND_BRAKE] = fbw_state->channels[RADIO_BRAKE];
  }
}

#include "subsystems/gps.h"

bool_t gps_has_been_good(void)
{
  static bool_t gps_has_been_good = FALSE;
  if (GpsFixValid())
  {
    gps_has_been_good = TRUE;
  }
  return gps_has_been_good;
}
