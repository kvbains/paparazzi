/*
 * Copyright (C) C. De Wagter
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/sensors/kalamos_uart.c"
 * @author C. De Wagter
 * Parrot Kalamos Nvidia tk1 stereo vision uart (RS232) communication
 */

#include "modules/sensors/kalamos_uart.h"

#include "pprzlink/pprz_transport.h"
#include "pprzlink/intermcu_msg.h"
#include "mcu_periph/uart.h"
#include "subsystems/abi.h"
#include "subsystems/imu.h"
#include "state.h"
#include "subsystems/navigation/waypoints.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/gps.h"

#include "generated/flight_plan.h"

 /* Main magneto structure */
static struct kalamos_t kalamos = {
  .device = (&((KALAMOS_PORT).device)),
  .msg_available = false
};
static uint8_t mp_msg_buf[128]  __attribute__((aligned));   ///< The message buffer for the Kalamos

struct  Kalamos2PPRZPackage k2p_package;
float kalamos_search_height = 35.0;
float kalamos_descend_height = 20.0;
float kalamos_height_gain = 1.0;
bool kalamos_enable_landing = false;
bool kalamos_enable_spotsearch = false;
float kalamos_landing_decent_speed = 0.05;
float kalamos_pos_gain = 0.05;
int32_t zeroheight;
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void kalamos_raw_downlink(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DEBUG(trans, dev, AC_ID, sizeof(struct Kalamos2PPRZPackage), (unsigned char *) &k2p_package);
}
#endif

/* Initialize the Kalamos */
void kalamos_init() {
  // Initialize transport protocol
  pprz_transport_init(&kalamos.transport);


#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG, kalamos_raw_downlink);
#endif

  NavSetWaypointHere(WP_LANDING);
  k2p_package.height = -0.01;
  k2p_package.status = 1;


}

static int timeoutcount = 0;

/* Parse the InterMCU message */
static inline void kalamos_parse_msg(void)
{

  /* Parse the kalamos message */
  uint8_t msg_id = mp_msg_buf[1];

  switch (msg_id) {

  /* Got a kalamos message */
  case DL_IMCU_DEBUG: {
    uint8_t size = DL_IMCU_DEBUG_msg_length(mp_msg_buf);
    uint8_t *msg = DL_IMCU_DEBUG_msg(mp_msg_buf);

    unsigned char * tmp = (unsigned char*)&k2p_package;
    for(uint8_t i = 0; i < size; i++) {
      tmp[i] = msg[i];
    }
    timeoutcount = 100;

    struct EnuCoor_f *pos = stateGetPositionEnu_f();

    float diff_landing = (kalamos_descend_height - k2p_package.height)*kalamos_height_gain;

    if (gps.hmsl < 20000) // above 20 meters stereo not reliable
      diff_landing = 0;
    float pprzheight_landing  = pos->z + diff_landing;

    waypoint_set_alt(WP_LANDING,pprzheight_landing);

    float diff_search = (kalamos_search_height - k2p_package.height)*kalamos_height_gain;

    if (kalamos_enable_spotsearch) {
      waypoint_set_xy_i(WP_LANDING,POS_BFP_OF_REAL(k2p_package.land_enu_x), POS_BFP_OF_REAL(k2p_package.land_enu_y));
    }

    if (k2p_package.height < 5.0) {
        kalamos_enable_landing = false;
    }

    if (kalamos_enable_landing && timeoutcount > 0) {
      if (k2p_package.min_height > 5.0) {
        kalamos_descend_height -= kalamos_landing_decent_speed;
      }
/*
      struct FloatQuat *att = stateGetNedToBodyQuat_f();

      struct FloatRMat ltp_to_kalamos_rmat;
      float_rmat_of_quat(&ltp_to_kalamos_rmat, att);

      //x,y,z pos van joe
      struct FloatVect3 joe;
      joe.x = k2p_package.target_x;
      joe.y = k2p_package.target_y;
      joe.z = k2p_package.height;

      struct FloatVect3 measured_ltp;
      float_rmat_transp_vmult(&measured_ltp, &ltp_to_kalamos_rmat, &joe);

      waypoint_set_xy_i(WP_LANDING,POS_BFP_OF_REAL(measured_ltp.x), POS_BFP_OF_REAL(measured_ltp.y));
      */

    }
    waypoint_set_xy_i(WP_JOE,POS_BFP_OF_REAL(k2p_package.joe_enu_x), POS_BFP_OF_REAL(k2p_package.joe_enu_y));


    // Send ABI message
    if (timeoutcount > 0) {
      AbiSendMsgAGL(AGL_SONAR_ADC_ID, k2p_package.height);
    }

    break;
  }
    default:
      break;
  }
}

/* We need to wait for incomming messages */
void kalamos_event() {
  // Check if we got some message from the Kalamos
  pprz_check_and_parse(kalamos.device, &kalamos.transport, mp_msg_buf, &kalamos.msg_available);

  // If we have a message we should parse it
  if (kalamos.msg_available) {
    kalamos_parse_msg();
    kalamos.msg_available = false;
  }
}

void kalamos_periodic() {


  struct FloatEulers *attE = stateGetNedToBodyEulers_f();
  struct FloatQuat *att = stateGetNedToBodyQuat_f();
  struct EnuCoor_f *pos = stateGetPositionEnu_f();


  struct PPRZ2KalamosPackage p2k_package;
  p2k_package.phi = attE->phi;
  p2k_package.theta = attE->theta;
  p2k_package.psi = attE->psi;
  p2k_package.qi = att->qi;
  p2k_package.qx = att->qx;
  p2k_package.qy = att->qy;
  p2k_package.qz = att->qz;
  p2k_package.gpsx = pos->x;
  p2k_package.gpsy = pos->y;
  p2k_package.gpsz = (float)(gps.hmsl - zeroheight)/1000;
  p2k_package.enables = 0;
  if (kalamos_enable_landing)
    p2k_package.enables |= 0b1;
  if (kalamos_enable_spotsearch)
    p2k_package.enables |= 0b10;

  if (timeoutcount > 0) {
    timeoutcount--;
  } else {
    k2p_package.status = 1;
  }

  // Send Telemetry report
  DOWNLINK_SEND_KALAMOS(DefaultChannel, DefaultDevice, &k2p_package.status, &k2p_package.height,&k2p_package.min_height,&k2p_package.joe_enu_x,&k2p_package.joe_enu_y,&k2p_package.land_enu_x,&k2p_package.land_enu_y,&k2p_package.flow_x,&k2p_package.flow_y);


  pprz_msg_send_IMCU_DEBUG(&(kalamos.transport.trans_tx), kalamos.device,
                                         1, sizeof(struct PPRZ2KalamosPackage), (unsigned char *)(&p2k_package));
}

void enableLandingspotSearch(bool b) {
  kalamos_enable_spotsearch = b;
}

void enableKalamosDescent(bool b) {
  kalamos_enable_landing = b;
}


void setZeroHeight(void){
  zeroheight = gps.hmsl;
}

