/*
 * flow_field_computation.c
 *
 *  Created on: Jul 18, 2016
 *      Author: bas
 */

#include "flow_field_estimation.h"
#include "mcu_periph/sys_time.h"

float FLT_MIN_RESOLUTION = 1e-3;


void updateFlowStats(struct flowStats* s, struct flowEvent e, struct flowField lastField,
    float filterTimeConstant, float movingAverageWindow, float maxSpeedDifference,
    struct cameraIntrinsicParameters intrinsics) {
  static int32_t tLast = 0;

  // X,Y are defined around the camera's principal point
  float x = (float) e.x - intrinsics.principalPointX;
  float y = (float) e.y - intrinsics.principalPointY;
  float u = e.u;
  float v = e.v;

  // Dot product of position/flow
  float w = x * u + y * v;
  // Squared flow magnitude
  float m = u * u + v * v;

  // Update stats through hybrid moving averaging/low-pass filtering
  float dt = ((float)(e.t - tLast))/1e6;
  if (dt <= 0) {
    dt = 1e-6;
  }
  float tFactor =  dt / filterTimeConstant;
  // To prevent very large updates after a large dt, the update reduces to a moving average
  if (tFactor > 1/movingAverageWindow)
    tFactor = 1/movingAverageWindow;

  // Limit outlier influence based on predicted flow magnitude
  float p[3] = {
      lastField.wx * intrinsics.focalLengthX,
      lastField.wy * intrinsics.focalLengthY,
      lastField.D
  };
  float mPred = p[0]*u + p[1]*v + p[2]*w;
  float dm = m - mPred;
  float dmMax = powf(maxSpeedDifference,2);
  if (fabsf(dm) > dmMax) {
    tFactor *= fabsf(dmMax/dm);
  }

  // Update flow statistics
  s->mx  += (x     - s->mx ) * tFactor;
  s->mu  += (u     - s->mu ) * tFactor;
  s->mxx += (x * x - s->mxx) * tFactor;
  s->mxu += (x * u - s->mxu) * tFactor;
  s->muu += (u * u - s->muu) * tFactor;

  s->my  += (y   - s->my ) * tFactor;
  s->mv  += (v   - s->mv ) * tFactor;
  s->myy += (y * y - s->myy) * tFactor;
  s->myv += (y * v - s->myv) * tFactor;
  s->mvv += (v * v - s->mvv) * tFactor;

  s->mxy += (x * y - s->mxy) * tFactor;

  s->eventRate += (1/dt - s->eventRate) * tFactor;

  tLast = e.t;
}

enum updateStatus recomputeFlowField(struct flowField* field, struct flowStats* s,
    float minEventRate, float minPosVariance, float maxFlowResidual,
    struct cameraIntrinsicParameters intrinsics) {

  // Quality checking for event rate
  if (s->eventRate < minEventRate) {
    return UPDATE_WARNING_RATE;
  }
  // Compute position variances from mean statistics
  float varX  = s->mxx - pow(s->mx,2);
  float varY  = s->myy - pow(s->my,2);
  float covXY = s->mxy - s->mx*s->my;

  // Compute variance eigenvalues (i.e. spread independent of x/y coordinates)
  float d = powf(varX - varY,2) + 4*powf(covXY,2);
  if (d < 0) {
    return UPDATE_WARNING_SPREAD;
  }
  float eig1 = (varX + varY + sqrtf(d))/2;
  float eig2 = (varX + varY - sqrtf(d))/2;
  if (eig1 < minPosVariance || eig2 < minPosVariance) {
    return UPDATE_WARNING_SPREAD;
  }

  /*
   * The linear system to be solved is:
   *
   * [1,  sx,      0     [p[0]    [su
   *  sx, sxx+syy, sy  *  p[1]  =  sxu+syv
   *  0,  sy,      1]     p[2]]    sv]
   *
   *  where the entries are weighted by orientation.
   */

  // Compute determinant
  float D = pow(s->mx,2) + pow(s->my,2) - s->mxx - s->myy;
  if (fabs(D) < FLT_MIN_RESOLUTION) {
    return UPDATE_WARNING_SINGULAR;
  }
  float p[3];
  p[0] = 1/D*(s->mu*s->my*s->my - s->mv*s->mx*s->my - s->mu*s->mxx
      - s->mu*s->myy + s->mxu*s->mx + s->mx*s->myv);
  p[1] = 1/D*(s->mv*s->mx*s->mx - s->mu*s->my*s->mx - s->mv*s->mxx
      - s->mv*s->myy + s->mxu*s->my + s->myv*s->my);
  p[2] = 1/D*(s->mu*s->mx + s->mv*s->my - s->mxu - s->myv);

  // To check coherence in the flow field, we compute the
  // Normalized Mean Squared Residuals
  float msRes = s->muu - 2*p[0]*s->mu - 2*p[2]*s->mxu + p[0]*p[0] + 2*p[0]*p[2]*s->mx + p[2]*p[2]*s->mxx
      + s->mvv - 2*p[1]*s->mv - 2*p[2]*s->myv + p[1]*p[1] + 2*p[1]*p[2]*s->my + p[2]*p[2]*s->myy;
  float nmsRes = msRes / (0.0001+s->muu + s->mvv);
  if (nmsRes > maxFlowResidual) {
    return UPDATE_WARNING_RESIDUAL;
  }

  // Convert solution to ventral flow and divergence using camera intrinsics
  field->wx = p[0]/intrinsics.focalLengthX;
  field->wy = p[1]/intrinsics.focalLengthY;
  field->D  = p[2];

  // If no problem was found, update is successful
  return UPDATE_SUCCESS;
}


void derotateFlowField(struct flowField* field, struct FloatRates* rates) {
  float p = rates->p;
  float q = rates->q;
  field->wxDerotated = field->wx - p;
  field->wyDerotated = field->wy + q;
}
