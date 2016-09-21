/*
 * flow_field_computation.h
 *
 *  Created on: Jul 18, 2016
 *      Author: bas
 */

#ifndef SW_AIRBORNE_MODULES_EVENT_OPTIC_FLOW_FLOW_FIELD_ESTIMATION_H_
#define SW_AIRBORNE_MODULES_EVENT_OPTIC_FLOW_FLOW_FIELD_ESTIMATION_H_

#include <stdbool.h>
#include <inttypes.h>
#include "math/pprz_algebra_float.h"

/**
 * Flow event struct, simplified version of the cAER implementation.
 * It contains all fields passed from the DVS through UART.
 * Note that polarity and validity information are not transferred:
 * all received events are assumed to be valid.
 */
struct flowEvent {
  uint8_t x,y;
  int32_t t;
  float u,v;
};

/**
 * Flow field parameter struct.
 * Contains the parameters ventral flow and divergence
 */
struct flowField {
  float wx;
  float wy;
  float wxDerotated;
  float wyDerotated;
  float D;
  int32_t t;
};

/**
 * Flow statistics container.
 * These statistics are re-evaluated at every event and form the basis
 * for flow field recomputation. At all times they represent the mean of
 * N previous vector coordinates or the mean of cross-products of two
 * vector coordinates.
 *
 * E.g. 'm<x><x>' refers to \f$\sum_i^N {<x_i>*<x_i>} / N \f$.
 *
 * These mean values can be used to compute, among others, variance:
 * \f[
 * {\rm Var}{x} = (\sum_i^N {x_i^2} - \left(\sum_i^N {x_i}\right)^2) / N
 *      = mxx - mx^2
 * \f]
 * And similarly, covariance:
 * \f[
 * {\rm Cov}{x,y} = mxy - mx * my
 * \f]
 *
 * And ultimately they are used for computing the flow field as a
 * least-squares solution.
 */
struct flowStats {
  float mx, mu, mxx, mxu, muu;
  float my, mv, myy, myv, mvv;
  float mxy;
  float eventRate;
};

/**
 * Camera intrinsic parameters struct.
 *
 * Contains the principal point coordinates (x,y) and the focal lengths for x and y.
 * Note that these are scaled by
 */
struct cameraIntrinsicParameters {
  float principalPointX;
  float principalPointY;
  float focalLengthX;
  float focalLengthY;
};

/**
 * Indicator for flow field computation result in a module periodic function call
 */
enum updateStatus {
  UPDATE_SUCCESS,           // Successful flow field update
  UPDATE_NONE,              // No incoming data, no update
  UPDATE_STATS,             // Stats were updated - the flow field still needs to be recomputed
  UPDATE_WARNING_SINGULAR,  // No update, flow field system is singular
  UPDATE_WARNING_RATE,      // No update, event rate is too low
  UPDATE_WARNING_SPREAD,    // No update, there is too little spread in flow vector position
  UPDATE_WARNING_RESIDUAL   // No update, the flow field residuals are too large
};

/**
 * Performs an update of all flow field statistics with a new event.
 */
void updateFlowStats(struct flowStats* s, struct flowEvent e, struct flowField lastField,
    float filterTimeConstant, float movingAverageWindow, float maxSpeedDifference,
  struct cameraIntrinsicParameters intrinsics);

/**
 * Recomputation of the flow field using the latest statistics.
 */
enum updateStatus recomputeFlowField(struct flowField* field, struct flowStats* s,
    float minEventRate, float minPosVariance, float maxFlowResidual,
    struct cameraIntrinsicParameters intrinsics);

/**
 * Simple derotation of the optic flow field parameters.
 *
 * @param field The flow field to be derotated.
 * @param rates The input body rates.
 */
void derotateFlowField(struct flowField* field, struct FloatRates* rates);

#endif /* SW_AIRBORNE_MODULES_EVENT_OPTIC_FLOW_FLOW_FIELD_ESTIMATION_H_ */
