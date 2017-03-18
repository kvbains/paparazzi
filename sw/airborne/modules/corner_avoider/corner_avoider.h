/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/corner_avoider/corner_avoider.h"
 * @author Group 1
 * Corner avoidance using fast9
 */

#ifndef CORNER_AVOIDER_H
#define CORNER_AVOIDER_H
#include <inttypes.h>
#include "state.h"

extern uint8_t safeToGoForwards;
extern float incrementForAvoidance;
extern uint16_t trajectoryConfidence;
extern void corner_avoider_init(void);
extern void corner_avoider_periodic(void);
extern uint8_t moveWaypointForward(uint8_t, float);
extern uint8_t moveWaypoint(uint8_t, struct EnuCoor_i *);
extern uint8_t increase_nav_heading(int32_t *, float);
extern uint8_t chooseRandomIncrementAvoidance(void);

#endif

