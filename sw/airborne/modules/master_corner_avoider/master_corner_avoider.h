/*
 * Copyright (C) GROUP 1
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/master_corner_avoider/master_corner_avoider.h"
 * @author GROUP 1
 * Links the corner detector software to the camera and makes it see the corners
 */

#ifndef MASTER_CORNER_AVOIDER_H
#define MASTER_CORNER_AVOIDER_H

#define MAX_POINTS 10000

#include <stdint.h>
#include "modules/computer_vision/cv.h"

// Module functions
extern void mca_init( void );

extern uint8_t  threshold; 				
extern uint16_t min_dist; 				

extern uint16_t x_padding; 				
extern uint16_t y_padding; 				

extern uint16_t num_corners; 			
extern uint16_t ret_corners_length; 	

extern struct point_t ret_corners[MAX_POINTS];

extern struct video_listener *mca_listener;

extern int corner_count;

#endif

