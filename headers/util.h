// util.h
// Helper functions shared among modules
// ======================================================================

#ifndef UTIL_H
#define UTIL_H

#include "messages.h" // for DroneStateMsg
#include "params.h"   // for SimParams
#include <stdbool.h>
#include "obstacles.h"   
#include "targets.h"   

#include <stdio.h>
#include <unistd.h>
#include <errno.h> 
#include <stdlib.h>

// Prints error message (with errno) and exit.
void die(const char *msg);

// Returns the maximum of two integers (tiny helper for select()).
int  imax(int a, int b);

// Maps the keys (w,e,r,s,d,f,x,c,v) to corresponding unit direction increments (dFx, dFy).
void direction_from_key(char key, double *dFx, double *dFy);


// Represents one of the 8 possible key directions.
typedef struct {
    char   key;  // 'w','e','r','s','f','x','c','v'
    double ux;   // unit vector x-component
    double uy;   // unit vector y-component
} Dir8;

// Any object with x, y, active : to be treated as PointLike for latter calculated distance b/n the pointLikes
typedef struct {
    double x;
    double y;
    int    active;
} PointLike;


// Normalization factor for diagonals: 1/sqrt(2).
extern const double INV_SQRT2;

// Table of the 8 possible vecotrs, aligned with the keyboard key cluster:
//     w e r
//     s d f
//     x c v
// Directions (ux, uy) will be unit-length
extern const Dir8 g_dir8[8];

// Finds the index (0-7) of the direction in g_dir8[] which has the largest *positive* dot product with (Px,Py).
// Returns -1 if all dot products are <= 0 to mean no "good" allignment found.
int best_dir8_for_vector(double Px, double Py);

// Does dot product of two vectors
double dot2(double ax, double ay, double bx, double by);

// Computes total force vector using a "virtual key" computed from obstacles or walls
void send_total_force_to_d(const ForceStateMsg *user_force,
                                  const DroneStateMsg *cur_state,
                                  const SimParams     *params,
                                  const Obstacle      *obs,
                                  int                  num_obs,
                                  int                  fd_to_d,
                                  FILE                *logfile,
                                  const char          *reason);

// Computes unified repulsive field from point obstacles
void compute_repulsive_P(const DroneStateMsg *s,
                         const SimParams     *params,
                         const Obstacle      *obs,
                         int                  num_obs,
                         bool                 include_walls,
                         bool                 include_obstacles,
                         double              *Px,
                         double              *Py);

// Checks if a point (x,y) is too close to the walls.
int target_too_close_to_wall(double x,
                                    double y,
                                    const SimParams *params,
                                    double wall_margin);

// Checks if (x,y) is too close to any active obstacle.
int too_close_to_any_pointlike(double px,
                               double py,
                               const PointLike *arr,
                               int count,
                               double min_dist);

// Checks if the drone has "hit" any active target.
int check_target_hits(const DroneStateMsg *cur_state,
                      Target              *targets,
                      int                  num_targets,
                      const SimParams     *params,
                      int                 *score,
                      int                 *targets_collected,
                      int                 *last_hit_step,
                      int                  current_step);


// Helper to perform uniform random double in [min, max].
double rand_in_range(double min, double max);
#endif // UTIL_H
