// util.h
// Small helper functions shared among modules.
// ======================================================================

#ifndef UTIL_H
#define UTIL_H

#include "messages.h" // for DroneStateMsg
#include "params.h"   // for SimParams
#include <stdbool.h>
#include "obstacles.h"   // add this near the top (after guards)



// Print error message (with errno) and exit.
void die(const char *msg);

// Integer max (tiny helper for select()).
int  imax(int a, int b);

// Map the keys (w,e,r,s,d,f,x,c,v) to corresponding unit direction increments (dFx, dFy).
void direction_from_key(char key, double *dFx, double *dFy);


// Represent one of the 8 discrete directions.
typedef struct {
    char   key;  // 'w','e','r','s','f','x','c','v'
    double ux;   // unit vector x-component
    double uy;   // unit vector y-component
} Dir8;


// Normalization factor for diagonals: 1/sqrt(2).
extern const double INV_SQRT2;

// Table of the 8 unit directions, aligned with the key cluster:
//
//     w e r
//     s d f
//     x c v
//
// Directions (ux, uy) are unit-length (diagonals use INV_SQRT2).
extern const Dir8 g_dir8[8];

// Simple 2D dot product helper: dot = (ax,ay) · (bx,by).
double dot2(double ax, double ay, double bx, double by);

// Given a vector (Px, Py), find the index (0..7) of the direction in
// g_dir8[] which has the largest *positive* dot product with (Px,Py).
// Returns -1 if all dot products are <= 0 (i.e., no "good" direction).
int best_dir8_for_vector(double Px, double Py);

// ----------------------------------------------------------------------
// Wall repulsion field (Khatib-like), used by B to compute a repulsive
// vector P from the 4 geo-fence walls.
// ----------------------------------------------------------------------
//
// Walls are at x = ±world_half, y = ±world_half.
// Parameters:
//   - wall_clearance: distance within which repulsion is active.
//   - wall_gain: gain factor for the repulsive magnitude.
//
// For a given drone state, compute_wall_repulsive_P fills (Px,Py) with
// the total repulsive vector from the 4 walls.
// If repulsion is disabled (clearance or gain <= 0), it returns (0,0).
void compute_wall_repulsive_P(const DroneStateMsg *s,
                              const SimParams    *params,
                              double *Px, double *Py);


/* extern const Dir8 g_dir8[8];
extern const double INV_SQRT2;

// Vector math
double dot2(double ax, double ay, double bx, double by);

// Repulsion field from walls
void compute_wall_repulsive_P(const DroneStateMsg *s,
                              const SimParams    *params,
                              double *Px, double *Py);

// Find best 8-direction index
int best_dir8_for_vector(double Px, double Py); */


// ...

// Unified repulsive field:
//   - can include walls (geo-fence)
//   - can include point obstacles
//
// include_walls      : if true, add wall repulsion
// include_obstacles  : if true, add obstacle repulsion
//
// obs / num_obs      : obstacle list (ignored if include_obstacles == false)
void compute_repulsive_P(const DroneStateMsg *s,
                         const SimParams     *params,
                         const Obstacle      *obs,
                         int                  num_obs,
                         bool                 include_walls,
                         bool                 include_obstacles,
                         double              *Px,
                         double              *Py);

void compute_obstacles_repulsive_P(const DroneStateMsg *s,
                                   const SimParams     *params,
                                   const Obstacle      *obs,
                                   int                  num_obs,
                                   double              *Px,
                                   double              *Py);


#endif // UTIL_H
