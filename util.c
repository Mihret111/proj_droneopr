// util.c
// shared helper functions
// ======================================================================

#include "headers/util.h"
#include "headers/messages.h" // for DroneStateMsg
#include "headers/params.h"   // for SimParams
#include "headers/obstacles.h"

#include <math.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>

// Return max of two ints.
// ----------------------------------------------------------------------
int imax(int a, int b) {
    return (a > b) ? a : b;
}


// Print error  and terminate program.
// ----------------------------------------------------------------------
void die(const char *msg) {
    perror(msg);
    exit(EXIT_FAILURE);
}

// Convert a key to direction increments in the 9-key cluster.
// The resulting increments are multiplied by force_step to build actual force.
// ----------------------------------------------------------------------
void direction_from_key(char key, double *dFx, double *dFy) {
    *dFx = 0.0;
    *dFy = 0.0;

    switch (key) {
        case 'w': *dFx = -1; *dFy = +1; break; // up-left
        case 'e': *dFx =  0; *dFy = +1; break; // up
        case 'r': *dFx = +1; *dFy = +1; break; // up-right

        case 's': *dFx = -1; *dFy =  0; break; // left
        case 'd': *dFx =  0; *dFy =  0; break; // brake (handled specially)
        case 'f': *dFx = +1; *dFy =  0; break; // right

        case 'x': *dFx = -1; *dFy = -1; break; // down-left
        case 'c': *dFx =  0; *dFy = -1; break; // down
        case 'v': *dFx = +1; *dFy = -1; break; // down-right

        default:
            // any other key results in no directional change
            break;
    }
}

// ----------------------------------------------------------------------
// 8 discrete directions and vector helpers
// ----------------------------------------------------------------------

// 1/sqrt(2) for diagonals
const double INV_SQRT2 = 0.7071067811865475;

// Order is arbitrary but fixed; we also store the corresponding key.
const Dir8 g_dir8[8] = {
    { 'w', -INV_SQRT2, +INV_SQRT2 },  // up-left
    { 'e',  0.0,        +1.0       }, // up
    { 'r', +INV_SQRT2, +INV_SQRT2 },  // up-right
    { 's', -1.0,        0.0       },  // left
    { 'f', +1.0,        0.0       },  // right
    { 'x', -INV_SQRT2, -INV_SQRT2 },  // down-left
    { 'c',  0.0,        -1.0      },  // down
    { 'v', +INV_SQRT2, -INV_SQRT2 }   // down-right
};

// Simple 2D dot product: (ax,ay) dot (bx,by)
double dot2(double ax, double ay, double bx, double by) {
    return ax * bx + ay * by;
}

// Given a vector (Px, Py), find the index (0..7) of the direction in
// g_dir8 with largest positive dot product with (Px,Py).
// If all dot products are <= 0, return -1.
int best_dir8_for_vector(double Px, double Py) {
    double best_dot = 0.0;
    int    best_idx = -1;

    for (int i = 0; i < 8; ++i) {
        double dot = dot2(Px, Py, g_dir8[i].ux, g_dir8[i].uy);
        if (dot > best_dot) {
            best_dot = dot;
            best_idx = i;
        }
    }
    return best_idx;
}

// ----------------------------------------------------------------------
// Wall repulsion field (Khatib-like) for the 4 borders.
// ----------------------------------------------------------------------
//
// Walls are at x = ±world_half, y = ±world_half.
// For each wall, if distance d < wall_clearance, we add a repulsive
// magnitude ~ wall_gain * (1/d - 1/clearance) in the direction away
// from the wall. Otherwise no contribution.
// This is a simplified but consistent model with Latombe/Khatib-style
// potential fields.
//
// Returned vector (Px,Py) is the sum of contributions from 4 walls.
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// Unified Khatib-style repulsive field from walls AND/OR obstacles.
//
// - Walls:
//     x = ±world_half, y = ±world_half
//     clearance = params->wall_clearance
//     gain      = params->wall_gain
//
// - Obstacles: radial field from each point obstacle.
//
// You can enable/disable each contribution with include_walls,
// include_obstacles flags.
// ----------------------------------------------------------------------
void compute_repulsive_P(const DroneStateMsg *s,
                         const SimParams     *params,
                         const Obstacle      *obs,
                         int                  num_obs,
                         bool                 include_walls,
                         bool                 include_obstacles,
                         double              *Px,
                         double              *Py)
{
    const double eps = 1e-3;
    include_walls = false;
    *Px = 0.0;
    *Py = 0.0;

    // -------------------- WALL REPULSION --------------------
    if (include_walls) {
        double world_half     = params->world_half;
        double wall_clearance = params->wall_clearance;
        double wall_gain      = params->wall_gain;

        if (wall_clearance > 0.0 && wall_gain > 0.0) {
            // Right wall at x = +world_half
            double d_right = world_half - s->x;
            if (d_right < wall_clearance) {
                if (d_right < eps) d_right = eps;
                double mag = wall_gain * (1.0/d_right - 1.0/wall_clearance);
                if (mag < 0.0) mag = 0.0;
                // Push left
                *Px -= mag; // py=0 here 
            }

            // Left wall at x = -world_half
            double d_left = world_half + s->x;
            if (d_left < wall_clearance) {
                if (d_left < eps) d_left = eps;
                double mag = wall_gain * (1.0/d_left - 1.0/wall_clearance);
                if (mag < 0.0) mag = 0.0;
                // Push right
                *Px += mag;
            }

            // Top wall at y = +world_half
            double d_top = world_half - s->y;
            if (d_top < wall_clearance) {
                if (d_top < eps) d_top = eps;
                double mag = wall_gain * (1.0/d_top - 1.0/wall_clearance);
                if (mag < 0.0) mag = 0.0;
                // Push down
                *Py -= mag;
            }

            // Bottom wall at y = -world_half
            double d_bottom = world_half + s->y;
            if (d_bottom < wall_clearance) {
                if (d_bottom < eps) d_bottom = eps;
                double mag = wall_gain * (1.0/d_bottom - 1.0/wall_clearance);
                if (mag < 0.0) mag = 0.0;
                // Push up
                *Py += mag;
            }
        }
    }

    // ------------------ OBSTACLE REPULSION -------------------
    if (include_obstacles && obs && num_obs > 0) {
        // For now we use fixed obstacle params derived from world size.
        // If you want, you can later add obs_clearance/obs_gain into SimParams.
         const double obs_clearance = params->world_half * 0.35;
        const double obs_gain      = 120.0;   // you said 120 behaved well
        //const double obs_clearance = params->wall_clearance;
        //const double obs_gain      = params-> wall_gain;
        if (obs_clearance <= 0.0 || obs_gain <= 0.0) {
            return;
        }

        for (int k = 0; k < num_obs; ++k) {
            if (!obs[k].active) continue;  // skip inactive obstacles

            double ox = obs[k].x;
            double oy = obs[k].y;

            double dx  = s->x - ox;
            double dy  = s->y - oy;
            double rho = sqrt(dx*dx + dy*dy);

            if (rho < eps) {
                rho = eps;
            }

            if (rho < obs_clearance) {
                double mag = obs_gain * (1.0/rho - 1.0/obs_clearance);
                if (mag < 0.0) mag = 0.0;

                double ux = dx / rho;
                double uy = dy / rho;

                *Px += mag * ux;
                *Py += mag * uy;
            }
        }
    }
}



/* // Walls-only wrapper (what D currently uses)
void compute_wall_repulsive_P(const DroneStateMsg *s,
                              const SimParams    *params,
                              double *Px, double *Py)
{
    compute_repulsive_P(s, params,
                        NULL,      // no obstacles
                        0,
                        true,      // include_walls
                        false,     // include_obstacles
                        Px, Py);
} */

// Obstacles-only wrapper (what B uses for virtual-key logic)
void compute_obstacles_repulsive_P(const DroneStateMsg *s,
                                   const SimParams     *params,
                                   const Obstacle      *obs,
                                   int                  num_obs,
                                   double              *Px,
                                   double              *Py)
{
    compute_repulsive_P(s, params,
                        obs,
                        num_obs,
                        false,     // include_walls
                        true,      // include_obstacles
                        Px, Py);
}

//A unified function: for repulsive forces


// ----------------------------------------------------------------------
// Wall repulsion field (Khatib-like) for the 4 borders.
// ----------------------------------------------------------------------
//
// Walls are at x = ±world_half, y = ±world_half.
// For each wall, if distance d < wall_clearance, we add a repulsive
// magnitude ~ wall_gain * (1/d - 1/clearance) in the direction away
// from the wall. Otherwise no contribution.
// This is a simplified but consistent model with Latombe/Khatib-style
// potential fields.
//
// Returned vector (Px,Py) is the sum of contributions from 4 walls.
// ----------------------------------------------------------------------
void compute_wall_repulsive_P(const DroneStateMsg *s,
                              const SimParams    *params,
                              double *Px, double *Py)
{
    double world_half     = params->world_half;
    double wall_clearance = params->wall_clearance;
    double wall_gain      = params->wall_gain;

    *Px = 0.0;
    *Py = 0.0;

    if (wall_clearance <= 0.0 || wall_gain <= 0.0) {
        // Repulsion disabled by parameters.
        return;
    }

    const double eps = 1e-3; // to avoid division by zero

    // ---- Right wall at x = +world_half ----
    double d_right = world_half - s->x;  // distance from drone to right wall
    if (d_right < wall_clearance) {
        if (d_right < eps) d_right = eps;
        double mag = wall_gain * (1.0/d_right - 1.0/wall_clearance);
        if (mag < 0.0) mag = 0.0;
        // Repulsive direction: push LEFT → (-1,0)
        *Px -= mag;
    }

    // ---- Left wall at x = -world_half ----
    double d_left = world_half + s->x;   // distance from drone to left wall
    if (d_left < wall_clearance) {
        if (d_left < eps) d_left = eps;
        double mag = wall_gain * (1.0/d_left - 1.0/wall_clearance);
        if (mag < 0.0) mag = 0.0;
        // Repulsive direction: push RIGHT → (+1,0)
        *Px += mag;
    }

    // ---- Top wall at y = +world_half ----
    double d_top = world_half - s->y;
    if (d_top < wall_clearance) {
        if (d_top < eps) d_top = eps;
        double mag = wall_gain * (1.0/d_top - 1.0/wall_clearance);
        if (mag < 0.0) mag = 0.0;
        // Repulsive direction: push DOWN → (0, -1)
        *Py -= mag;
    }

    // ---- Bottom wall at y = -world_half ----
    double d_bottom = world_half + s->y;
    if (d_bottom < wall_clearance) {
        if (d_bottom < eps) d_bottom = eps;
        double mag = wall_gain * (1.0/d_bottom - 1.0/wall_clearance);
        if (mag < 0.0) mag = 0.0;
        // Repulsive direction: push UP → (0, +1)
        *Py += mag;
    }
}


// Check if a point (x,y) is too close to the walls.
static int target_too_close_to_wall(double x,
                                    double y,
                                    const SimParams *params,
                                    double wall_margin)
{
    int insp_width = 15;               // was 35
    double wh = params->world_half;
    double mod_wh = wh - wall_margin+insp_width;   
    // Distance to right wall = wh - x (if x > 0)
    // Distance to left  wall = wh + x   (if x < 0)
    // But easiest: distance in x is wh - |x|
    double dx_to_wall = mod_wh - fabs(x);
    double dy_to_wall = wh - fabs(y);

    if (dx_to_wall < wall_margin) return 1;
    if (dy_to_wall < wall_margin) return 1;

    return 0;
}

// Check if (x,y)of a target/obstacle is too close to any active obstacle/ target.. casted to the point-like struct for refactoring
int too_close_to_any_pointlike(double px,
                               double py,
                               const PointLike *arr,
                               int count,
                               double min_dist)
{
    double min_d2 = min_dist * min_dist;

    for (int i = 0; i < count; ++i) {
        if (!arr[i].active)
            continue;

        double dx = px - arr[i].x;
        double dy = py - arr[i].y;

        if (dx*dx + dy*dy <= min_d2)
            return 1;
    }
    return 0;
}


