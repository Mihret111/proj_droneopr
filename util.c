// util.c
// Shared helper functions
// ======================================================================

#include "headers/util.h"
#include "headers/messages.h" // for DroneStateMsg
#include "headers/params.h"   // for SimParams
#include "headers/obstacles.h"
#include "headers/targets.h"

#include <math.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>

// Returns max of two ints.
// ----------------------------------------------
int imax(int a, int b) {
    return (a > b) ? a : b;
}


// Prints error and terminates program.
// ----------------------------------------------
void die(const char *msg) {
    perror(msg);
    exit(EXIT_FAILURE);
}

// Converts a key to direction increments in the 9-key cluster.
// Multiplies resulting increments by force_step to build actual force.
// ----------------------------------------------
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

// 1/sqrt(2) 
const double INV_SQRT2 = 0.7071067811865475;

// Stores the corresponding key
// ----------------------------------------------
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

// Calculates simple 2D dot product: (ax,ay) dot (bx,by)
// ----------------------------------------------
double dot2(double ax, double ay, double bx, double by) {
    return ax * bx + ay * by;
}

// Compute the maximum dot product between a vector and the 8-directions
// ------------------ --------------------------------------------------------------
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

// Sends total force to D using a "virtual key" computed from obstacles and walls
// ----------------------------------------------------------------------
void send_total_force_to_d(const ForceStateMsg *user_force,
                                  const DroneStateMsg *cur_state,
                                  const SimParams     *params,
                                  const Obstacle      *obs,
                                  int                  num_obs,
                                  int                  fd_to_d,
                                  FILE                *logfile,
                                  const char          *reason)
{
    // Computes repulsive force vector 
    double Px = 0.0, Py = 0.0;
    compute_repulsive_P(cur_state,
                        params,
                        obs,
                        num_obs,
                        false,   // calculate repulsive force for obstacles here
                        true,   // include_obstacles
                        &Px, &Py);

    // Sends user_force alone if very small.
    double Pnorm2 = Px*Px + Py*Py;
    if (Pnorm2 < 1e-6) {
        ForceStateMsg out = *user_force;
        if (write(fd_to_d, &out, sizeof(out)) == -1) {
            perror("[B] write to D failed (no rep)");
        } else if (logfile) {
            fprintf(logfile,
                    "SEND_FORCE (%s): userFx=%.2f userFy=%.2f, "
                    "P ~ 0 -> Fx=%.2f Fy=%.2f\n",
                    reason ? reason : "?",
                    user_force->Fx, user_force->Fy,
                    out.Fx, out.Fy);
            fflush(logfile);
        }
        return;
    }

    // Finds discrete direction that best matches the repulsion P
    int idx = best_dir8_for_vector(Px, Py);  // util.c
    if (idx < 0) {
        // Falls back to user-only command if no good direction
        ForceStateMsg out = *user_force;
        if (write(fd_to_d, &out, sizeof(out)) == -1) {
            perror("[B] write to D failed (no good dir)");
        } else if (logfile) {
            fprintf(logfile,
                    "SEND_FORCE (%s): userFx=%.2f userFy=%.2f, "
                    "P=(%.2f,%.2f), no good dir -> Fx=%.2f Fy=%.2f\n",
                    reason ? reason : "?",
                    user_force->Fx, user_force->Fy,
                    Px, Py,
                    out.Fx, out.Fy);
            fflush(logfile);
        }
        return;
    }

    char   best_key = g_dir8[idx].key;
    double best_dot = dot2(Px, Py, g_dir8[idx].ux, g_dir8[idx].uy);   // the maximum projection of P on the direction vector

    if (best_dot <= 0.0) {
        // Same: Falls back to user-only command if projection is not positive
        ForceStateMsg out = *user_force;
        if (write(fd_to_d, &out, sizeof(out)) == -1) {
            perror("[B] write to D failed (best_dot<=0)");
        } else if (logfile) {
            fprintf(logfile,
                    "SEND_FORCE (%s): userFx=%.2f userFy=%.2f, "
                    "P=(%.2f,%.2f), best_dot<=0 -> Fx=%.2f Fy=%.2f\n",
                    reason ? reason : "?",
                    user_force->Fx, user_force->Fy,
                    Px, Py,
                    out.Fx, out.Fy);
            fflush(logfile);
        }
        return;
    }

    // Converts best_dot (intensity of P) into 8 key steps
    double step_force = params->force_step;
    double n_steps_f  = best_dot / (step_force + 1e-9);

    int n_steps = (int)(n_steps_f + 0.5);    // round to nearest integer
    // if (n_steps < 1) n_steps = 1;
    // if (n_steps > 3) n_steps = 3;   // Tunable

    // Gets direction from key
    double dFx, dFy;
    direction_from_key(best_key, &dFx, &dFy);

    double Fvk_x = n_steps * dFx * step_force;
    double Fvk_y = n_steps * dFy * step_force;

    // Combines user + virtual-key repulsion
    ForceStateMsg out = *user_force;
    out.Fx += Fvk_x;
    out.Fy += Fvk_y;

    // Sends to D
    if (write(fd_to_d, &out, sizeof(out)) == -1) {
        perror("[B] write to D failed (virtual key rep)");
    } else if (logfile) {
        fprintf(logfile,
                "SEND_FORCE (%s): userFx=%.2f userFy=%.2f, "
                "P=(%.2f,%.2f), best_key=%c, n_steps=%d "
                "Fvk=(%.2f,%.2f) => Fx=%.2f Fy=%.2f\n",
                reason ? reason : "?",
                user_force->Fx, user_force->Fy,
                Px, Py,
                best_key, n_steps,
                Fvk_x, Fvk_y,
                out.Fx, out.Fy);
        fflush(logfile);
    }
}

// Computes ontinuous repulsive force vector
// ------------------ --------------------------------------------------------------
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
    *Px = 0.0;
    *Py = 0.0;

    
    // Computes wall repulsion force
    // Returns vector (Px,Py) as the sum of contributions for the 4 borders
    // ------------------ --------------------------------------------------------------
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
                // Pushes left
                *Px -= mag; // py=0 here 
            }

            // Left wall at x = -world_half
            double d_left = world_half + s->x;
            if (d_left < wall_clearance) {
                if (d_left < eps) d_left = eps;
                double mag = wall_gain * (1.0/d_left - 1.0/wall_clearance);
                if (mag < 0.0) mag = 0.0;
                // Pushes right
                *Px += mag;
            }

            // Top wall at y = +world_half
            double d_top = world_half - s->y;
            if (d_top < wall_clearance) {
                if (d_top < eps) d_top = eps;
                double mag = wall_gain * (1.0/d_top - 1.0/wall_clearance);
                if (mag < 0.0) mag = 0.0;
                // Pushes down
                *Py -= mag;
            }

            // Bottom wall at y = -world_half
            double d_bottom = world_half + s->y;
            if (d_bottom < wall_clearance) {
                if (d_bottom < eps) d_bottom = eps;
                double mag = wall_gain * (1.0/d_bottom - 1.0/wall_clearance);
                if (mag < 0.0) mag = 0.0;
                // Pushes up
                *Py += mag;
            }
        }
    }

    // Computes wall repulsion force continuous force(later mapped to the directions of the key cluster)
    // Uses fixed obstacle params derived from world size
    // ------------------ --------------------------------------------------------------
    if (include_obstacles && obs && num_obs > 0) {
         const double obs_clearance = params->world_half * 0.30;
        const double obs_gain      = 120.0;   // 120 behaved well
        if (obs_clearance <= 0.0 || obs_gain <= 0.0) {
            return;
        }

        for (int k = 0; k < num_obs; ++k) {
            if (!obs[k].active) continue;  // Skips inactive obstacles

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

// Checks if the drone has "hit" any active target.
// Returns: number of targets collected in this call (0 or more).
// ------------------------------------------------------------------
int check_target_hits(const DroneStateMsg *cur_state,
                      Target              *targets,
                      int                  num_targets,
                      const SimParams     *params,
                      int                 *score,
                      int                 *targets_collected,
                      int                 *last_hit_step,
                      int                  current_step)
{
    // Hitting radius in world units
    double R_hit  = params->world_half * 0.08;           // 8% of world half-range.
    double R_hit2 = R_hit * R_hit;

    double px = cur_state->x;
    double py = cur_state->y;

    int hits = 0;

    for (int i = 0; i < num_targets; ++i) {
        if (!targets[i].active)
            continue;

        double dx = px - targets[i].x;
        double dy = py - targets[i].y;
        double d2 = dx*dx + dy*dy;

        if (d2 <= R_hit2) {
            // once target is hit, deactivate it
            targets[i].active     = 0;
            targets[i].life_steps = 0;

            // Updates counters if pointers provided
            if (score)             (*score)++;
            if (targets_collected) (*targets_collected)++;
            if (last_hit_step)     (*last_hit_step) = current_step;

            hits++;
        }
    }

    return hits;
}


// Helper to perform uniform random double : used in obs and target generation
double rand_in_range(double min, double max) {
    double u = (double)rand() / (double)RAND_MAX;  // b/n [0,1]
    return min + u * (max - min);                  // linear interpolation
}



// ------------------ --------------------------------------------------------------
// Meaningfull spawning :                        
// Checks against cases which don't serve the purpose of the game
//------- --------------------------------------------------------------------------

// Checks if a candidate target spawning point (x,y) is too close to the walls
// (targets too close to the walls are unattainable)
// ------------------ --------------------------------------------------------------
int target_too_close_to_wall(double x,
                                    double y,
                                    const SimParams *params,
                                    double wall_margin)
{
    int insp_width = 35;              
    double wh = params->world_half;
    double mod_wh = wh - wall_margin+insp_width; 
    double dx_to_wall = mod_wh - fabs(x);
    double dy_to_wall = wh - fabs(y);

    if (dx_to_wall < wall_margin) return 1;
    if (dy_to_wall < wall_margin) return 1;

    return 0;
}

// Checks if position of target/obstacle is too close to any active obstacle/ target, respectively
// (targets with too close obstacles are unattainable )
// ------------------ --------------------------------------------------------------------------
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