// params.h
// Defines simulation parameters 
// Stores parameters loaded once in main() and passed by value into B and D
// ======================================================================

#ifndef PARAMS_H
#define PARAMS_H

typedef struct {
    double mass;        // Mass of the drone
    double visc;        // Viscous friction coefficient
    double dt;          // Timestep (seconds)
    double force_step;  // Force increment for each directional key
    double world_half;  // World range: x,y ∈ [-world_half, +world_half]

    double wall_clearance; // Distance from wall where repulsion starts
    double wall_gain;      // Strength of repulsive force
} SimParams;

// Sets default values- just in case params.txt is not found
void init_default_params(SimParams *p);

// Overrides default values with values from params.txt, if present.
void load_params_from_file(const char *filename, SimParams *p);

#endif // PARAMS_H
