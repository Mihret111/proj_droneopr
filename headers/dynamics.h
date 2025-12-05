// dynamics.h
// Interface for the dynamics process (D)
// ======================================================================
// #define _POSIX_C_SOURCE 199309L  // to solve the nanosleep warning(telling the headers to expose the POSIX nanosleep prototype)

#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "params.h"

// Runs the dynamics process:
//   - Reads ForceStateMsg from force_fd (from B)
//   - Integrates dynamics
//   - Sends DroneStateMsg to state_fd (to B)
void run_dynamics_process(int force_fd, int state_fd, SimParams params);

#endif // DYNAMICS_H

