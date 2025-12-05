// dynamics.c
// Defines the dynamics process (D)
// Integrates the drone state forward in time
// ======================================================================

#include "headers/dynamics.h"
#include "headers/messages.h"
#include "headers/util.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>     // exit, strtod
#include <errno.h>
#include <time.h>
#include <fcntl.h>

// ----------------------------------------------------------------------
// Defines the dynamics process:
//   - Reads ForceStateMsg from force_fd (from B)
//   - Integrates dynamics
//   - Sends DroneStateMsg to state_fd (to B)
// ----------------------------------------------------------------------
void run_dynamics_process(int force_fd, int state_fd, SimParams params) {
    setbuf(stdout, NULL);
    fprintf(stderr,
            "[D] Dynamics process started. M=%.3f, K=%.3f, dt=%.3f\n",
            params.mass, params.visc, params.dt);

    double M = params.mass;
    double K = params.visc;
    double T = params.dt;

    ForceStateMsg f;
    f.Fx = 0.0;
    f.Fy = 0.0;
    f.reset = 0;

    DroneStateMsg s = (DroneStateMsg){0.0, 0.0, 0.0, 0.0};

    int flags = fcntl(force_fd, F_GETFL, 0);
    if (flags == -1) flags = 0;
    if (fcntl(force_fd, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("[D] fcntl O_NONBLOCK");
    }

    while (1) {
        // Reads any new force command from B (non-blocking).
        ForceStateMsg new_f;
        int n = read(force_fd, &new_f, sizeof(new_f));

        if (n == (int)sizeof(new_f)) {
            if (new_f.reset != 0) {
                s.x  = 0.0;
                s.y  = 0.0;
                s.vx = 0.0;
                s.vy = 0.0;
            }
            f = new_f;
            f.reset = 0;
        } else if (n == 0) {
            fprintf(stderr, "[D] EOF on force pipe, exiting.\n");
            break;
        } else if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("[D] read");
                break;
            }
        } else {
            fprintf(stderr, "[D] Partial read (%d bytes) on force pipe.\n", n);
        }

        // Computes wall repulsive force from current state
        double Pwx = 0.0, Pwy = 0.0;
        compute_repulsive_P(&s, 
                    &params, 
                    0,
                    0,
                    true,   // calculate wall repulsion here
                    false,   // obstactles treated in server side
                    &Pwx, 
                    &Pwy);
        // Calculates total force = user force from B + wall repulsive force
        double Fx_total = f.Fx + Pwx;
        double Fy_total = f.Fy + Pwy;

        // Integrates dynamics with total force
        double ax = (Fx_total - K * s.vx) / M;
        double ay = (Fy_total - K * s.vy) / M;

        s.vx += ax * T;
        s.vy += ay * T;

        s.x  += s.vx * T;
        s.y  += s.vy * T;

        // Sends state back to B
        if (write(state_fd, &s, sizeof(s)) == -1) {
            perror("[D] write state");
            break;
        }

        // Sleeps until next time step
        struct timespec ts;
        ts.tv_sec  = 0;
        ts.tv_nsec = (long)(T * 1e9);
        nanosleep(&ts, NULL);
    }

    close(force_fd);
    close(state_fd);
    exit(EXIT_SUCCESS);
}
