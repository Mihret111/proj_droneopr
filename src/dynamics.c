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

/**
 * @brief Main loop for the Dynamics (D) process.
 * 
 * @details
 * Performs the physics simulation of the drone.
 * - **Architecture**: Receives force commands (user + obstacles) from Server (B) and sends back updated state (pos, vel).
 * - **Timing**: Runs at a fixed time step defined by params.dt (e.g., 0.01s).
 * - **Integration**: Uses simple Euler integration for velocity and position updates.
 * 
 * @param force_fd File descriptor for reading ForceStateMsg from Server (B).
 * @param state_fd File descriptor for writing DroneStateMsg to Server (B).
 * @param params   Simulation parameters (Mass, Viscosity, Time step).
 */
void run_dynamics_process(int force_fd, int state_fd, SimParams params) {
    FILE *log = open_process_log("dynamics", "D");
    if (!log) {
        // If log fails, still run; or exit. I recommend exit for assignment clarity:
        fprintf(stderr, "[D] cannot open dynamics log\n");
        // exit(EXIT_FAILURE);
    }

    setbuf(stdout, NULL);
    fprintf(log,
            "[D] Dynamics process started | PID = %d\n, M=%.3f, K=%.3f, dt=%.3f\n",
            getpid(), params.mass, params.visc, params.dt);

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
        fprintf(log, "[D] fcntl O_NONBLOCK failed\n");
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
            fprintf(log, "[D] EOF on force pipe, exiting.\n");
            break;
        } else if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                fprintf(log, "[D] read error on force pipe, exiting.\n");
                perror("[D] read");
                break;
            }
        } else {
            fprintf(log, "[D] Partial read (%d bytes) on force pipe.\n", n);
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

        // --------------------------------------------------------------
        // Physics Model: Newton's Second Law with Viscous Damping
        // F_net = F_user + F_repulsion - K * v
        // a = F_net / M
        // --------------------------------------------------------------
        double ax = (Fx_total - K * s.vx) / M;
        double ay = (Fy_total - K * s.vy) / M;

        // --------------------------------------------------------------
        // Numerical Integration: Standard Euler Method
        // v(t+dt) = v(t) + a * dt
        // x(t+dt) = x(t) + v(t+dt) * dt
        // --------------------------------------------------------------
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
