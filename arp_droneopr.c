#include <stdio.h>      // printf, fprintf, perror, fgets, etc.
#include <stdlib.h>     // exit, strtod
#include <unistd.h>     // fork, pipe, read, write, close
#include <sys/select.h> // select(), fd_set
#include <sys/types.h>  // pid_t
#include <sys/wait.h>   // wait()
#include <ncurses.h>    // ncurses UI
#include <string.h>     // strlen, strtok, strcmp
#include <time.h>       // nanosleep()
#include <fcntl.h>      // fcntl(), O_NONBLOCK
#include <stdbool.h>    // bool, true, false


 /*  MESSAGE STRUCTURES  fo use over pipes)*/

// Key input from I to B
typedef struct {
    char key;   // the character typed (e.g., 'w', 'e', 'q', etc.)
} KeyMsg;

// Force command from B to D
typedef struct {
    double Fx;
    double Fy;
} ForceStateMsg;

// Drone state from D to B
typedef struct {
    double x, y;    // position
    double vx, vy;  // velocity
} DroneStateMsg;


/*KEY CLUSTER input- Given a key, fill dFx and dFy with unit direction components.
 *  B will multiply these by some "force_step" and accumulate.*/
void direction_from_key(char key, double *dFx, double *dFy) {
    *dFx = 0.0;
    *dFy = 0.0;

    switch (key) {
        case 'w': *dFx = -1; *dFy = +1; break; // up-left
        case 'e': *dFx =  0; *dFy = +1; break; // straight up
        case 'r': *dFx = +1; *dFy = +1; break; // up-right

        case 's': *dFx = -1; *dFy =  0; break; // left
        case 'd': *dFx =  0; *dFy =  0; break; // brake 
        case 'f': *dFx = +1; *dFy =  0; break; // right

        case 'x': *dFx = -1; *dFy = -1; break; // down-left
        case 'c': *dFx =  0; *dFy = -1; break; // straight down
        case 'v': *dFx = +1; *dFy = -1; break; // down-right

        default:
            // For any other key, the force component is not changed
            break;
    }
}

// KEYBOARD PROCESS (I)

void run_keyboard_process(int write_fd) {
    // Turn off stdout buffering so debug prints appear immediately.
    setbuf(stdout, NULL);

    fprintf(stderr,
        "[I] Keyboard process started.\n"
        "[I] Use the 3x3 cluster: w e r / s d f / x c v.\n"
        "[I] Press 'q' to quit.\n");

    while (1) {
        // getchar() blocks until user presses a key or EOF happens.
        int c = getchar();

        if (c == EOF) {
            fprintf(stderr, "[I] EOF on stdin, exiting keyboard process.\n");
            break;
        }

        KeyMsg km;
        km.key = (char)c;

        // Send the key message to B through the pipe.
        if (write(write_fd, &km, sizeof(km)) == -1) {
            perror("[I] write to B failed");
            break;
        }

        // If user presses 'q', we also terminate this process.
        if (km.key == 'q') {
            fprintf(stderr, "[I] 'q' pressed, exiting keyboard process.\n");
            break;
        }
    }

    // Always close the pipe descriptor before exiting.
    close(write_fd);
    exit(EXIT_SUCCESS);
}

/* DYNAMICS PROCESS (D)*/
void run_dynamics_process(int force_fd, int state_fd) {
    setbuf(stdout, NULL);
    fprintf(stderr, "[D] Dynamics process started.\n");

    // --- Simulation parameters ---
    double M = 1.0;    // "mass" of the drone
    double K = 1.0;    // viscous friction coefficient
    double T = 0.05;   // simulation time step in seconds (20 Hz)

    // Start with zero force and zero state.
    ForceStateMsg f = {0.0, 0.0};
    DroneStateMsg s = {0.0, 0.0, 0.0, 0.0};

    // Make the force pipe non-blocking.
    //   - read() will NOT block if there is no data.
    //   - Instead, it will return -1 with errno = EAGAIN / EWOULDBLOCK.
    int flags = fcntl(force_fd, F_GETFL, 0);
    if (flags == -1) flags = 0;
    if (fcntl(force_fd, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("[D] fcntl O_NONBLOCK");
        // Not fatal, but means read() may block sometimes.
    }

    while (1) {
        // --- 1) Try to read a new force command (non-blocking) ---
        ForceStateMsg new_f;
        int n = read(force_fd, &new_f, sizeof(new_f));

        if (n == (int)sizeof(new_f)) {
            // We got a full ForceStateMsg, update f.
            f = new_f;
        } else if (n == 0) {
            // EOF on force pipe: B has closed the write end.
            fprintf(stderr, "[D] EOF on force pipe, exiting.\n");
            break;
        } else if (n < 0) {
            // If there's simply no data, read fails with EAGAIN/EWOULDBLOCK.
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                // Any other error is unexpected → terminate.
                perror("[D] read");
                break;
            }
            // If EAGAIN/EWOULDBLOCK, we just keep using old f.
        } else {
            // Partial read (should not really happen here) → ignore.
            fprintf(stderr, "[D] Partial read (%d bytes) on force pipe.\n", n);
        }

        // --- 2) Integrate dynamics one step ---
        // dv/dt = (F - K v) / M
        double ax = (f.Fx - K * s.vx) / M;
        double ay = (f.Fy - K * s.vy) / M;

        // Update velocities
        s.vx += ax * T;
        s.vy += ay * T;

        // Update positions
        s.x  += s.vx * T;
        s.y  += s.vy * T;

        // --- 3) Send updated state back to B ---
        if (write(state_fd, &s, sizeof(s)) == -1) {
            perror("[D] write state");
            break;
        }

        // --- 4) Wait for the next time step ---
        struct timespec ts;
        ts.tv_sec  = 0;
        ts.tv_nsec = (long)(T * 1e9); // T seconds in nanoseconds
        nanosleep(&ts, NULL);
    }

    close(force_fd);
    close(state_fd);
    exit(EXIT_SUCCESS);
}
