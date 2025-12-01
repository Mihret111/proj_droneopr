#include <stdio.h>      // printf, fprintf, perror, fgets, etc.
#include <stdlib.h>     // exit, strtod
#include <unistd.h>     // fork, pipe, read, write, close
#include <sys/select.h> // select(), fd_set
#include <sys/types.h>  // pid_t
#include <sys/wait.h>   // wait()
#include <ncurses.h>    // ncurses UI
#include <string.h>     // strlen, strtok, strcmp
#include <errno.h>      // errno, EINTR, EAGAIN, EWOULDBLOCK
#include <time.h>       // nanosleep()
#include <fcntl.h>      // fcntl(), O_NONBLOCK
#include <stdbool.h>    // bool, true, false


// PARAM STRUCTURE:  Collecting all tunable parameters in one struct.
// Each process (B and D) will use a copy of this, inherited after fork.
typedef struct {
    double mass;        // M
    double visc;        // K
    double dt;          // time step T
    double force_step;  // how much each key increments the force
    double world_half;  // world coordinate half-range (x,y ∈ [-world_half, +world_half])
} SimParams;

 /*  MESSAGE STRUCTURES  fo use over pipes)*/

// Key input from I to B
typedef struct {
    char key;   // the character typed (e.g., 'w', 'e', 'q', etc.)
} KeyMsg;

// Force command from B to D
typedef struct {
    double Fx;
    double Fy;
    int    reset;   // 0 = normal, 1 = reset
} ForceStateMsg;


// Drone state from D to B
typedef struct {
    double x, y;    // position
    double vx, vy;  // velocity
} DroneStateMsg;

/* UTILITY FUNCTIONS*/

// Simple integer max: used to compute max FD for select().
int imax(int a, int b) {
    return (a > b) ? a : b;
}

// Kill program with error message.
// use perror so we also see errno description.
void die(const char *msg) {
    perror(msg);
    exit(EXIT_FAILURE);
}

/*PARAMETER LOADING (params.txt)*/

// Initialize default parameter values.
void init_default_params(SimParams *p) {
    p->mass       = 1.0;
    p->visc       = 1.0;
    p->dt         = 0.05;
    p->force_step = 5.0;
    p->world_half = 50.0;
}

// Trim leading/trailing whitespace from a string (in place).
static void trim(char *s) {
    if (!s) return;
    // trim leading
    char *start = s;
    while (*start == ' ' || *start == '\t' || *start == '\n' || *start == '\r')
        start++;
    if (start != s) memmove(s, start, strlen(start) + 1);

    // trim trailing
    size_t len = strlen(s);
    while (len > 0 &&
           (s[len-1] == ' ' || s[len-1] == '\t' || s[len-1] == '\n' || s[len-1] == '\r')) {
        s[len-1] = '\0';
        len--;
    }
}

// Load parameters from a file with key=value lines, '#' comments.
// Unknown keys are ignored; missing file uses defaults (already set).
void load_params_from_file(const char *filename, SimParams *p) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        fprintf(stderr,
                "[PARAMS] Could not open '%s'. Using default parameters.\n",
                filename);
        return;
    }

    fprintf(stderr, "[PARAMS] Loading parameters from '%s'...\n", filename);

    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        // Remove leading/trailing whitespace
        trim(line);
        // Skip empty or comment lines
        if (line[0] == '\0' || line[0] == '#')
            continue;

        // Split "key=value"
        char *eq = strchr(line, '=');
        if (!eq) continue;  // malformed line, ignore

        *eq = '\0';
        char *key = line;
        char *val = eq + 1;

        trim(key);
        trim(val);

        double d = strtod(val, NULL);

        if (strcmp(key, "mass") == 0) {
            p->mass = d;
        } else if (strcmp(key, "visc") == 0) {
            p->visc = d;
        } else if (strcmp(key, "dt") == 0) {
            p->dt = d;
        } else if (strcmp(key, "force_step") == 0) {
            p->force_step = d;
        } else if (strcmp(key, "world_half") == 0) {
            p->world_half = d;
        } else {
            fprintf(stderr, "[PARAMS] Unknown key '%s', ignoring.\n", key);
        }
    }

    fclose(fp);

    fprintf(stderr,
            "[PARAMS] Loaded: mass=%.3f, visc=%.3f, dt=%.3f, force_step=%.3f, world_half=%.3f\n",
            p->mass, p->visc, p->dt, p->force_step, p->world_half);
}

/*********************************************************************************** */

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
        "[I] Use w e r / s d f / x c v to command force.\n"
        "[I] 'd' = brake, 'p' = pause, 'R' = reset, 'q' = quit.\n");

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

    DroneStateMsg s = {0.0, 0.0, 0.0, 0.0};

    // Non-blocking read on force_fd
    int flags = fcntl(force_fd, F_GETFL, 0);
    if (flags == -1) flags = 0;
    if (fcntl(force_fd, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("[D] fcntl O_NONBLOCK");
    }

    while (1) {
        // 1) Try to read new force command
        ForceStateMsg new_f;
        int n = read(force_fd, &new_f, sizeof(new_f));

        if (n == (int)sizeof(new_f)) {
            // Received a complete new force command
            if (new_f.reset != 0) {
                // Reset requested: zero state
                s.x  = 0.0;
                s.y  = 0.0;
                s.vx = 0.0;
                s.vy = 0.0;
                // force in message might be zero; we adopt it
            }
            f = new_f;
            f.reset = 0;  // locally clear reset flag
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

        // 2) Integrate dynamics one time step
        double ax = (f.Fx - K * s.vx) / M;
        double ay = (f.Fy - K * s.vy) / M;

        s.vx += ax * T;
        s.vy += ay * T;

        s.x  += s.vx * T;
        s.y  += s.vy * T;

        // 3) Send state to B
        if (write(state_fd, &s, sizeof(s)) == -1) {
            perror("[D] write state");
            break;
        }

        // 4) Sleep for T seconds
        struct timespec ts;
        ts.tv_sec  = 0;
        ts.tv_nsec = (long)(T * 1e9);
        nanosleep(&ts, NULL);
    }

    close(force_fd);
    close(state_fd);
    exit(EXIT_SUCCESS);
}

/* SERVER / BLACKBOARD PROCESS (B)*/
void run_server_process(int fd_kb, int fd_to_d, int fd_from_d, SimParams params) {
    // --- Init ncurses ---
    initscr();
    cbreak();
    noecho();
    curs_set(0);

    // --- Open logfile ---
    FILE *logfile = fopen("log.txt", "w");
    if (!logfile) {
        endwin();
        die("[B] cannot open log.txt");
    }

    // --- Blackboard state ---
    ForceStateMsg cur_force;
    cur_force.Fx = 0.0;
    cur_force.Fy = 0.0;
    cur_force.reset = 0;

    DroneStateMsg cur_state = {0.0, 0.0, 0.0, 0.0};
    char last_key = '?';
    bool paused = false;  // pause flag

    // Send initial zero-force to D
    if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
        fclose(logfile);
        endwin();
        die("[B] initial write to D failed");
    }

    // --- Main loop ---
    int max_y, max_x;

    while (1) {
        // 1) Get terminal size every loop (handle resize)
        getmaxyx(stdscr, max_y, max_x);

        // Compute inspection-right width and world-left width
        int insp_width = 35;
        if (max_x < insp_width + 10) {
            insp_width = max_x / 3;
            if (insp_width < 20) insp_width = 20;
        }

        int insp_start_x = max_x - insp_width;
        if (insp_start_x < 1) insp_start_x = 1;

        int main_width  = insp_start_x - 2; // left world inside borders
        if (main_width < 10) main_width = 10;

        int main_height = max_y - 2;        // vertical world inside borders
        if (main_height < 5) main_height = 5;

        // 2) Prepare fd_set and use select()
        fd_set rfds;
        int maxfd = imax(fd_kb, fd_from_d) + 1;

        int sel;
        while (1) {
            FD_ZERO(&rfds);
            FD_SET(fd_kb,     &rfds);
            FD_SET(fd_from_d, &rfds);

            sel = select(maxfd, &rfds, NULL, NULL, NULL);

            if (sel == -1) {
                if (errno == EINTR) {
                    // Interrupted by signal (e.g., resize SIGWINCH) → retry
                    continue;
                } else {
                    fclose(logfile);
                    endwin();
                    die("[B] select failed");
                }
            }
            // sel >= 0 -> at least one fd ready
            break;
        }

        // 3) Keyboard input from I (if any)
        if (FD_ISSET(fd_kb, &rfds)) {
            KeyMsg km;
            int n = read(fd_kb, &km, sizeof(km));
            if (n <= 0) {
                mvprintw(0, 1, "[B] Keyboard process ended (EOF).");
                refresh();
                break;
            }

            last_key = km.key;

            // Global quit
            if (km.key == 'q') {
                fprintf(logfile, "QUIT requested by 'q'\n");
                fflush(logfile);
                break;
            }

            // Handle PAUSE toggle
            if (km.key == 'p') {
                paused = !paused;
                if (paused) {
                    // On entering pause: set force to zero and send it.
                    cur_force.Fx = 0.0;
                    cur_force.Fy = 0.0;
                    cur_force.reset = 0;
                    if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
                        fclose(logfile);
                        endwin();
                        die("[B] write to D failed (pause)");
                    }
                    fprintf(logfile, "PAUSE: ON\n");
                } else {
                    fprintf(logfile, "PAUSE: OFF\n");
                }
                fflush(logfile);
            }
            // Handle RESET (uppercase R)
            else if (km.key == 'R') {
                // Reset position & velocity in B
                cur_state.x  = 0.0;
                cur_state.y  = 0.0;
                cur_state.vx = 0.0;
                cur_state.vy = 0.0;

                // Reset forces
                cur_force.Fx = 0.0;
                cur_force.Fy = 0.0;
                // Request D to also reset its internal state
                cur_force.reset = 1;

                if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
                    fclose(logfile);
                    endwin();
                    die("[B] write to D failed (reset)");
                }
                // Clear reset flag locally for future messages
                cur_force.reset = 0;

                // Also unpause if it was paused
                paused = false;

                fprintf(logfile, "RESET requested (R)\n");
                fflush(logfile);
            }
            // Direction / brake keys
            else {
                // Compute direction increments for possible directional keys
                double dFx, dFy;
                direction_from_key(km.key, &dFx, &dFy);

                // If we are paused, ignore directional changes
                if (!paused) {
                    if (km.key == 'd') {
                        // Brake key: zero forces
                        cur_force.Fx = 0.0;
                        cur_force.Fy = 0.0;
                    } else {
                        // Accumulate force using force_step from params
                        cur_force.Fx += dFx * params.force_step;
                        cur_force.Fy += dFy * params.force_step;
                    }

                    // For normal directional updates, reset=0
                    cur_force.reset = 0;

                    // Send updated force to D
                    if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
                        fclose(logfile);
                        endwin();
                        die("[B] write to D failed (force)");
                    }

                    // Log key & resulting force
                    fprintf(logfile,
                            "KEY: %c  dFx=%.1f dFy=%.1f → Fx=%.2f Fy=%.2f\n",
                            km.key, dFx, dFy, cur_force.Fx, cur_force.Fy);
                    fflush(logfile);
                } else {
                    // Paused: we still update last_key but ignore changes to force.
                    fprintf(logfile,
                            "KEY: %c ignored (PAUSED)\n", km.key);
                    fflush(logfile);
                }
            }
        }

        // 4) State updates from D (if any)
        if (FD_ISSET(fd_from_d, &rfds)) {
            DroneStateMsg s;
            int n = read(fd_from_d, &s, sizeof(s));
            if (n <= 0) {
                mvprintw(1, 1, "[B] Dynamics process ended (EOF).");
                refresh();
                break;
            }

            cur_state = s;

            fprintf(logfile,
                    "STATE: x=%.2f y=%.2f vx=%.2f vy=%.2f\n",
                    s.x, s.y, s.vx, s.vy);
            fflush(logfile);
        }

        // ---------------- 5) DRAW WORLD + INSPECTION ----------------
        erase();
        box(stdscr, 0, 0);

        // Separator between world and right-side inspection region
        int sep_x = insp_start_x - 1;
        if (sep_x > 1 && sep_x < max_x - 1) {
            for (int y = 1; y < max_y - 1; ++y) {
                mvaddch(y, sep_x, '|');
            }
        }

        // Map world coordinates to screen coordinates
        double world_half = params.world_half;
        double scale_x = main_width  / (2.0 * world_half);
        double scale_y = main_height / (2.0 * world_half);

        int sx = (int)(cur_state.x * scale_x) + main_width / 2 + 1;
        int sy = (int)(-cur_state.y * scale_y) + main_height / 2 + 1;

        if (sx < 1) sx = 1;
        if (sx > main_width) sx = main_width;
        if (sy < 1) sy = 1;
        if (sy > main_height) sy = main_height;

        mvaddch(sy, sx, '+');

        // Top info line
        mvprintw(1, 2,
                 "Controls: w e r / s d f / x c v | d=brake, p=pause, R=reset, q=quit");
        mvprintw(2, 2, "Paused: %s", paused ? "YES" : "NO");

        // Right inspection text region
        int info_y = 3;
        int info_x = insp_start_x + 1;
        if (info_x < max_x - 1) {
            mvprintw(info_y,     info_x, "INSPECTION");
            mvprintw(info_y + 2, info_x, "Last key: %c", last_key);
            mvprintw(info_y + 4, info_x, "Fx = %.2f", cur_force.Fx);
            mvprintw(info_y + 5, info_x, "Fy = %.2f", cur_force.Fy);
            mvprintw(info_y + 7, info_x, "x  = %.2f", cur_state.x);
            mvprintw(info_y + 8, info_x, "y  = %.2f", cur_state.y);
            mvprintw(info_y + 9, info_x, "vx = %.2f", cur_state.vx);
            mvprintw(info_y +10, info_x, "vy = %.2f", cur_state.vy);
        }

        refresh();
    }

    // Cleanup
    fclose(logfile);
    endwin();

    close(fd_kb);
    close(fd_to_d);
    close(fd_from_d);

    exit(EXIT_SUCCESS);
}

/*MAIN: Sets up pipes and processes*/
int main(void) {
    // 1) Load parameters in master BEFORE fork so children inherit them
    SimParams params;
    init_default_params(&params);
    load_params_from_file("params.txt", &params);

    // 2) Create pipes
    int pipe_I_to_B[2]; // I -> B
    int pipe_B_to_D[2]; // B -> D
    int pipe_D_to_B[2]; // D -> B

    if (pipe(pipe_I_to_B) == -1) die("pipe I->B");
    if (pipe(pipe_B_to_D) == -1) die("pipe B->D");
    if (pipe(pipe_D_to_B) == -1) die("pipe D->B");

    // 3) Fork Keyboard process (I)
    pid_t pid_I = fork();
    if (pid_I == -1) die("fork I");

    if (pid_I == 0) {
        // CHILD: I
        close(pipe_I_to_B[0]);   // I only writes to I->B[1]
        close(pipe_B_to_D[0]); close(pipe_B_to_D[1]);
        close(pipe_D_to_B[0]); close(pipe_D_to_B[1]);
        run_keyboard_process(pipe_I_to_B[1]);
    }

    // 4) Fork Dynamics process (D)
    pid_t pid_D = fork();
    if (pid_D == -1) die("fork D");

    if (pid_D == 0) {
        // CHILD: D
        close(pipe_I_to_B[0]); close(pipe_I_to_B[1]);
        close(pipe_B_to_D[1]);   // D reads from B->D[0]
        close(pipe_D_to_B[0]);   // D writes to D->B[1]
        run_dynamics_process(pipe_B_to_D[0], pipe_D_to_B[1], params);
    }

    // 5) PARENT: becomes Server B
    close(pipe_I_to_B[1]);  // B reads from I->B[0]
    close(pipe_B_to_D[0]);  // B writes to B->D[1]
    close(pipe_D_to_B[1]);  // B reads from D->B[0]

    run_server_process(pipe_I_to_B[0], pipe_B_to_D[1], pipe_D_to_B[0], params);

    // 6) Wait for children to avoid zombies
    wait(NULL);
    wait(NULL);

    return 0;
}
