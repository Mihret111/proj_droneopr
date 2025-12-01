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

/* SERVER / BLACKBOARD PROCESS (B)*/
void run_server_process(int fd_kb, int fd_to_d, int fd_from_d) {
    // ---------------- ncurses initialization ----------------
    initscr();     // start ncurses mode
    cbreak();      // pass characters immediately, no line buffering
    noecho();      // do not echo typed chars automatically
    curs_set(0);   // hide the cursor (cosmetic)

    // ---------------- open logfile ----------------
    FILE *logfile = fopen("log.txt", "w");
    if (!logfile) {
        endwin();
        die("[B] cannot open log.txt");
    }

    // ---------------- server-side state ----------------
    ForceStateMsg cur_force = {0.0, 0.0};
    DroneStateMsg cur_state = {0.0, 0.0, 0.0, 0.0};
    double force_step = 5.0;    // how much one key "pushes" the force
    char last_key = '?';        // last key seen from I

    // Send initial zero force to D so it has something to start with.
    if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
        endwin();
        die("[B] initial write to D failed");
    }

    // ---------------- the main event loop ----------------
    while (1) {
        // 1) Get current terminal size.
        int max_y, max_x;
        getmaxyx(stdscr, max_y, max_x);

        // Decide inspection panel width:
        // We want some space for text but avoid negative or tiny widths.
        int insp_width = 35;
        if (max_x < insp_width + 10) {
            // Terminal is narrow: shrink inspection width.
            insp_width = max_x / 3;
            if (insp_width < 20) insp_width = 20; // minimum width for readability
        }

        // Column index where inspection area starts.
        int insp_start_x = max_x - insp_width;
        if (insp_start_x < 1) insp_start_x = 1;

        // World (drone map) width = area to the left of inspection.
        int main_width  = insp_start_x - 2; // -2 for outer border and separator
        if (main_width < 10) main_width = 10;

        // World height uses rows inside the border.
        int main_height = max_y - 2;
        if (main_height < 5) main_height = 5;

        // 2) Prepare fd_set for select().
        fd_set rfds;
        int maxfd = imax(fd_kb, fd_from_d) + 1;

        int sel;
        while (1) {
            FD_ZERO(&rfds);
            FD_SET(fd_kb, &rfds);
            FD_SET(fd_from_d, &rfds);

            // select() waits indefinitely (NULL timeout) until any FD has data.
            sel = select(maxfd, &rfds, NULL, NULL, NULL);

            if (sel == -1) {

                    endwin();
                    die("[B] select failed");
                }
            
            // sel >= 0 → we have some event to handle.
            break;
        }

        // 3) Handle keyboard messages from I (if available).
        if (FD_ISSET(fd_kb, &rfds)) {
            KeyMsg km;
            int n = read(fd_kb, &km, sizeof(km));

            if (n <= 0) {
                // Pipe closed (keyboard process ended).
                mvprintw(0, 1, "[B] Keyboard process ended (EOF).");
                refresh();
                break;
            }

            last_key = km.key;

            // Global quit if 'q' is pressed.
            if (km.key == 'q') {
                fprintf(logfile, "QUIT requested by 'q'\n");
                fflush(logfile);
                break;
            }

            // Convert key → direction increments.
            double dFx, dFy;
            direction_from_key(km.key, &dFx, &dFy);

            if (km.key == 'd') {
                // Brake key: zero all forces.
                cur_force.Fx = 0.0;
                cur_force.Fy = 0.0;
            } else {
                // Accumulate force command.
                cur_force.Fx += dFx * force_step;
                cur_force.Fy += dFy * force_step;
            }

            // Send updated force to D.
            if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
                endwin();
                die("[B] write to D failed");
            }

            // Log key event.
            fprintf(logfile,
                    "KEY: %c  dFx=%.1f dFy=%.1f → Fx=%.2f Fy=%.2f\n",
                    km.key, dFx, dFy, cur_force.Fx, cur_force.Fy);
            fflush(logfile);
        }

        // 4) Handle state messages from D (if available).
        if (FD_ISSET(fd_from_d, &rfds)) {
            DroneStateMsg s;
            int n = read(fd_from_d, &s, sizeof(s));
            if (n <= 0) {
                mvprintw(1, 1, "[B] Dynamics process ended (EOF).");
                refresh();
                break;
            }

            cur_state = s;

            // Log new state.
            fprintf(logfile,
                    "STATE: x=%.2f y=%.2f vx=%.2f vy=%.2f\n",
                    s.x, s.y, s.vx, s.vy);
            fflush(logfile);
        }

        // ---------------- 5) Draw entire screen (stdscr) ----------------
        erase();            // clear buffer
        box(stdscr, 0, 0);  // draw outer border

        // Draw vertical separator between world area and inspection area.
        int sep_x = insp_start_x - 1;
        if (sep_x > 1 && sep_x < max_x - 1) {
            for (int y = 1; y < max_y - 1; ++y) {
                mvaddch(y, sep_x, '|');
            }
        }

        // Map physical coordinates to screen coordinates.
        // Assume world roughly in [-50, 50] in both x and y.
        double world_half = 50.0;
        double scale_x = main_width  / (2.0 * world_half);
        double scale_y = main_height / (2.0 * world_half);

        int sx = (int)(cur_state.x * scale_x) + main_width / 2 + 1;
        int sy = (int)(-cur_state.y * scale_y) + main_height / 2 + 1;

        // Clamp drone visual position inside main area.
        if (sx < 1) sx = 1;
        if (sx > main_width) sx = main_width;
        if (sy < 1) sy = 1;
        if (sy > main_height) sy = main_height;

        // Draw drone as a '+' character.
        mvaddch(sy, sx, '+');


        // ---------------- 6) Draw inspection info (right side) ----------------
        int info_y = 2;
        int info_x = insp_start_x + 1;

        if (info_x < max_x - 1) {

        // Print control hint on top.
            mvprintw(info_y,     info_x,
                 "Controls: w e r / s d f / x c v");
            mvprintw(info_y + 1,     info_x + 8 ,
            "(d = brake, q = quit)");
            mvprintw(info_y + 2, info_x, "INSPECTION");
            mvprintw(info_y + 4, info_x, "Last key: %c", last_key);
            mvprintw(info_y + 6, info_x, "Fx = %.2f", cur_force.Fx);
            mvprintw(info_y + 7, info_x, "Fy = %.2f", cur_force.Fy);
            mvprintw(info_y + 9, info_x, "x  = %.2f", cur_state.x);
            mvprintw(info_y + 10, info_x, "y  = %.2f", cur_state.y);
            mvprintw(info_y + 11, info_x, "vx = %.2f", cur_state.vx);
            mvprintw(info_y +12, info_x, "vy = %.2f", cur_state.vy);
        }

        // Push all drawings to the actual terminal.
        refresh();
    }

    // ---------------- clean up ----------------
    fclose(logfile);
    endwin();  // restore terminal to normal mode

    close(fd_kb);
    close(fd_to_d);
    close(fd_from_d);

    exit(EXIT_SUCCESS);
}

/*MAIN: Sets up pipes and processes*/
int main(void) {
    // Each pipe is an array of 2 ints: [0]=read end, [1]=write end.
    int pipe_I_to_B[2]; // from I to B
    int pipe_B_to_D[2]; // from B to D
    int pipe_D_to_B[2]; // from D to B

    // Create all pipes.
    if (pipe(pipe_I_to_B) == -1) die("pipe I->B");
    if (pipe(pipe_B_to_D) == -1) die("pipe B->D");
    if (pipe(pipe_D_to_B) == -1) die("pipe D->B");

    // ---------------- Fork Keyboard Process (I) ----------------
    pid_t pid_I = fork();
    if (pid_I == -1) die("fork I");

    if (pid_I == 0) {
        // CHILD: I

        // I only writes to pipe_I_to_B[1].
        close(pipe_I_to_B[0]);   // close read end

        // I does not use these pipes at all.
        close(pipe_B_to_D[0]);
        close(pipe_B_to_D[1]);
        close(pipe_D_to_B[0]);
        close(pipe_D_to_B[1]);

        // Run keyboard logic. This function never returns.
        run_keyboard_process(pipe_I_to_B[1]);
    }

    // ---------------- Fork Dynamics Process (D) ----------------
    pid_t pid_D = fork();
    if (pid_D == -1) die("fork D");

    if (pid_D == 0) {
        // CHILD: D

        // D does not use the I->B pipe.
        close(pipe_I_to_B[0]);
        close(pipe_I_to_B[1]);

        // D reads from B->D[0] and writes to D->B[1].
        close(pipe_B_to_D[1]);   // close unused write end
        close(pipe_D_to_B[0]);   // close unused read end

        run_dynamics_process(pipe_B_to_D[0], pipe_D_to_B[1]);
    }

    // ---------------- PARENT: becomes Server B ----------------

    // B reads from I->B[0].
    close(pipe_I_to_B[1]);  // close unused write end

    // B writes to B->D[1].
    close(pipe_B_to_D[0]);  // close unused read end

    // B reads from D->B[0].
    close(pipe_D_to_B[1]);  // close unused write end

    // Now run the server logic (ncurses UI + IPC).
    run_server_process(pipe_I_to_B[0], pipe_B_to_D[1], pipe_D_to_B[0]);

    // After server exits, wait for children to avoid zombies.
    wait(NULL); // for one child
    wait(NULL); // for the other child

    return 0;
}
