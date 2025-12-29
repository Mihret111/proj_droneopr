// server.c
// Defines server / blackboard process (B)
//   - Owns global "blackboard" state: force and drone state
//   - Listens to keys from I and states from D (via pipes)
//   - Sends updated forces to D
//   - Monitors obstacles and targets
//   - Draws ncurses User Interface comprising of the drone world and an inspection window
//   - Reacts to the commands pause 'p', reset 'O', brake 'd', quit 'q'
// ======================================================================

#define _POSIX_C_SOURCE 200809L
#include <signal.h>
#include <string.h>
#include <sys/select.h>   // for fd_set, FD_ZERO, FD_SET, select()
#include <sys/types.h>

#include "headers/server.h"
#include "headers/messages.h"
#include "headers/util.h"
#include "headers/obstacles.h"
#include "headers/targets.h"
#include <time.h>   // clock_gettime


#include <ncurses.h>
#include <stdio.h>
#include <stdlib.h>     // exit, strtod
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>
#include <math.h>  // for sqrt, to be used in key mapping instead of hard code, REP

//#define NUM_OBSTACLES 8
// Defines global / static array of 8 obstacles
//static Obstacle g_obstacles[NUM_OBSTACLES];

// Defines scoring globals
static int g_score             = 0;
static int g_targets_collected = 0;
static int g_last_hit_step     = -1;
static int g_step_counter      = 0;

// blinking warning banner globals
static int  wd_warning_active = 0;   // warning state ON/OFF
static int  wd_blink_phase   = 0;   // 0 or 1 (visible / invisible)
static int  wd_blink_counter = 0;   // counts simulation steps

static int wd_blink_ticks = 0;

// ---- Heartbeat timing ----
static struct timespec g_last_hb_ts;
static int g_have_hb = 0; // becomes 1 after first heartbeat timestamp is recorded

static double monotonic_now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

static void set_last_hb_now(void) {
    clock_gettime(CLOCK_MONOTONIC, &g_last_hb_ts);
    g_have_hb = 1;
}

static double hb_age_sec(void) {
    if (!g_have_hb) return 0.0;
    double now = monotonic_now_sec();
    double last = (double)g_last_hb_ts.tv_sec + 1e-9 * (double)g_last_hb_ts.tv_nsec;
    return now - last;
}


// ---------------- Watchdog signal flags (set by signal handlers) ----------------
static volatile sig_atomic_t g_wd_warning_flag = 0; // set by SIGUSR2 handler
static volatile sig_atomic_t g_wd_stop    = 0;  // set when SIGTERM arrives

static void on_watchdog_warning(int signo) {
    (void)signo;
    g_wd_warning_flag = 1;
}

static void on_watchdog_stop(int signo) {
    (void)signo;
    g_wd_stop = 1;
}

// ---------------- Watchdog banner UI state ----------------
// Show a warning banner for a limited amount of time after SIGUSR2
// We store it as "how many simulation steps remaining" to show the banner.
static char watchdog_banner_msg[] = "WATCHDOG WARNING, system may be unstable"; 

/**
 * @brief Main function for the Server (B) process.
 *
 * @details
 * Acts as the "Blackboard" or central hub of the architecture.
 * - **Responsibility**: Maintains the authoritative state of the world (drone, obstacles, targets).
 * - **IPC Hub**: Multiplexes inputs from Keyboard (I), Dynamics (D), Obstacles (O), and Targets (T).
 * - **Visualization**: Draws the ncurses UI.
 * - **Synchronization**: Sends the official force commands to Dynamics to step the physics.
 * 
 * @param fd_kb      Pipe FD for reading KeyMsg from Keyboard (I).
 * @param fd_to_d    Pipe FD for writing ForceStateMsg to Dynamics (D).
 * @param fd_from_d  Pipe FD for reading DroneStateMsg from Dynamics (D).
 * @param fd_obs     Pipe FD for reading obstacles from Generator (O).
 * @param fd_tgt     Pipe FD for reading targets from Generator (T).
 * @param pid_W      PID of the Watchdog process (for sending heartbeat signals).
 * @param params     Simulation parameters.
 */
void run_server_process(int fd_kb, int fd_to_d, int fd_from_d, int fd_obs, int fd_tgt, pid_t pid_W, SimParams params) 
{
    // --- Opens logfile ---
    FILE *logfile = open_process_log("server", "B");
    if (!logfile) {
        endwin();
        die("[B] cannot open logs/server.log");
    }
    // Initialize heartbeat tracking
    set_last_hb_now(); // assume "alive" at start

    // --- Initialize ncurses ---
    initscr();      // Assignment-1 (previously was called inside loop which caused seldom window flickering issues)
    cbreak();
    noecho();
    curs_set(0);  // hide cursor

    // Assignment-1 (previously was defined inside loop casing uneccessary repeated calls)
    // ---- ncurses color init (DO THIS ONCE) ----
    if (has_colors()) {     
        start_color();

        // Only attempt init_color if terminal supports changing colors.
        if (can_change_color()) {
            init_color(COLOR_YELLOW, 1000, 500, 0); // orange-ish
            init_color(COLOR_GREEN,  0, 1000, 0);   // green
        }

        init_pair(1, COLOR_YELLOW, COLOR_BLACK); // obstacles
        init_pair(2, COLOR_GREEN,  COLOR_BLACK); // targets
        init_pair(3, COLOR_RED,    COLOR_BLACK); // watchdog warning
    } else {
        // If cmd doesnot permit colors, then continue without colors.
    }

    // ---------------- Install signal handlers for Watchdog ----------------
    struct sigaction sa_warn;
    memset(&sa_warn, 0, sizeof(sa_warn));
    sa_warn.sa_handler = on_watchdog_warning;
    sigemptyset(&sa_warn.sa_mask);
    sa_warn.sa_flags = SA_RESTART;
    if (sigaction(SIGUSR2, &sa_warn, NULL) == -1) {
        fprintf(logfile, "[B] sigaction(SIGUSR2) failed: %s\n", strerror(errno));
        fflush(logfile);
    }

    struct sigaction sa_stop;
    memset(&sa_stop, 0, sizeof(sa_stop));
    sa_stop.sa_handler = on_watchdog_stop;
    sigemptyset(&sa_stop.sa_mask);
    sa_stop.sa_flags = SA_RESTART;
    if (sigaction(SIGTERM, &sa_stop, NULL) == -1) {
        fprintf(logfile, "[B] sigaction(SIGTERM) failed: %s\n", strerror(errno));
        fflush(logfile);
    }


    // --- Defines Blackboard state (model of the world)
    ForceStateMsg cur_force;
    cur_force.Fx = 0.0;
    cur_force.Fy = 0.0;
    cur_force.reset = 0;

    DroneStateMsg cur_state = (DroneStateMsg){0.0, 0.0, 0.0, 0.0};
    char last_key = '?';
    bool paused = false;

    // Sends to helper rather than directly write to D
    // Initial state is zero, so cur_state is still {0,0,0,0}.
    // Sends initial total force (which is just user=0 + obstacles repulsion).
    send_total_force_to_d(&cur_force,
                          &cur_state,
                          &params,
                          g_obstacles,
                          NUM_OBSTACLES,
                          fd_to_d,
                          logfile,
                          "init");


    int max_y, max_x;

    // Needed for handling time_since_last_hit
    double time_since_last_hit = 0; // for tracking time since last hit

    // --- Main event loop ---
    while (1) {

        // ---------------- Watchdog notifications ----------------
        // Watchdog warning -> start banner
        // Convert the SIGUSR2 flag into a visible banner for 2 seconds.
        // We do this here (NOT in signal handler) because ncurses is not signal-safe.
        if (g_wd_warning_flag) {
            g_wd_warning_flag = 0;

            // Start blinking warning until SIGTERM arrives.
            wd_warning_active = 1;
            wd_blink_phase    = 1;   // start "visible"
            wd_blink_counter  = 0;
            wd_blink_ticks    = 0;

            fprintf(logfile, "[B] WATCHDOG WARNING: blinking ON\n");
            fflush(logfile);
        }



        if (g_wd_stop) {
            fprintf(logfile, "[B] WATCHDOG STOP: received SIGTERM, exiting.\n");
            fflush(logfile);
            break; // exit from server loop
        }

        // Queries current terminal size (for resizing).
        getmaxyx(stdscr, max_y, max_x);

        // Plans layout:
        //   - 2 top lines of info
        //   - horizontal separator
        //   - world area below
        //   - inspection panel on the right
        int content_top    = 1;                 // first row inside border
        int top_lines      = 2;                 // 2 text lines at top
        int top_info_y1    = content_top;
        int top_info_y2    = content_top + 1;
        int sep_y          = content_top + top_lines; // horizontal separator row
        int content_bottom = max_y - 2;         // last row inside bottom border

        if (sep_y >= content_bottom) {
            sep_y = content_top; // in tiny terminals
        }

        // Defines right inspection panel width
        int insp_width = 35;               // was 35
        if (max_x < insp_width + 10) {
            insp_width = max_x / 4;
            if (insp_width < 10) insp_width = 10;
        }
        int insp_start_x = max_x - insp_width;
        if (insp_start_x < 1) insp_start_x = 1;

        // Defines world area below separator.
        int world_top    = sep_y + 1;
        if (world_top > content_bottom) world_top = content_top + 1;
        int world_bottom = content_bottom;
        int world_height = world_bottom - world_top + 1;
        if (world_height < 1) world_height = 1;

        // Defines left world width.
        int main_width = insp_start_x - 2;
        if (main_width < 10) main_width = 10;

        // ---------------- Uses select() to wait for events ----------------        // Uses select() to wait for data from keyboard, dynamics, obstacles, and targets.
        // Also handles EINTR (signal generated on resize to permit window resize without exiting the program).
        fd_set rfds;
        int maxfd = fd_kb;
        
        if (fd_from_d > maxfd) maxfd = fd_from_d;
        if (fd_obs    > maxfd) maxfd = fd_obs;
        if (fd_tgt    > maxfd) maxfd = fd_tgt;
        maxfd += 1;

        int sel;
        while (1) {
            FD_ZERO(&rfds);
            FD_SET(fd_kb,     &rfds);
            FD_SET(fd_from_d, &rfds);
            //
            FD_SET(fd_obs,    &rfds);
            FD_SET(fd_tgt,    &rfds);

            // sel = select(maxfd, &rfds, NULL, NULL, NULL);
            struct timeval tv;
            tv.tv_sec  = 0;
            tv.tv_usec = 100000; // 100 ms

            sel = select(maxfd, &rfds, NULL, NULL, &tv);

            if (sel == 0) {
                if (wd_warning_active && !paused) {
                    wd_blink_ticks++;

                    // blink every 5 ticks -> 5 * 100ms = 500ms
                    if (wd_blink_ticks >= 5) {
                        wd_blink_ticks = 0;
                        wd_blink_phase = !wd_blink_phase;
                    }
                }



                }

            if (sel == -1) {
                if (errno == EINTR) {
                    // Retries if interrupted by signal (like resize)
                    continue;
                } else {
                    fclose(logfile);
                    endwin();
                    die("[B] select failed");
                }
            }
            break; // sel >= 0, we have an event
        }

        // ------------------------------------------------------------------
        // Handles keyboard input from I (if available).
        // ------------------------------------------------------------------
        if (FD_ISSET(fd_kb, &rfds)) {
            KeyMsg km;
            int n = read(fd_kb, &km, sizeof(km));
            if (n <= 0) {
                mvprintw(0, 1, "[B] Keyboard process ended (EOF).");
                refresh();
                break;
            }

            last_key = km.key;

            // Handles Quit request
            if (km.key == 'q') {
                fprintf(logfile, "QUIT requested by 'q'\n");
                fflush(logfile);
                break;
            }
            // ------------------------------------------------------------------
            // Handles Pause toggle
            // ------------------------------------------------------------------
            if (km.key == 'p') {
                paused = !paused;

                if (paused) {
                    // Zeroes the force when entering pause.
                    cur_force.Fx = 0.0;
                    cur_force.Fy = 0.0;
                    cur_force.reset = 0;
                    send_total_force_to_d(&cur_force,
                                        &cur_state,
                                        &params,
                                        g_obstacles,
                                        NUM_OBSTACLES,
                                        fd_to_d,
                                        logfile,
                                        "key");
                    fprintf(logfile, "PAUSE: ON\n");
                } else {
                    fprintf(logfile, "PAUSE: OFF\n");
                }
                fflush(logfile);
            }
            // ------------------------------------------------------------------
            // Handles Reset (uppercase O)
            // ------------------------------------------------------------------
            else if (km.key == 'O') {
                // Resets server-side state
                cur_state.x  = 0.0;
                cur_state.y  = 0.0;
                cur_state.vx = 0.0;
                cur_state.vy = 0.0;

                // Resets forces
                cur_force.Fx = 0.0;
                cur_force.Fy = 0.0;
                cur_force.reset = 1; // Signals D to reset its state

                send_total_force_to_d(&cur_force,
                      &cur_state,
                      &params,
                      g_obstacles,
                      NUM_OBSTACLES,
                      fd_to_d,
                      logfile,
                      "key");

                cur_force.reset = 0; // Clears locally
                paused = false;      // Unpauses

                fprintf(logfile, "RESET requested (O)\n");
                fflush(logfile);
            }
            // ------------------------------------------------------------------
            // Handles Directional keys and the break 'd'
            // ------------------------------------------------------------------
            else {
                double dFx, dFy;
                direction_from_key(km.key, &dFx, &dFy);

                if (!paused) {
                    if (km.key == 'd') {
                        // Brake: Zeroes forces
                        cur_force.Fx = 0.0;
                        cur_force.Fy = 0.0;
                    } else {
                        // Accumulates new force
                        cur_force.Fx += dFx * params.force_step;
                        cur_force.Fy += dFy * params.force_step;
                    }

                    cur_force.reset = 0;

                    send_total_force_to_d(&cur_force,
                        &cur_state,
                        &params,
                        g_obstacles,
                        NUM_OBSTACLES,
                        fd_to_d,
                        logfile,
                        "key");

                    fprintf(logfile,
                            "KEY: %c  dFx=%.1f dFy=%.1f -> Fx=%.2f Fy=%.2f\n",
                            km.key, dFx, dFy, cur_force.Fx, cur_force.Fy);
                    fflush(logfile);
                } else {
                    // Paused: Ignores directional changes (but still log)
                    fprintf(logfile,
                            "KEY: %c ignored (PAUSED)\n", km.key);
                    fflush(logfile);
                }
            }
        }

        // ------------------------------------------------------------------
        // 4) Handles state updates from D (if available).
        // ------------------------------------------------------------------
        if (FD_ISSET(fd_from_d, &rfds)) {
            DroneStateMsg s;
            int n = read(fd_from_d, &s, sizeof(s));
            if (n == (int)sizeof(s)) {
                // We received a valid "tick" from dynamics => system is alive
                set_last_hb_now();

                // Send heartbeat to watchdog (as before)
                if (pid_W > 0) kill(pid_W, SIGUSR1);

                // POLISH: if we were blinking due to warning, clear it once activity resumes
                if (wd_warning_active) {
                    wd_warning_active = 0;
                    wd_blink_phase = 0;
                    wd_blink_counter = 0;

                    fprintf(logfile, "[B] Heartbeat resumed -> cleared watchdog warning UI\n");
                    fflush(logfile);
                }
            }
            else if (n <= 0) {
                mvprintw(1, 1, "[B] Dynamics process ended (EOF).");
                refresh();
                break;
            } else {
                // partial read (should not happen with pipes + small struct, but handle anyway)
                fprintf(logfile, "[B] Partial read from D: %d bytes\n", n);
                fflush(logfile);
                continue;
            }



            // Updates current state
            cur_state = s;

            // Increments global step counter (one more state update)
            if (!paused) {
                g_step_counter++;
            }   

            // Logs state
            fprintf(logfile,
                    "STATE: x=%.2f y=%.2f vx=%.2f vy=%.2f\n",
                    s.x, s.y, s.vx, s.vy);
            fflush(logfile);
            // Checks for target hits (only when not paused)
            if (!paused) {
                int hits = check_target_hits(&cur_state,
                                            g_targets,
                                            NUM_TARGETS,
                                            &params,
                                            &g_score,
                                            &g_targets_collected,
                                            &g_last_hit_step,
                                            g_step_counter);
                if (hits > 0) {
                    fprintf(logfile,
                            "[B] Collected %d target(s). SCORE=%d\n",
                            hits, g_score);
                    fflush(logfile);
                }
            }
            // Decrements obstacles and targets lifetimes 
            // Considers each time input is received from D, 1 sim time had elapsed
            // Only age obstacles & targets when simulation is running
            if (!paused){    
                for (int i = 0; i < NUM_OBSTACLES; ++i) {
                    if (g_obstacles[i].active && g_obstacles[i].life_steps > 0) {
                        g_obstacles[i].life_steps--;   // Decreases 1 step from its lifetime
                        if (g_obstacles[i].life_steps == 0) {
                            g_obstacles[i].active = 0;
                        }
                    }
                }
                for (int i = 0; i < NUM_TARGETS; ++i) {
                    if (g_targets[i].active && g_targets[i].life_steps > 0) {
                        g_targets[i].life_steps--;
                        if (g_targets[i].life_steps == 0) {
                            g_targets[i].active = 0;
                        }
                    }
                }
            }
            // Update blinking phase only while running (not paused)
            if (wd_warning_active && !paused) {
                wd_blink_counter++;

                // Blink period in seconds:
                const double BLINK_PERIOD_SEC = 0.5; // 0.5s ON/OFF toggle

                int blink_steps = (int)(BLINK_PERIOD_SEC / params.dt);
                if (blink_steps < 1) blink_steps = 1;

                if (wd_blink_counter >= blink_steps) {
                    wd_blink_counter = 0;
                    wd_blink_phase = !wd_blink_phase; // toggle
                }
            }


            // Then, sends updated total force (evenif user doesn't send cmd) (user + obstacles)
            send_total_force_to_d(&cur_force,
                                  &cur_state,
                                  &params,
                                  g_obstacles,
                                  NUM_OBSTACLES,
                                  fd_to_d,
                                  logfile,
                                  "state");
                                  
        }

        // ------------------------------------------------------------------
        // Handles obstacle set messages from O
        // ------------------------------------------------------------------
        if (FD_ISSET(fd_obs, &rfds)) {
            ObstacleSetMsg msg;
            int n = read(fd_obs, &msg, sizeof(msg));
            if (n <= 0) {
                // if nth read, O process ended; may log and continue
                mvprintw(0, 1, "[B] Obstacle generator ended.");
                // Optionally: fd_obs = -1 and stop using it
            } else {
                if (paused){
                    // Reads but ignores new obstacles while paused
                    fprintf(logfile,
                            "[B] Received obstacle set but PAUSED -> ignored.\n");
                    fflush(logfile);
                } else {
                    int requested = msg.count;
                    if (requested > NUM_OBSTACLES) requested = NUM_OBSTACLES;

                    // Uses a clearance similar to what we used for targets
                    double tgt_clearance = params.world_half * 0.15;

                    int accepted = 0;

                    for (int i = 0; i < requested; ++i) {
                        double x = msg.obs[i].x;
                        double y = msg.obs[i].y;

                        // Rejects if too close to any active target
                        if (too_close_to_any_pointlike(x, y,
                               (PointLike*)g_obstacles,
                               NUM_OBSTACLES,
                               tgt_clearance)){
                            fprintf(logfile,
                                    "[B] Obstacle (%.2f, %.2f) rejected: too close to target.\n",
                                    x, y);
                            continue;
                        }

                        // Stores it if accepted index is within capacity
                        if (accepted < NUM_OBSTACLES) {
                            g_obstacles[accepted].x          = x;
                            g_obstacles[accepted].y          = y;
                            g_obstacles[accepted].life_steps = msg.obs[i].life_steps;
                            g_obstacles[accepted].active     = 1;
                            accepted++;
                        }
                    }

                    // Deactivates remaining slots
                    for (int i = accepted; i < NUM_OBSTACLES; ++i) {
                        g_obstacles[i].active     = 0;
                        g_obstacles[i].life_steps = 0;
                    }

                    fprintf(logfile,
                            "[B] Accepted %d obstacles (requested %d).\n",
                            accepted, requested);
                    fflush(logfile);
                }

            }
        }

        // ------------------------------------------------------------------
        // Handles target-set messages from T
        // ------------------------------------------------------------------

        if (FD_ISSET(fd_tgt, &rfds)) {
            TargetSetMsg msg;
            int n = read(fd_tgt, &msg, sizeof(msg));
            if (n <= 0) {
                mvprintw(1, 1, "[B] Target generator ended.");
            } else {
                if (paused) {
                    fprintf(logfile,
                            "[B] Received target set but PAUSED -> ignored.\n");
            fflush(logfile);
        } else {
            int requested = msg.count;
            if (requested > NUM_TARGETS) requested = NUM_TARGETS;

            // Tuning for filtering:
            double wall_margin     = params.world_half * 0.20; // keep away from walls
            double obs_clearance   = params.world_half * 0.15; // away from obstacles

            int accepted = 0;

            for (int i = 0; i < requested; ++i) {
                double x = msg.tgt[i].x;
                double y = msg.tgt[i].y;

                // Rejects if too close to walls
                if (target_too_close_to_wall(x, y, &params, wall_margin)) {
                    fprintf(logfile,
                            "[B] Target (%.2f,%.2f) rejected: too close to walls.\n",
                            x, y);
                    continue;
                }

                // Rejects if too close to obstacles
                if (too_close_to_any_pointlike(x, y,
                               (PointLike*)g_targets,
                               NUM_TARGETS,
                               obs_clearance)){
                    fprintf(logfile,
                            "[B] Target (%.2f,%.2f) rejected: too close to obstacles.\n",
                            x, y);
                    continue;
                }

                // Accepts target if it passed the above checks
                if (accepted < NUM_TARGETS) {
                    g_targets[accepted].x          = x;
                    g_targets[accepted].y          = y;
                    g_targets[accepted].life_steps = msg.tgt[i].life_steps;
                    g_targets[accepted].active     = 1;
                    accepted++;
                }
            }

            // Deactivates remaining slots
            for (int i = accepted; i < NUM_TARGETS; ++i) {
                g_targets[i].active     = 0;
                g_targets[i].life_steps = 0;
            }

            fprintf(logfile,
                    "[B] Accepted %d targets (requested %d).\n",
                    accepted, requested);
            fflush(logfile);
        }
    }

}
        // ------------------------------------------------------------------
        // Draws UI (drone world + inspection panel)
        // ------------------------------------------------------------------
        erase();
        box(stdscr, 0, 0);

        // Top info lines
        mvprintw(top_info_y1, 2,
                 "Controls: w e r / s d f / x c v | d=brake, p=pause, O=reset, q=quit");
        mvprintw(top_info_y2, 2,
                 "Paused: %s", paused ? "YES" : "NO");
        
        // Watchdog blinking warning: visible only when active AND blink phase is ON
        // --- Watchdog live timing info ---
        double age = hb_age_sec();  // seconds since last valid DroneStateMsg
        double warn_in = (double)params.wd_warn_sec - age;
        double kill_in = (double)params.wd_kill_sec - age;

        if (warn_in < 0) warn_in = 0;
        if (kill_in < 0) kill_in = 0;

        if (wd_warning_active && wd_blink_phase) {
            // If colors exist, use a red-ish pair. Otherwise use reverse + bold.
            if (has_colors()) {
                // Create once somewhere during init if you want:
                // init_pair(3, COLOR_RED, COLOR_BLACK);
                attron(COLOR_PAIR(3) | A_BOLD | A_REVERSE);
                mvprintw(top_info_y2, 18, " %s ", watchdog_banner_msg);
                mvprintw(top_info_y2, 60, "KILL IN: %.2fs", kill_in);
                attroff(COLOR_PAIR(3) | A_BOLD | A_REVERSE);
            } else {
                attron(A_BOLD | A_REVERSE);
                mvprintw(top_info_y2, 18, " %s ", watchdog_banner_msg);
                mvprintw(top_info_y2, 60, "KILL IN: %.2fs", kill_in);
                attroff(A_BOLD | A_REVERSE);
            }
        }



        

        // Horizontal separator row (under top info)
        if (sep_y >= 1 && sep_y <= max_y - 2) {
            for (int x = 1; x < max_x - 1; ++x) {
                mvaddch(sep_y, x, '-');
            }
        }

        // Vertical separator between world and inspection
        int sep_x = insp_start_x - 1;
        if (sep_x > 1 && sep_x < max_x - 1) {
            for (int y = world_top; y <= world_bottom; ++y) {
                mvaddch(y, sep_x, '|');
            }
        }

        // WORLD DRAWING (left)
        double world_half = params.world_half;
        double scale_x = main_width  / (2.0 * world_half);        // Maps world coordinates to the drone world on the display scree
        double scale_y = world_height / (2.0 * world_half);
        if (scale_x <= 0) scale_x = 1.0;
        if (scale_y <= 0) scale_y = 1.0;

        int sx = (int)(cur_state.x * scale_x) + main_width / 2 + 1;
        int sy = (int)(-cur_state.y * scale_y) + world_top + world_height / 2;

        if (sx < 1) sx = 1;
        if (sx > main_width) sx = main_width;
        if (sy < world_top) sy = world_top;
        if (sy > world_bottom) sy = world_bottom;

        mvaddch(sy, sx, '+'); // Draws drone

        // Draws active obstacles as 'o' in the drone world
        for (int k = 0; k < NUM_OBSTACLES; ++k) {
            if (!g_obstacles[k].active) continue;  // Skips inactive 

            int ox = (int)(g_obstacles[k].x * scale_x) + main_width / 2 + 1;
            int oy = (int)(-g_obstacles[k].y * scale_y) + world_top + world_height / 2;

            if (ox < 1) ox = 1;
            if (ox > main_width) ox = main_width;
            if (oy < world_top) oy = world_top;
            if (oy > world_bottom) oy = world_bottom;
            
            attron(COLOR_PAIR(1));
            mvaddch(oy, ox, 'o');  // TODO: Adds color to make them orange
            
            attroff(COLOR_PAIR(1));
        }

        for (int k = 0; k < NUM_TARGETS; ++k) {
            if (!g_targets[k].active) continue;

            int tx = (int)(g_targets[k].x * scale_x) + main_width / 2 + 1;
            int ty = (int)(-g_targets[k].y * scale_y) + world_top + world_height / 2;

            if (tx < 1) tx = 1;
            if (tx > main_width) tx = main_width;
            if (ty < world_top) ty = world_top;
            if (ty > world_bottom) ty = world_bottom;
            
            attron(COLOR_PAIR(2));
            mvaddch(ty, tx, 'T');  // Placeholder, later make them numbered
            attroff(COLOR_PAIR(2));
        }


        // INSPECTION panel on the right
        int info_y = world_top;
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
            
            mvprintw(info_y +12, info_x, "Score: %d", g_score);
            mvprintw(info_y +13, info_x, "Targets collected: %d", g_targets_collected);
            if (g_last_hit_step >= 0 ) {
                time_since_last_hit = (g_step_counter - g_last_hit_step) * params .dt;

                mvprintw(info_y +15, info_x, "Since last hit: %.2f sec", time_since_last_hit);
            }
            else {
                mvprintw(info_y +14, info_x, "Last hit: none");
            }

        }

        refresh();
    }

    // Final cleanup
    if (logfile) {
        fprintf(logfile, "[B] Exiting.\n");
        fclose(logfile);
    }
    // Ends ncurses
    endwin();
    // Closes pipes
    close(fd_kb);
    close(fd_to_d);
    close(fd_from_d);
    exit(EXIT_SUCCESS); 
}

