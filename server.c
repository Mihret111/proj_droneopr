// server.c
// Defines server / blackboard process (B)
//   - Owns global "blackboard" state: force and drone state
//   - Listens to keys from I and states from D (via pipes)
//   - Sends updated forces to D
//   - Monitors obstacles and targets
//   - Draws ncurses User Interface comprising of the drone world and an inspection window
//   - Reacts to the commands pause 'p', reset 'O', brake 'd', quit 'q'
// ======================================================================

#include "headers/server.h"
#include "headers/messages.h"
#include "headers/util.h"
#include "headers/obstacles.h"
#include "headers/targets.h"

#include <ncurses.h>
#include <stdio.h>
#include <stdlib.h>     // exit, strtod
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>

#include <sys/select.h>   // for fd_set, FD_ZERO, FD_SET, select()
#include <math.h>  // for sqrt, to be used in key mapping instead of hard code, REP

//#define NUM_OBSTACLES 8
// Defines global / static array of 8 obstacles
//static Obstacle g_obstacles[NUM_OBSTACLES];

// Defines scoring globals
static int g_score             = 0;
static int g_targets_collected = 0;
static int g_last_hit_step     = -1;
static int g_step_counter      = 0;

// ----------------------------------------------------------------------
// Runs the server process:
//   - reads KeyStateMsg   from fd_kb (from I)
//   - Sends ForceStateMsg to fd_to_d (to D)
//   - Reads DroneStateMsg   from fd_from_d (from D)
//   - Reads ObstacleMsg   from fd_obs (from B)
//   - Reads TargetMsg   from fd_tgt (from B)
// ----------------------------------------------------------------------
void run_server_process(int fd_kb, int fd_to_d, int fd_from_d, int fd_obs, int fd_tgt, SimParams params) 
{
    // --- Initialize ncurses ---
    initscr();
    cbreak();
    noecho();
    curs_set(0);  // hide cursor

    // --- Opens logfile ---
    FILE *logfile = fopen("log.txt", "w");
    if (!logfile) {
        endwin();
        die("[B] cannot open log.txt");
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

        // Needed to plot the colored items
        initscr();
        start_color();

        // Enables color if terminal supports it
        if (has_colors() == FALSE) {
            endwin();
            printf("Your terminal does not support colors.\n");
        return;
        }

        // Defines custom colors for target and obstacles
        // RGB scaled 0–1000 in ncurses
        init_color(COLOR_YELLOW, 1000, 500, 0);   // For obs: Strong orange (R=1000, G=500, B=0)
        init_color(COLOR_GREEN, 0, 1000, 0);   // For targets: light green
        // Assigns color-pair IDs 1 and 2: ORANGE foreground, BLACK background
        init_pair(1, COLOR_YELLOW, COLOR_BLACK);
        init_pair(2, COLOR_GREEN, COLOR_BLACK);
       
        // Uses select() to wait for data from keyboard, dynamics, obstacles, and targets.
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

            sel = select(maxfd, &rfds, NULL, NULL, NULL);

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
            if (n <= 0) {
                mvprintw(1, 1, "[B] Dynamics process ended (EOF).");
                refresh();
                break;
            }

            cur_state = s;
            // Increments global step counter (one more state update)
            if (!paused) {
                g_step_counter++;
            }   

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
    fclose(logfile);
    endwin();
    close(fd_kb);
    close(fd_to_d);
    close(fd_from_d);
    exit(EXIT_SUCCESS);
}
