// server.c
// server / blackboard process (B)
//   - Owns global "blackboard" state: force and drone state
//   - Listens to keys from I and states from D (via pipes)
//   - Sends updated forces to D
//   - Draws ncurses User Interface comprising of the drone world and an inspection window
//   - Reacts to the commands pause 'p', reset 'R', brake 'd', quit 'q'
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
// Global / static array of 8 obstacles
//static Obstacle g_obstacles[NUM_OBSTACLES];

#include <math.h>  // at top of server.c, if not already there


// another helper

// ----------------------------------------------------------------------
// Send total force to D:
//   F_total = user_force (cur_force) + obstacles repulsion (P_obs).
// Walls repulsion is still computed in D.
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// Send total force to D using a "virtual key" computed from
// the unified repulsive field (walls + obstacles).
//
// user_force  = persistent user force from keys (cur_force).
// cur_state   = latest drone state from D.
// params      = simulation params (mass, visc, force_step, world_half,
//               wall_clearance, wall_gain, etc.).
// obs         = array of obstacles in world coords.
// num_obs     = number of obstacles (8).
// fd_to_d     = pipe to dynamics process.
// logfile     = log file for debugging.
// reason      = small string saying why we are sending (e.g. "key", "state").
//
// Steps:
//   1. compute_repulsive_P(..., include_walls=true, include_obstacles=true)
//   2. project P onto 8 discrete directions (g_dir8).
//   3. choose best positive dot -> virtual key direction.
//   4. map magnitude to 1..N key steps.
//   5. build F_vk from best_key, n_steps, force_step.
//   6. out = user_force + F_vk, send to D.
//   7. IMPORTANT: user_force is NOT modified.
// ----------------------------------------------------------------------
static void send_total_force_to_d(const ForceStateMsg *user_force,
                                  const DroneStateMsg *cur_state,
                                  const SimParams     *params,
                                  const Obstacle      *obs,
                                  int                  num_obs,
                                  int                  fd_to_d,
                                  FILE                *logfile,
                                  const char          *reason)
{
    // 1) Unified continuous repulsive vector from walls + obstacles
    double Px = 0.0, Py = 0.0;
    compute_repulsive_P(cur_state,
                        params,
                        obs,
                        num_obs,
                        true,   // include_walls
                        true,   // include_obstacles
                        &Px, &Py);

    // If very small, just send user_force alone.
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

    // 2) Find discrete direction that best matches P
    int idx = best_dir8_for_vector(Px, Py);  // util.c
    if (idx < 0) {
        // No good direction -> fallback to user only
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
    double best_dot = dot2(Px, Py, g_dir8[idx].ux, g_dir8[idx].uy);

    if (best_dot <= 0.0) {
        // Same: fallback if projection is not positive
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

    // 3) Convert best_dot into key steps
    double step_force = params->force_step;
    double n_steps_f  = best_dot / (step_force + 1e-9);

    int n_steps = (int)(n_steps_f + 0.5);
/*     if (n_steps < 1) n_steps = 1;
    if (n_steps > 3) n_steps = 3;   */// you can tune this; 3–4 is usually safe

    // 4) Get direction from key
    double dFx, dFy;
    direction_from_key(best_key, &dFx, &dFy);

    double Fvk_x = n_steps * dFx * step_force;
    double Fvk_y = n_steps * dFy * step_force;

    // 5) Combine user + virtual-key repulsion
    ForceStateMsg out = *user_force;
    out.Fx += Fvk_x;
    out.Fy += Fvk_y;

    // 6) Send to D
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


//function to map wall repulsion to a virtual key

// ----------------------------------------------------------------------
// Given the current state and parameters, compute the total wall
// repulsive vector P (using util.c), project it onto the 8 directions,
// choose the direction with maximum positive projection, and interpret
// that as a "virtual key" that pushes the drone away from the walls.
//
// If a significant repulsion is found, this function:
//   - updates cur_force (vectorially, like a key)
//   - sends cur_force to D via fd_to_d
//   - logs the event to logfile.
//
// If no significant repulsion is needed, it does nothing.
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// WALL REPULSION via "virtual key" *without* accumulating into cur_force.
//
// cur_force   = persistent user force (from real keys).
// cur_state   = current drone state from D.
// params      = simulation + wall params.
// fd_to_d     = pipe to D.
// logfile     = for debug logging.
// paused      = if true, we don't apply automatic corrections.
//
// We compute P from the 4 walls, project it onto the 8 directions,
// choose the best direction, build a *temporary* repulsive contribution
// F_rep, and send (cur_force + F_rep) to D.
// cur_force itself is NOT modified, so repulsion does not explode over time.
// ----------------------------------------------------------------------
static void apply_wall_repulsion_virtual_key(ForceStateMsg       *cur_force,
                                             const DroneStateMsg *cur_state,
                                             const SimParams     *params,
                                             int                  fd_to_d,
                                             FILE                *logfile,
                                             bool                 paused)
{
    if (paused) {
        // When paused, do not do any automatic wall corrections.
        return;
    }

    // 1) Compute continuous repulsive vector P from the four walls.
    double Px = 0.0, Py = 0.0;
    compute_wall_repulsive_P(cur_state, params, &Px, &Py);

    // If P is very small, no need to do anything special.
    double Pnorm2 = Px*Px + Py*Py;
    if (Pnorm2 < 1e-6) {
        return;
    }

    // 2) Find which of the 8 discrete directions best matches P.
    int idx = best_dir8_for_vector(Px, Py);
    if (idx < 0) {
        // No direction with positive projection => skip.
        return;
    }

    char   best_key = g_dir8[idx].key;
    double best_dot = dot2(Px, Py, g_dir8[idx].ux, g_dir8[idx].uy);

    if (best_dot <= 0.0) {
        return;
    }

    // 3) Convert best_dot (a continuous magnitude) into a "virtual key"
    //    contribution. Important difference vs before:
    //    we DO NOT add it into cur_force; we just build a temporary F_rep.
    double step_force = params->force_step;

    // Approximate how many key steps would correspond to best_dot.
    double n_steps_f = best_dot / (step_force + 1e-9);

    // Round and clamp to keep repulsion moderate (no explosions).
    int n_steps = (int)(n_steps_f + 0.5);
    if (n_steps < 1) n_steps = 1;
    if (n_steps > 2) n_steps = 2;   // smaller than before to reduce bouncing

    // 4) Convert chosen key to unit direction (dFx, dFy).
    double dFx, dFy;
    direction_from_key(best_key, &dFx, &dFy);

    // Build repulsive force vector F_rep (NOT stored in cur_force).
    double Frep_x = n_steps * dFx * step_force;
    double Frep_y = n_steps * dFy * step_force;

    // 5) Construct the total force to send: user force + repulsive force.
    ForceStateMsg out = *cur_force;  // copy the persistent user force
    out.Fx += Frep_x;
    out.Fy += Frep_y;
    out.reset = 0;

    // 6) Send this total force to D.
    if (write(fd_to_d, &out, sizeof(out)) == -1) {
        perror("[B] write to D failed (wall repulsion)");
    } else if (logfile) {
        fprintf(logfile,
                "WALL_REPULSION: P=(%.2f,%.2f), best_key=%c, n_steps=%d, "
                "Frep=(%.2f,%.2f), outFx=%.2f outFy=%.2f\n",
                Px, Py, best_key, n_steps,
                Frep_x, Frep_y, out.Fx, out.Fy);
        fflush(logfile);
    }
}




void run_server_process(int fd_kb, int fd_to_d, int fd_from_d, int fd_obs, int fd_tgt, SimParams params) 
{
    // --- Initialize ncurses ---
    initscr();
    cbreak();
    noecho();
    curs_set(0);  // hide cursor

    // --- Open logfile ---
    FILE *logfile = fopen("log.txt", "w");
    if (!logfile) {
        endwin();
        die("[B] cannot open log.txt");
    }

    // -------------------------------------------
    // Initialize 8 obstacles in world coordinates
    // (you can tweak positions later)
    // -------------------------------------------
    double R = params.world_half * 0.5;  // radius for placing obstacles
    g_obstacles[0] = (Obstacle){ +0.0, +R };
    g_obstacles[1] = (Obstacle){ +R, +0.0 };
    g_obstacles[2] = (Obstacle){ +0.0, -R };
    g_obstacles[3] = (Obstacle){ -R, +0.0 };
    g_obstacles[4] = (Obstacle){ -R, -0.0 };
    g_obstacles[5] = (Obstacle){ +R * 0.7, +R * 0.7 };
    g_obstacles[6] = (Obstacle){ -R * 0.7, +R * 0.7 };
    g_obstacles[7] = (Obstacle){ -R * 0.7, -R * 0.7 };


    // --- Blackboard state (shared model of the world) ---
    ForceStateMsg cur_force;
    cur_force.Fx = 0.0;
    cur_force.Fy = 0.0;
    cur_force.reset = 0;

    DroneStateMsg cur_state = (DroneStateMsg){0.0, 0.0, 0.0, 0.0};
    char last_key = '?';
    bool paused = false;

    // Send initial zero-force to D so it starts defined.
/*     if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
        fclose(logfile);
        endwin();
        die("[B] initial write to D failed");
    } */
    // Replaced: send to helper rather than directly write to D
        // Initial state is zero, so cur_state is still {0,0,0,0}.
    // Send initial total force (which is just user=0 + obstacles repulsion).
    send_total_force_to_d(&cur_force,
                          &cur_state,
                          &params,
                          g_obstacles,
                          NUM_OBSTACLES,
                          fd_to_d,
                          logfile,
                          "init");


    int max_y, max_x;

    // --- Main event loop ---
    while (1) {
        // ------------------------------------------------------------------
        // 1) Query current terminal size (for resizing).
        // ------------------------------------------------------------------
        getmaxyx(stdscr, max_y, max_x);

        // Layout planning:
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

        // Right inspection panel width
        int insp_width = 15;               // was 35
        if (max_x < insp_width + 10) {
            insp_width = max_x / 4;
            if (insp_width < 10) insp_width = 10;
        }
        int insp_start_x = max_x - insp_width;
        if (insp_start_x < 1) insp_start_x = 1;

        // World area is below separator.
        int world_top    = sep_y + 1;
        if (world_top > content_bottom) world_top = content_top + 1;
        int world_bottom = content_bottom;
        int world_height = world_bottom - world_top + 1;
        if (world_height < 1) world_height = 1;

        // Left world width.
        int main_width = insp_start_x - 2;
        if (main_width < 10) main_width = 10;

        // nedded to plot the colored items
        initscr();
        start_color();

        // Enable color if terminal supports it
        if (has_colors() == FALSE) {
            endwin();
            printf("Your terminal does not support colors.\n");
        return 1;
        }

        // Define a custom colors for target and obstacles
        // RGB scaled 0–1000 in ncurses
        init_color(COLOR_YELLOW, 1000, 500, 0);   // For obs: Strong orange (R=1000, G=500, B=0)
        init_color(COLOR_GREEN, 0, 1000, 0);   // For targets: light green
        // Assign color-pair IDs 1 and 2: ORANGE foreground, BLACK background
        init_pair(1, COLOR_YELLOW, COLOR_BLACK);
        init_pair(2, COLOR_GREEN, COLOR_BLACK);
       
        // ------------------------------------------------------------------
        // 2) Use select() to wait for data from keyboard and dynamics.
        //    Also handle EINTR (e.g., from SIGWINCH on resize).
        // ------------------------------------------------------------------
        fd_set rfds;
        // int maxfd = imax(fd_kb, fd_from_d) + 1;
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
                    // Interrupted by signal (like resize) → retry
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
        // 3) Handle keyboard input from I (if available).
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

            // (a) Global quit
            if (km.key == 'q') {
                fprintf(logfile, "QUIT requested by 'q'\n");
                fflush(logfile);
                break;
            }

            // (b) Pause toggle
            if (km.key == 'p') {
                paused = !paused;

                if (paused) {
                    // When entering pause, we also zero the force.
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
            // (c) Reset (uppercase R)
            else if (km.key == 'R') {
                // Reset server-side state
                cur_state.x  = 0.0;
                cur_state.y  = 0.0;
                cur_state.vx = 0.0;
                cur_state.vy = 0.0;

                // Reset forces
                cur_force.Fx = 0.0;
                cur_force.Fy = 0.0;
                cur_force.reset = 1; // signal D to reset its state

    /*             if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
                    fclose(logfile);
                    endwin();
                    die("[B] write to D failed (reset)");
                } */
                // Replaced
                send_total_force_to_d(&cur_force,
                      &cur_state,
                      &params,
                      g_obstacles,
                      NUM_OBSTACLES,
                      fd_to_d,
                      logfile,
                      "key");

                cur_force.reset = 0; // clear locally
                paused = false;      // also unpause

                fprintf(logfile, "RESET requested (R)\n");
                fflush(logfile);
            }
            // (d) Directional keys and the break 'd'
            else {
                double dFx, dFy;
                direction_from_key(km.key, &dFx, &dFy);

                if (!paused) {
                    if (km.key == 'd') {
                        // Brake: zero forces
                        cur_force.Fx = 0.0;
                        cur_force.Fy = 0.0;
                    } else {
                        // Accumulate new force
                        cur_force.Fx += dFx * params.force_step;
                        cur_force.Fy += dFy * params.force_step;
                    }

                    cur_force.reset = 0;

/*                     if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
                        fclose(logfile);
                        endwin();
                        die("[B] write to D failed (force)");
                    } */
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
                    // Paused → ignore directional changes (but still log)
                    fprintf(logfile,
                            "KEY: %c ignored (PAUSED)\n", km.key);
                    fflush(logfile);
                }
            }
        }

        // ------------------------------------------------------------------
        // 4) Handle state updates from D (if available).
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

            fprintf(logfile,
                    "STATE: x=%.2f y=%.2f vx=%.2f vy=%.2f\n",
                    s.x, s.y, s.vx, s.vy);
            fflush(logfile);
            
            // **** ADD decrement obstacle & target lifetimes logic here ****
            // considers each time input is received from d, 1 sim time had elapsed.
            // Decrement obstacle lifetimes
            if (!paused){    // Only age obstacles & targets when simulation is running
                for (int i = 0; i < NUM_OBSTACLES; ++i) {
                    if (g_obstacles[i].active && g_obstacles[i].life_steps > 0) {
                        g_obstacles[i].life_steps--;   // decrease 1 step from its lifetime
                        if (g_obstacles[i].life_steps == 0) {
                            g_obstacles[i].active = 0;
                        }
                    }
                }

                // Decrement target lifetimes
                for (int i = 0; i < NUM_TARGETS; ++i) {
                    if (g_targets[i].active && g_targets[i].life_steps > 0) {
                        g_targets[i].life_steps--;
                        if (g_targets[i].life_steps == 0) {
                            g_targets[i].active = 0;
                        }
                    }
                }
            }
            // Then, send updated total force (evenif user doesn't send cmd) (user + obstacles)
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
        //Handle obstacle-set messages in B
        // ------------------------------------------------------------------
        
        // ----- New obstacle set from O -----
        if (FD_ISSET(fd_obs, &rfds)) {
            ObstacleSetMsg msg;
            int n = read(fd_obs, &msg, sizeof(msg));
            if (n <= 0) {
                // O process ended; you may log and continue
                mvprintw(0, 1, "[B] Obstacle generator ended.");
                // optionally: fd_obs = -1 and stop using it
            } else {
                if (paused){
                    // Read but ignore new obstacles while paused
                    fprintf(logfile,
                            "[B] Received obstacle set but PAUSED -> ignored.\n");
                    fflush(logfile);
                } else {
                    int requested = msg.count;
                    if (requested > NUM_OBSTACLES) requested = NUM_OBSTACLES;

                    // Use a clearance similar to what we used for targets
                    double tgt_clearance = params.world_half * 0.15;

                    int accepted = 0;

                    for (int i = 0; i < requested; ++i) {
                        double x = msg.obs[i].x;
                        double y = msg.obs[i].y;

                        // Reject if too close to any active target
                        if (too_close_to_any_pointlike(x, y,
                               (PointLike*)g_obstacles,
                               NUM_OBSTACLES,
                               obs_clearance)){
                            fprintf(logfile,
                                    "[B] Obstacle (%.2f, %.2f) rejected: too close to target.\n",
                                    x, y);
                            continue;
                        }

                        // If accepted index is within our capacity, store it
                        if (accepted < NUM_OBSTACLES) {
                            g_obstacles[accepted].x          = x;
                            g_obstacles[accepted].y          = y;
                            g_obstacles[accepted].life_steps = msg.obs[i].life_steps;
                            g_obstacles[accepted].active     = 1;
                            accepted++;
                        }
                    }

                    // Deactivate remaining slots
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
        //Handle target-set messages in B
        // ------------------------------------------------------------------

        // ----- New target set from T -----
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
            double obs_clearance   = params.world_half * 0.30; // away from obstacles

            int accepted = 0;

            for (int i = 0; i < requested; ++i) {
                double x = msg.tgt[i].x;
                double y = msg.tgt[i].y;

                // 1) reject if too close to walls
                if (target_too_close_to_wall(x, y, &params, wall_margin)) {
                    fprintf(logfile,
                            "[B] Target (%.2f,%.2f) rejected: too close to walls.\n",
                            x, y);
                    continue;
                }

                // 2) reject if too close to obstacles
                if (too_close_to_any_pointlike(x, y,
                               (PointLike*)g_targets,
                               NUM_TARGETS,
                               tgt_clearance)){
                    fprintf(logfile,
                            "[B] Target (%.2f,%.2f) rejected: too close to obstacles.\n",
                            x, y);
                    continue;
                }

                // If we get here, target is acceptable.
                if (accepted < NUM_TARGETS) {
                    g_targets[accepted].x          = x;
                    g_targets[accepted].y          = y;
                    g_targets[accepted].life_steps = msg.tgt[i].life_steps;
                    g_targets[accepted].active     = 1;
                    accepted++;
                }
            }

            // Deactivate remaining slots
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
        // 4.5) Apply wall repulsion as a virtual key (assignment-style).
        //      This uses the current state and params to compute a repulsive
        //      vector P from the walls, then projects it onto the 8 key
        //      directions and applies a "virtual key" to cur_force.
        // ------------------------------------------------------------------
/*         apply_wall_repulsion_virtual_key(&cur_force,
                                         &cur_state,
                                         &params,
                                         fd_to_d,
                                         logfile,
                                         paused);
 */
        // ------------------------------------------------------------------
        // 5) Draw UI (world + inspection panel).
        // ------------------------------------------------------------------
        erase();
        box(stdscr, 0, 0);

        // Top info lines
        mvprintw(top_info_y1, 2,
                 "Controls: w e r / s d f / x c v | d=brake, p=pause, R=reset, q=quit");
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
        double scale_x = main_width  / (2.0 * world_half);   // mapping world coordinates to the drone world on the display scree
        double scale_y = world_height / (2.0 * world_half);
        if (scale_x <= 0) scale_x = 1.0;
        if (scale_y <= 0) scale_y = 1.0;

        int sx = (int)(cur_state.x * scale_x) + main_width / 2 + 1;
        int sy = (int)(-cur_state.y * scale_y) + world_top + world_height / 2;

        if (sx < 1) sx = 1;
        if (sx > main_width) sx = main_width;
        if (sy < world_top) sy = world_top;
        if (sy > world_bottom) sy = world_bottom;

        mvaddch(sy, sx, '+'); // draw drone

        // Drawing active obstacles as 'o' in the drone world
        for (int k = 0; k < NUM_OBSTACLES; ++k) {
            if (!g_obstacles[k].active) continue;  // skip inactive 

            int ox = (int)(g_obstacles[k].x * scale_x) + main_width / 2 + 1;
            int oy = (int)(-g_obstacles[k].y * scale_y) + world_top + world_height / 2;

            if (ox < 1) ox = 1;
            if (ox > main_width) ox = main_width;
            if (oy < world_top) oy = world_top;
            if (oy > world_bottom) oy = world_bottom;
            
            attron(COLOR_PAIR(1));
            mvaddch(oy, ox, 'o');  // TODO: add color to make them orange
            
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
            mvaddch(ty, tx, 'T');  // placeholder, later make them numbered
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
        }

        refresh();
    }

    // --- Cleanup ---
    fclose(logfile);
    endwin();
    close(fd_kb);
    close(fd_to_d);
    close(fd_from_d);
    exit(EXIT_SUCCESS);
}
