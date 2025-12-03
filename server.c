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

#include <ncurses.h>
#include <stdio.h>
#include <stdlib.h>     // exit, strtod
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>

#include <sys/select.h>   // for fd_set, FD_ZERO, FD_SET, select()
#include <math.h>  // for sqrt, to be used in key mapping instead of hard code, REP

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




void run_server_process(int fd_kb, int fd_to_d, int fd_from_d, SimParams params) {
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

    // --- Blackboard state (shared model of the world) ---
    ForceStateMsg cur_force;
    cur_force.Fx = 0.0;
    cur_force.Fy = 0.0;
    cur_force.reset = 0;

    DroneStateMsg cur_state = (DroneStateMsg){0.0, 0.0, 0.0, 0.0};
    char last_key = '?';
    bool paused = false;

    // Send initial zero-force to D so it starts defined.
    if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
        fclose(logfile);
        endwin();
        die("[B] initial write to D failed");
    }

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

        // ------------------------------------------------------------------
        // 2) Use select() to wait for data from keyboard and dynamics.
        //    Also handle EINTR (e.g., from SIGWINCH on resize).
        // ------------------------------------------------------------------
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

                if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
                    fclose(logfile);
                    endwin();
                    die("[B] write to D failed (reset)");
                }

                cur_force.reset = 0; // clear locally
                paused = false;      // also unpause

                fprintf(logfile, "RESET requested (R)\n");
                fflush(logfile);
            }
            // (d) Direction / brake keys
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

                    if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
                        fclose(logfile);
                        endwin();
                        die("[B] write to D failed (force)");
                    }

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
        double scale_x = main_width  / (2.0 * world_half);
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
