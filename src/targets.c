#include "headers/messages.h"
#include "headers/params.h"
#include "headers/targets.h"
#include "headers/util.h"

#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

#define _GNU_SOURCE

#include <math.h>

Target g_targets[NUM_TARGETS];

/**
 * @brief Run the Target Generator (T) process.
 * 
 * @details
 * Periodically spawns targets and sends them to the Server (B).
 * - **Generation Logic**:
 *   - Samples random positions using polar coordinates (r, theta) for uniform disk distribution.
 *   - Assigns a finite lifetime to each batch.
 *   - Server (B) performs the final validation (filtering unsafe targets) before accepting.
 * 
 * @param pipe_fd Write-end pipe to Server (B).
 * @param params  Simulation parameters (used for world boundaries).
 */
void run_target_process(int write_fd, SimParams params) {
    // opens log file
    FILE *log = open_process_log("targets", "T");
    if (!log) log = stderr;   // <-- don't die, just log to stderr

    fprintf(log, "[T] Targets started | PID = %d\n", getpid());
    srand((unsigned)time(NULL) ^ (getpid() << 1));

    double world_half = params.world_half;

    // Parameters:
    // Determines how long each target stays alive in terms of B's "state updates".
    const int life_steps_default = 1000;   // 1000 physics steps

    // Places targets mostly in the central area, radius < central_factor * world_half.
    const double central_factor  = 0.5;    // inner 50% radius
    double max_r                 = world_half * central_factor;

    // Defines minimum spacing between targets in the same batch.
    const double spacing_factor  = 0.12;   // 12% of world_half
    double min_spacing           = world_half * spacing_factor;
    double min_spacing2          = min_spacing * min_spacing;

    // Defines max attempts per target to find a non-overlapping position.
    const int max_attempts       = 50;  // 50 attempts

    // Determines how often to *try* to spawn a new batch of targets (in seconds)
    const unsigned spawn_interval_sec = 50;   // 50 seconds

    while (1) {
        TargetSetMsg msg;

        // Determines how many targets per batch. Can use MAX_TARGETS,
        // or choose a smaller number if fewer are desired.
        int batch_count = MAX_TARGETS;   // targets per a single batch
        msg.count = batch_count;

        for (int i = 0; i < batch_count; ++i) {
            int attempts = 0;
            int placed   = 0;

            while (attempts < max_attempts) {
                attempts++;

                // Samples position in a central disk of radius max_r:
                //
                // - theta ∈ [0, 2π)
                // - r ∈ [0, max_r], but to make uniform in area
                //   samples sqrt(u) * max_r
                double theta = rand_in_range(0.0, 2.0 * M_PI);
                double u     = (double)rand() / (double)RAND_MAX; // [0,1]
                double r     = sqrt(u) * max_r;   // area-uniform disk

                double x = r * cos(theta);
                double y = r * sin(theta);

                // Checks spacing with already placed targets in this batch.
                int ok = 1;
                for (int j = 0; j < i; ++j) {
                    double dx = x - msg.tgt[j].x;
                    double dy = y - msg.tgt[j].y;
                    double d2 = dx*dx + dy*dy;
                    if (d2 < min_spacing2) {
                        ok = 0;
                        break;
                    }
                }

                if (ok) {
                    msg.tgt[i].x          = x;
                    msg.tgt[i].y          = y;
                    msg.tgt[i].life_steps = life_steps_default;
                    placed = 1;
                    break;
                }
            }

            if (!placed) {
                // Fallback: Picks some central point without spacing check
                double theta = rand_in_range(0.0, 2.0 * M_PI);
                double u     = (double)rand() / (double)RAND_MAX;
                double r     = sqrt(u) * max_r;

                double x = r * cos(theta);
                double y = r * sin(theta);

                msg.tgt[i].x          = x;
                msg.tgt[i].y          = y;
                msg.tgt[i].life_steps = life_steps_default;
            }
        }

        // Sends batch to B.
        if (write(write_fd, &msg, sizeof(msg)) == -1) {
            perror("[T] write to B failed");
            break;
        }

        // Logs the sending event
        fprintf(log, "[T] sending batch count=%d ...\n", msg.count);
        fflush(log);


        // Waits before generating the next batch.
        sleep(spawn_interval_sec);
    }
    // Final cleanup
    if (log) {
        fprintf(log, "[T] Exiting.\n");
        fclose(log);
    }
    close(write_fd);
    exit(EXIT_SUCCESS);
}
