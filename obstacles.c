#include "headers/messages.h"
#include "headers/params.h"
#include "headers/obstacles.h"
#include "headers/util.h"

#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

Obstacle g_obstacles[NUM_OBSTACLES];


// ----------------------------------------------------------------------
// Defines the obstacle process:
//   - Sends ObstacleSetMsg to B via write_fd
//   - Spawns obstacles at regular intervals
// ----------------------------------------------------------------------
void run_obstacle_process(int write_fd, SimParams params) {
    srand((unsigned)time(NULL) ^ getpid());

    double world_half = params.world_half;


    // Tunable parameters
    // Determines how long each obstacle lives in terms of B's "state update" steps
    // Decides how long a batch stays on screen
    const int life_steps_default = 1000;    // e.g. 800 physics steps

    // Defines margin from walls: keep obstacles inside this inner box
    // Example: 20% margin on each side
    const double margin_factor   = 0.20;
    double margin                = world_half * margin_factor;

    // Defines minimum spacing between obstacles in the same batch
    const double spacing_factor  = 0.15;   // 15% of world_half
    double min_spacing           = world_half * spacing_factor;
    double min_spacing2          = min_spacing * min_spacing;

    // Defines maximum attempts per obstacle to find a valid (non-overlapping) position.
    const int max_attempts       = 50;

    // Determines how often to *try* to spawn a new batch of obstacles (in real seconds).
    // Decides how soon O tries to create the next batch
    const unsigned spawn_interval_sec = 45;   // 40 did good visually, test more
    
    
    while (1) {
        ObstacleSetMsg msg;
        msg.count = MAX_OBSTACLES;  // we'll try to generate this many each time

        // Samples a position for each obstacle in this batch that:
        //  -- Is inside the inner box (margin from walls)
        //  -- Is at least min_spacing away from previously generated obstacles
        for (int i = 0; i < msg.count; ++i) {
            int attempts = 0;
            int placed   = 0;
            while (attempts < max_attempts) {
                attempts++;

                // Samples inside inner box: [-world_half+margin, +world_half-margin]
                double x = rand_in_range(-world_half + margin, +world_half - margin);
                double y = rand_in_range(-world_half + margin, +world_half - margin);

                // Checks spacing with all previously placed obstacles in this batch
                int ok = 1;
                for (int j = 0; j < i; ++j) {
                    double dx = x - msg.obs[j].x;
                    double dy = y - msg.obs[j].y;
                    double d2 = dx*dx + dy*dy;
                    if (d2 < min_spacing2) {
                        ok = 0;
                        break;
                    }
                }

                if (ok) {
                    msg.obs[i].x          = x;
                    msg.obs[i].y          = y;
                    msg.obs[i].life_steps = life_steps_default;
                    placed = 1;
                    break;
                }
            }

            if (!placed) {
                // Falls back to "somewhere in the inner box without spacing check" if a good spot is not found.
                double x = rand_in_range(-world_half + margin, +world_half - margin);
                double y = rand_in_range(-world_half + margin, +world_half - margin);

                msg.obs[i].x          = x;
                msg.obs[i].y          = y;
                msg.obs[i].life_steps = life_steps_default;
            }
        }

        // Sends the whole batch to B.
        if (write(write_fd, &msg, sizeof(msg)) == -1) {
            perror("[O] write to B failed");
            break;  // exit the loop -> process ends
        }

        // Waits a while before attempting to spawn the next batch.
        sleep(spawn_interval_sec);
    }

    close(write_fd);
    _exit(EXIT_SUCCESS);
}