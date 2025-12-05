// obstacles.h
// Interface for the obstacle process (O)
// ======================================================================

#ifndef OBSTACLES_H
#define OBSTACLES_H


#define NUM_OBSTACLES 12 // Defines number of obstacles

typedef struct {
    double x;
    double y;
    int    active;      // 1 = currently present, 0 = off
    int    life_steps;  // Indicates how many state updates left before disappearing
} Obstacle;

extern Obstacle g_obstacles[NUM_OBSTACLES];

// Runs the obstacle process:
//   - fd_write     : write-end of pipe O->B
//   - fd_read   : read-end of pipe B->O
void run_obstacle_process(int write_fd, SimParams params) ;
#endif // OBSTACLES_H
