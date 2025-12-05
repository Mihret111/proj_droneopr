// targets.h
// Interface for the target process (T)
// ======================================================================

#ifndef TARGETS_H
#define TARGETS_H
#define _GNU_SOURCE


#define NUM_TARGETS 12  // Defines number of targets

typedef struct {
    double x;
    double y;
    int    active;      // 1 = visible, 0 = not
    int    life_steps;  // Lifetime in steps
} Target;

extern Target g_targets[NUM_TARGETS];

// Runs the target process:
//   - fd_write     : write-end of pipe T->B
//   - fd_read   : read-end of pipe B->T
void run_target_process(int write_fd, SimParams params);
#endif // TARGETS_H