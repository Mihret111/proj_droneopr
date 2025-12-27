// server.h
// Interface for the server/blackboard process (B)
// ======================================================================

#ifndef SERVER_H
#define SERVER_H

#include <sys/types.h>   // for pid_t
#include "params.h"

// Runs the server process:
//   - fd_kb     : read-end of pipe I->B
//   - fd_to_d   : write-end of pipe B->D
//   - fd_from_d : read-end of pipe D->B
//   - fd_obs    : read-end of pipe O->B
//   - fd_tgt    : read-end of pipe T->B
//   - pid_W     : watchdog PID (heartbeat target)
//   - params    : simulation parameters
void run_server_process(int fd_kb, int fd_to_d, int fd_from_d,
                        int fd_obs, int fd_tgt,
                        pid_t pid_W,
                        SimParams params);
#endif // SERVER_H
