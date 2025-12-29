// watchdog.c
// Signal-based Watchdog (W)
//
// Heartbeat mechanism:
//   - B sends SIGUSR1 to W whenever a new DroneStateMsg arrives.
//   - W updates "last_beat_time" on SIGUSR1.
//
// Watchdog actions:
//   - If no heartbeat for warn_sec: send SIGUSR2 to B (warning notification).
//   - If no heartbeat for kill_sec: send SIGTERM to all processes (stop system).
//
// Why signals:
//   - W is signal-based.
//   - Heartbeat = "I'm alive" → classic SIGUSR1 usage.

#define _POSIX_C_SOURCE 200809L

#include "headers/watchdog.h"
#include "headers/util.h"   // die()

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <string.h>

// Global/shared state inside watchdog process only
static volatile sig_atomic_t g_got_beat = 0;

// Store last heartbeat time (monotonic clock)
static struct timespec g_last_beat_ts;

// Received heartbeat from B
static void on_sigusr1(int signo) {
    (void)signo;
    g_got_beat = 1;
}

// Helper: get monotonic time in seconds (double)
static double now_monotonic_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

// Helper: convert a timespec to seconds
static double ts_to_sec(const struct timespec *ts) {
    return (double)ts->tv_sec + (double)ts->tv_nsec * 1e-9;
}

void run_watchdog_process(int cfg_read_fd, int warn_sec, int kill_sec) {
    
    // 1) Open watchdog log file
    FILE *log = open_process_log("watchdog", "W");
    if (!log) {
        // If logs/ doesn't exist, don't crash silently
        perror("[W] fopen logs/watchdog.log");       // print to stderr
        // exit(EXIT_FAILURE);
    }

    fprintf(log, "[W] Watchdog started | PID = %d\n", getpid());

    // 2) Read the PIDs struct from master (one-time configuration)
    WatchPids p;
    int n = read(cfg_read_fd, &p, sizeof(p));
    if (n != (int)sizeof(p)) {
        if (log) fprintf(log, "[W] ERROR: could not read WatchPids (n=%d)\n", n);
        if (log) fclose(log);
        close(cfg_read_fd);
        exit(EXIT_FAILURE);
    }
    close(cfg_read_fd);

    if (log) {
        fprintf(log, "[W] Started. Watching PIDs: B=%d I=%d D=%d O=%d T=%d\n",
                (int)p.pid_B, (int)p.pid_I, (int)p.pid_D, (int)p.pid_O, (int)p.pid_T);
        fprintf(log, "[W] warn_sec=%d kill_sec=%d\n", warn_sec, kill_sec);
        fflush(log);
    }

    // 3) Install signal handler for heartbeat (SIGUSR1)
    struct sigaction sa; 
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_sigusr1; 
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART; // restart interrupted syscalls where possible

    if (sigaction(SIGUSR1, &sa, NULL) == -1) {
        if (log) fprintf(log, "[W] sigaction(SIGUSR1) failed: %s\n", strerror(errno));
        if (log) fclose(log);
        exit(EXIT_FAILURE);
    }

    // Initialize last beat time to "now" (gives system time to start)
    clock_gettime(CLOCK_MONOTONIC, &g_last_beat_ts);

    // 4) Main loop: check heartbeat timing
    int warned = 0;
    while (1) {
        // If we got heartbeat since last loop, update last_beat_ts
        if (g_got_beat) {
            g_got_beat = 0;
            clock_gettime(CLOCK_MONOTONIC, &g_last_beat_ts);
            warned = 0; // reset warning state once heartbeat resumes
        }

        double elapsed = now_monotonic_sec() - ts_to_sec(&g_last_beat_ts);

        // WARN stage: notify B (one-time per missing-heartbeat episode)
        if (!warned && elapsed >= (double)warn_sec) {
            warned = 1;
            if (log) {
                fprintf(log, "[W] WARNING: no heartbeat for %.2f sec → SIGUSR2 to B\n", elapsed);
                fflush(log);
            }
            // SIGUSR2 is our "watchdog warning" notification to B
            kill(p.pid_B, SIGUSR2);
        }

        // KILL stage: stop the whole system
        if (elapsed >= (double)kill_sec) {
            if (log) {
                fprintf(log, "[W] TIMEOUT: no heartbeat for %.2f sec → stopping system (SIGTERM)\n", elapsed);
                fflush(log);
            }

            // Termination order: first tell B (so UI can exit), then the others active processes
            kill(p.pid_B, SIGTERM);
            kill(p.pid_I, SIGTERM);
            kill(p.pid_D, SIGTERM);
            kill(p.pid_O, SIGTERM);
            kill(p.pid_T, SIGTERM);

            break;
        }

        // Sleep a bit (low CPU usage)
        struct timespec ts;
        ts.tv_sec = 0;
        ts.tv_nsec = 100 * 1000 * 1000; // 100 ms
        nanosleep(&ts, NULL);
    }

    if (log) {
        fprintf(log, "[W] Exiting.\n");
        fclose(log);
    }

    exit(EXIT_SUCCESS);
}
