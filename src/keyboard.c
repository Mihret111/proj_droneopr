// keyboard.c
// Implements the keyboard process (I).
// This is the ONLY process that reads from stdin.
// ======================================================================

#include "headers/messages.h"
#include "headers/util.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

// ----------------------------------------------------------------------
// Defines keyboard process:
//   - Reads characters from stdin 
//   - Wraps each into KeyMsg and writes to the pipe to B.
//   - Exits on EOF or 'q'.
// ----------------------------------------------------------------------
void run_keyboard_process(int write_fd) {
    // Opens log file
    FILE *log = open_process_log("keyboard", "I");
    if (!log) log = stderr;   // <-- don't die, just log to stderr
    fprintf(log, "[I] Keyboard started | PID = %d\n", getpid());
    fprintf(log,
    "[I] Use w e r / s d f / x c v to command force.\n"
    "[I] 'd' = brake, 'p' = pause, 'O' = reset, 'q' = quit.\n");
    fflush(log);

    // Unbuffers stdout so debug messages appear immediately.
    setbuf(stdout, NULL);


    while (1) {
        // reads one character from stdin
        int c = getchar(); // blocking read 

        if (c == EOF) {
            fprintf(log, "[I] EOF on stdin, exiting keyboard process.\n");
            break;
        }

        KeyMsg km;
        km.key = (char)c;
        fprintf(log, "[I] key='%c' (%d)\n", km.key, (int)km.key);


        // Sends key to B through pipe.
        if (write(write_fd, &km, sizeof(km)) == -1) {
            fprintf(log, "[I] write to B failed");

            break;
        }

        if (km.key == 'q') {
            fprintf(log, "[I] 'q' pressed, exiting keyboard process.\n");
            break;
        }
    }
    // Final cleanup
    if (log) {
        fprintf(log, "[I] Exiting.\n");
        fclose(log);
    }
    // Closes pipe to B 
    close(write_fd);
    exit(EXIT_SUCCESS);  
}
