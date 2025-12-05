// keyboard.c
// Implements the keyboard process (I).
// This is the ONLY process that reads from stdin.
// ======================================================================

#include "headers/messages.h"

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
    // Unbuffers stdout so debug messages appear immediately.
    setbuf(stdout, NULL);

    fprintf(stderr,
        "[I] Keyboard process started.\n"
        "[I] Use w e r / s d f / x c v to command force.\n"
        "[I] 'd' = brake, 'p' = pause, 'O' = reset, 'q' = quit.\n");

    while (1) {
        int c = getchar(); // blocking read from stdin

        if (c == EOF) {
            fprintf(stderr, "[I] EOF on stdin, exiting keyboard process.\n");
            break;
        }

        KeyMsg km;
        km.key = (char)c;

        // Sends key to B through pipe.
        if (write(write_fd, &km, sizeof(km)) == -1) {
            perror("[I] write to B failed");
            break;
        }

        if (km.key == 'q') {
            fprintf(stderr, "[I] 'q' pressed, exiting keyboard process.\n");
            break;
        }
    }

    close(write_fd);
    exit(EXIT_SUCCESS);
}
