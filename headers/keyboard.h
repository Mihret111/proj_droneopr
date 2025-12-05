// keyboard.h
// Interface for the keyboard process (I)
// ======================================================================

#ifndef KEYBOARD_H
#define KEYBOARD_H

// Runs the keyboard process:
//   - Reads from stdin
//   - Sends KeyMsg to B via write_fd
void run_keyboard_process(int write_fd);

#endif // KEYBOARD_H
