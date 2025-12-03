// params.c
// Handling parameters
//  - Set defaults
//  - Load from "params.txt"-style key=value file
// ======================================================================

#include "headers/params.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>


// Helper: trim leading and trailing whitespace in-place.
// ----------------------------------------------------------------------
static void trim(char *s) {
    if (!s) return;

    // 1) Trim leading whitespace
    char *start = s;
    while (*start == ' ' || *start == '\t' ||
           *start == '\n' || *start == '\r')
        start++;

    if (start != s) {
        memmove(s, start, strlen(start) + 1);
    }

    // 2) Trim trailing whitespace
    size_t len = strlen(s);
    while (len > 0 &&
           (s[len-1] == ' ' || s[len-1] == '\t' ||
            s[len-1] == '\n' || s[len-1] == '\r')) {
        s[len-1] = '\0';
        len--;
    }
}

// Initialize default parameters (used if no params.txt exists).
// ----------------------------------------------------------------------
void init_default_params(SimParams *p) {
    p->mass       = 1.0;
    p->visc       = 1.0;
    p->dt         = 0.05;
    p->force_step = 5.0;
    p->world_half = 50.0;

    // unused now, but ready for walls
    p->wall_clearance = 5.0;
    p->wall_gain      = 0.1;
}

// Load parameters from a simple "key=value" file.
// Unknown keys are ignored. Missing file → keep defaults.
// ----------------------------------------------------------------------
void load_params_from_file(const char *filename, SimParams *p) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        fprintf(stderr,
                "[PARAMS] Could not open '%s'. Using default parameters.\n",
                filename);
        return;
    }

    fprintf(stderr, "[PARAMS] Loading parameters from '%s'...\n", filename);

    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        trim(line);
        if (line[0] == '\0' || line[0] == '#')
            continue;

        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';

        char *key = line;
        char *val = eq + 1;

        trim(key);
        trim(val);

        double d = strtod(val, NULL);

        if      (strcmp(key, "mass")           == 0) p->mass       = d;
        else if (strcmp(key, "visc")           == 0) p->visc       = d;
        else if (strcmp(key, "dt")             == 0) p->dt         = d;
        else if (strcmp(key, "force_step")     == 0) p->force_step = d;
        else if (strcmp(key, "world_half")     == 0) p->world_half = d;
        else if (strcmp(key, "wall_clearance") == 0) p->wall_clearance = d;
        else if (strcmp(key, "wall_gain")      == 0) p->wall_gain      = d;
        else {
            fprintf(stderr, "[PARAMS] Unknown key '%s', ignoring.\n", key);
        }
    }

    fclose(fp);

    fprintf(stderr,
            "[PARAMS] Loaded: mass=%.3f, visc=%.3f, dt=%.3f, force_step=%.3f, "
            "world_half=%.3f, wall_clearance=%.3f, wall_gain=%.3f\n",
            p->mass, p->visc, p->dt, p->force_step,
            p->world_half, p->wall_clearance, p->wall_gain);
}
