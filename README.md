# Drone Game Envionment Simulation Project

A dynamic drone navigation game where the objective is to control a drone to collect as many targets as possible, while also avoiding collisions with obstacles.

## Installation and Running

### Prerequisites
-   GCC compiler (can be installed via `sudo apt-get install build-essential`)
-   `ncurses` library (can be installed via `sudo apt-get install libncurses5-dev`)
-   `make` (can be installed via `sudo apt-get install make`)

### Building and Running the Game
1.  Navigate to the project directory: 
    ```bash
    cd proj_DroneGame/
    ```
2.  Run the following command to build the project:
    ```bash
    make
    ```
    This will generate an executable named `arp1`.
3. Run the executable:
    ```bash
    ./arp1
    ```
4. Clean: To remove all compiled files and start fresh
    ```bash
    make clean
    ```

## Operational Instructions

### Controls
-   **`e`**: Up
-   **`c`**: Down
-   **`s`**: Left
-   **`f`**: Right
-   **`w`**: Up-Left
-   **`r`**: Up-Right
-   **`x`**: Down-Left
-   **`v`**: Down-Right
-   **`d`**: Brake (Zero Force)
-   **`p`**: Pause/Unpause the game.
-   **`O`** (Shift+o): Reset the game (zero position and forces).
-   **`q`**: Quit the game.

### Game Rules
-   **Objective**: Fly the drone to collect as many **Targets** (Green `+`) as possible.
-   **Avoid**: **Obstacles** (Orange `O`).
-   **Physics**: The drone has physical properties (mass, viscosity) and inertia. A continuous *force* applied to it is controlled by the keyboard in a corresponding direction.
-   **Inspection**: The right panel shows the current state (Position, Velocity) and Score, and the time elapsed since a prior target has been collected.
-   **Run-time configuration**: Simulation parameters like Mass (`M`), Viscosity (`K`), and Time step (`dt`) can be modified in `params.txt` file.
