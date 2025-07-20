# Lebre Cat TV - Carriage Stabilization Project

This project implements a Sugeno-type Fuzzy Logic Controller (FLC) to stabilize a carriage at the highest point of a rotating wheel. The entire system is written in Python 3 and designed to run on a Raspberry Pi 4, but includes mock hardware interfaces for development and testing on Windows.

## Project Structure

The project is organized into a modular structure as specified in the requirements document:

lebre-cat-tv/
├── .vscode/ # VS Code project settings
├── config/ # Configuration files
│ └── flc_config.json
├── flc/ # Core Fuzzy Logic Controller modules
├── platform/ # Hardware abstraction for platform-specifics (motor)
├── sensor/ # Hardware abstraction for sensors (IMU)
├── test/ # Pytest unit tests
├── utils/ # Helper modules (logging, profiling)
├── main.py # Main application entry point
└── README.md # This file


## Features

- **Sugeno Fuzzy Logic Controller**: Implements a complete FLC with fuzzification, a rule engine, and defuzzification steps.
- **Modular Design**: Code is separated by concern into distinct packages (`flc`, `sensor`, `platform`).
- **Configurable**: All FLC parameters, including membership functions and the rule base, are defined in `config/flc_config.json`.
- **Real-Time Control Loop**: The main application runs a continuous loop at a configurable rate (default 50 Hz).
- **Hardware Abstraction**: Sensor and motor interfaces are mocked to allow for functional testing without physical hardware.
- **Comprehensive Logging**: Generates detailed, structured logs for each component, aiding in debugging and tuning.
- **Unit Tested**: Includes a suite of `pytest` unit tests to ensure the correctness of individual modules.

## Setup and Installation

### Prerequisites

- Python 3.9+
- `pip` for installing packages

### Dependencies

The required Python packages are listed below. You can install them all using `pip`:

```bash
pip install numpy scipy pytest pytest-mock

On a Raspberry Pi, you would also install the hardware-specific libraries:
Bash

pip install adafruit-circuitpython-busio

Running the Application

    Clone the repository or create the project files as specified.
    Install the dependencies as shown above.
    Run the main script from the project's root directory:
    Bash

    python main.py

    The application will start, initialize all components, and begin running the control loop. You will see status messages printed to the console.
    To stop the application, press Ctrl+C.

Running Tests
To run the full suite of unit tests, execute pytest from the project's root directory:
Bash

pytest

Pytest will automatically discover and run all files named test_*.py. You will see a summary of the test results.
Data Logging
The application generates detailed logs for each major component in the logs/ directory. These logs are essential for debugging, performance analysis, and tuning the fuzzy logic system.

    logs/main.log: General application status, startup, and shutdown messages.
    logs/controller.log: High-level FLC cycle information.
    logs/fuzzifier.log: Detailed output of the fuzzification process for each input.
    logs/rule_engine.log: Information on which rules fired and their calculated outputs.
    logs/defuzzifier.log: The final weighted average calculation.
    logs/sensor.log: Raw and processed sensor values.
    logs/motor.log: Commands being sent to the motor driver.
    logs/profiler.log: Execution timing for the control loop.

These logs can be analyzed to understand the controller's behavior under different conditions.

</details>



