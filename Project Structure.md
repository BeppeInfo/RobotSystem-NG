# RobotSystem-NG Project Structure

This document outlines the complete Rust workspace structure for RobotSystem-NG, the Rust port of RobotSystem-Lite.

## Workspace Layout

```
robot-system-ng/
├── Cargo.toml                 # Main workspace configuration
├── README.md                  # Project documentation
├── LICENSE                    # License file
├── .gitignore                # Git ignore patterns
├── src/                      # Main application source
│   ├── main.rs               # Application entry point
│   ├── robot.rs              # Robot control module
│   ├── config.rs             # Configuration management
│   ├── actuator.rs           # Actuator control
│   ├── sensor.rs             # Sensor interface
│   ├── motor.rs              # Motor control
│   ├── control_loop.rs       # Real-time control loop
│   ├── ipc_server.rs         # IPC communication server
│   └── shared_types.rs       # Shared type definitions
├── crates/                   # Workspace crates (submodules)
│   ├── data-logging/         # Data logging functionality
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── logger.rs
│   │       └── formats/
│   ├── kalman-filter/        # Kalman filtering
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── filter.rs
│   │       └── matrix_ops.rs
│   ├── plugin-loader/        # Dynamic plugin loading
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── loader.rs
│   │       └── registry.rs
│   ├── robot-control-interface/  # Robot controller interface
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── controller.rs
│   │       └── traits.rs
│   ├── signal-io-interface/  # Signal I/O interface
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── input.rs
│   │       └── output.rs
│   ├── signal-processing/    # Signal processing utilities
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── filters.rs
│   │       └── transforms.rs
│   ├── multithreading/       # Threading utilities
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── executor.rs
│   │       └── sync.rs
│   ├── precise-timing/       # High-precision timing
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── timer.rs
│   │       └── clock.rs
│   ├── system-linearizer/    # System linearization
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── linearizer.rs
│   │       └── models.rs
│   ├── data-io/             # Data I/O interface
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── json.rs
│   │       └── binary.rs
│   └── ipc-interface/       # IPC communication interface
│       ├── Cargo.toml
│       └── src/
│           ├── lib.rs
│           ├── zmq_transport.rs
│           ├── shared_memory.rs
│           └── tcp_transport.rs
├── config/                  # Configuration files
│   ├── system.json          # System-wide configuration
│   ├── robots/              # Robot configurations
│   │   ├── example_robot.json
│   │   └── rehabilitation_robot.json
│   ├── actuators/           # Actuator configurations
│   │   ├── shoulder_actuator.json
│   │   └── elbow_actuator.json
│   ├── sensors/             # Sensor configurations
│   │   ├── position_encoder.json
│   │   ├── force_sensor.json
│   │   └── imu_sensor.json
│   └── motors/              # Motor configurations
│       ├── servo_motor.json
│       └── stepper_motor.json
├── plugins/                 # Plugin libraries
│   ├── robot_control/       # Robot controller plugins
│   │   ├── impedance_controller.so
│   │   ├── pid_controller.so
│   │   └── adaptive_controller.so
│   └── signal_io/           # Signal I/O plugins
│       ├── can_bus.so
│       ├── ethernet_ip.so
│       ├── modbus_tcp.so
│       └── analog_io.so
├── examples/                # Example applications
│   ├── basic_robot/         # Basic robot example
│   │   ├── Cargo.toml
│   │   └── src/main.rs
│   ├── rehabilitation_demo/ # Rehabilitation demo
│   │   ├── Cargo.toml
│   │   └── src/main.rs
│   └── plugin_development/  # Plugin development examples
│       ├── custom_controller/
│       └── custom_io/
├── tests/                   # Integration tests
│   ├── robot_control_tests.rs
│   ├── ipc_tests.rs
│   └── config_tests.rs
├── benches/                 # Benchmarks
│   ├── control_loop_bench.rs
│   └── ipc_bench.rs
├── docs/                    # Documentation
│   ├── architecture.md
│   ├── configuration.md
│   ├── plugin_development.md
│   └── api_reference.md
└── scripts/                 # Build and deployment scripts
    ├── build.sh
    ├── install.sh
    └── run_tests.sh
```

## Key Components

### Core Application (`src/`)

- **main.rs**: Application entry point with CLI argument parsing
- **robot.rs**: Main robot control logic and state management
- **config.rs**: Configuration loading and validation 
- **actuator.rs**: Actuator abstraction layer
- **sensor.rs**: Sensor interface and data acquisition
- **motor.rs**: Motor control and command output
- **control_loop.rs**: Real-time control loop implementation
- **ipc_server.rs**: Inter-process communication server
- **shared_types.rs**: Common type definitions and data structures

### Workspace Crates (`crates/`)

Each crate is a self-contained library that can be developed and tested independently:

1. **data-logging**: Handles data logging to various formats (CSV, binary, HDF5)
2. **kalman-filter**: Kalman filtering implementation for sensor fusion
3. **plugin-loader**: Dynamic loading of controller and I/O plugins
4. **robot-control-interface**: Traits and types for robot controllers
5. **signal-io-interface**: Traits and types for signal input/output
6. **signal-processing**: Digital signal processing utilities
7. **multithreading**: Threading primitives and utilities
8. **precise-timing**: High-precision timing for real-time control
9. **system-linearizer**: System identification and linearization
10. **data-io**: Data serialization and deserialization
11. **ipc-interface**: Inter-process communication abstractions

### Configuration System (`config/`)

JSON-based configuration files organized by component type:
- **system.json**: Global system settings
- **robots/**: Robot-specific configurations
- **actuators/**: Actuator definitions and parameters
- **sensors/**: Sensor configurations and calibration
- **motors/**: Motor parameters and limits

### Plugin System (`plugins/`)

Dynamic libraries implementing specific functionality:
- **robot_control/**: Control algorithm implementations
- **signal_io/**: Hardware interface implementations

### Development Support

- **examples/**: Sample applications and use cases
- **tests/**: Comprehensive test suite
- **benches/**: Performance benchmarks
- **docs/**: Technical documentation
- **scripts/**: Build and deployment automation

## Building the Project

```bash
# Clone the repository with submodules
git clone --recursive https://github.com/your-org/robot-system-ng
cd robot-system-ng

# Build the entire workspace
cargo build --release

# Run tests
cargo test

# Run benchmarks
cargo bench

# Build specific crate
cargo build -p kalman-filter

# Run the main application
./target/release/robot-system-ng --config example_robot
```

## Key Rust Features Used

- **Workspace**: Multi-crate project organization
- **Async/Await**: Asynchronous I/O and concurrency with Tokio
- **Traits**: Plugin interfaces and abstractions
- **Generics**: Type-safe, zero-cost abstractions
- **Error Handling**: Comprehensive error propagation with `anyhow`
- **Serialization**: Configuration and IPC with `serde`
- **Real-time**: Precise timing and low-latency control loops
- **FFI**: Integration with C libraries (BLAS/LAPACK)
- **Memory Safety**: Safe concurrent access with `Arc`, `Mutex`, `RwLock`

## Migration Notes from C

The Rust port maintains the same architectural principles as the original C version while leveraging Rust's safety and performance features:

- **Memory Safety**: Eliminates buffer overflows and memory leaks
- **Concurrency**: Safe multi-threading without data races
- **Error Handling**: Explicit error propagation and handling
- **Type Safety**: Compile-time verification of data types
- **Performance**: Zero-cost abstractions and optimizations
- **Ecosystem**: Rich crate ecosystem for common functionality