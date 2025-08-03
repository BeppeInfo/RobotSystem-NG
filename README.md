# RobotSystem-NG

**A Rust port of RobotSystem-Lite - Robotic real-time control application for rehabilitation purposes**

RobotSystem-NG is a comprehensive robotic control system written in Rust, providing a safe, performant, and maintainable alternative to the original C-based RobotSystem-Lite. It maintains the same architectural principles while leveraging Rust's memory safety, concurrency features, and rich ecosystem.

## Features

- **Real-time Control**: High-precision control loops with microsecond timing accuracy
- **Plugin Architecture**: Modular design with dynamic plugin loading for controllers and I/O
- **Multi-layer Configuration**: Flexible JSON-based configuration system
- **IPC Communication**: Multiple communication protocols (ZMQ, TCP, shared memory)
- **Sensor Fusion**: Kalman filtering for multi-sensor state estimation  
- **Data Logging**: Comprehensive logging with multiple output formats
- **Safety Systems**: Emergency stop, fault detection, and safety interlocks
- **Memory Safety**: Rust's ownership system prevents common C pitfalls
- **Concurrent**: Safe multi-threading with actor-based design

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Client Apps   │◄──►│   IPC Server     │◄──►│  Control Loop   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                │                        │
                       ┌────────▼────────┐    ┌─────────▼─────────┐
                       │     Robot       │◄──►│   Actuators      │
                       └─────────────────┘    └───────────────────┘
                                                       │
                              ┌────────────────────────┼────────────────────────┐
                              │                        │                        │
                       ┌──────▼──────┐      ┌─────────▼────────┐      ┌────────▼────────┐
                       │   Sensors   │      │  Kalman Filter   │      │     Motors      │
                       └─────────────┘      └──────────────────┘      └─────────────────┘
                              │                        │                        │
                       ┌──────▼──────┐      ┌─────────▼────────┐      ┌────────▼────────┐
                       │ Signal I/O  │      │  Data Logging    │      │  Signal I/O     │
                       │  Plugins    │      └──────────────────┘      │   Plugins       │
                       └─────────────┘                                └─────────────────┘
```

## Quick Start

### Prerequisites

- Rust 1.70+ with Cargo
- CMake 3.16+ (for C library dependencies)
- BLAS/LAPACK implementation (OpenBLAS, Intel MKL, or ATLAS)
- ZeroMQ library (optional, for ZMQ transport)

### Installation

```bash
# Clone the repository
git clone https://github.com/your-org/robot-system-ng
cd robot-system-ng

# Initialize submodules (if using git submodules for plugins)
git submodule update --init --recursive

# Build the project
cargo build --release

# Run tests
cargo test

# Install system-wide (optional)
cargo install --path .
```

### Basic Usage

```bash
# Start with example robot configuration
./target/release/robot-system-ng --config example_robot

# Specify custom paths
./target/release/robot-system-ng \
    --root ./my_configs \
    --config my_robot \
    --log ./my_logs \
    --addr 192.168.1.100
```

### Configuration

Create robot configuration in `config/robots/my_robot.json`:

```json
{
  "name": "my_robot",
  "description": "Example rehabilitation robot",
  "control_plugin": "impedance_controller",
  "actuator_ids": ["shoulder", "elbow"],
  "default_control_mode": "Position",
  "max_velocity": [1.0, 1.0],
  "max_acceleration": [2.0, 2.0],
  "max_force": [100.0, 50.0],
  "logging": {
    "enabled": true,
    "sample_rate_hz": 1000.0,
    "log_measurements": true,
    "log_setpoints": true,
    "log_errors": true
  }
}
```

## Development

### Workspace Structure

- **src/**: Main application code
- **crates/**: Workspace crates for different subsystems
- **config/**: JSON configuration files
- **plugins/**: Dynamic plugin libraries
- **examples/**: Example applications and demos
- **tests/**: Integration tests
- **docs/**: Technical documentation

### Building Plugins

Plugins are implemented as dynamic libraries using the provided interfaces:

```rust
// Example controller plugin
use robot_control_interface::{RobotController, ControllerResult};
use async_trait::async_trait;

pub struct MyController {
    // Controller state
}

#[async_trait]
impl RobotController for MyController {
    async fn compute_control(
        &self,
        axes_states: &[AxisState],
        joint_states: &[JointState],
    ) -> ControllerResult<Vec<ControlOutput>> {
        // Implement control algorithm
        todo!()
    }
    
    // Other required methods...
}

// Plugin entry point
#[no_mangle]
pub extern "C" fn create_controller() -> Box<dyn RobotController> {
    Box::new(MyController::new())
}
```

### Testing

```bash
# Run all tests
cargo test

# Run specific test suite
cargo test --package kalman-filter

# Run integration tests
cargo test --test robot_control_tests

# Run benchmarks
cargo bench
```

## IPC Communication

### Request-Reply Protocol

For state changes and queries:

```python
import zmq

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

# Activate robot
socket.send_json({"Activate": None})
response = socket.recv_json()
print(f"Response: {response}")

# Get robot state
socket.send_json({"GetState": None})
state = socket.recv_json()
print(f"Robot state: {state}")
```

### Publisher-Subscriber Protocol

For real-time data streaming:

```python
import zmq

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5556")
socket.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to all

while True:
    # Receive real-time robot state updates
    state_update = socket.recv_json()
    print(f"State update: {state_update}")
```

## Plugin Development

### Signal I/O Plugin

```rust
use signal_io_interface::{SignalInput, SignalOutput};
use async_trait::async_trait;

pub struct MyIOPlugin {
    // Hardware interface state
}

#[async_trait]
impl SignalInput for MyIOPlugin {
    async fn read(&self, channel: &str) -> Result<f64> {
        // Read from hardware
        todo!()
    }
    
    async fn activate(&self, channel: &str) -> Result<()> {
        // Initialize hardware channel
        todo!()
    }
    
    // Other methods...
}

#[async_trait]
impl SignalOutput for MyIOPlugin {
    async fn write(&self, channel: &str, value: f64) -> Result<()> {
        // Write to hardware
        todo!()
    }
    
    // Other methods...
}
```

## Safety Features

- **Emergency Stop**: Immediate motor shutdown with fault state
- **Workspace Limits**: Software limits for position, velocity, and force
- **Rate Limiting**: Prevents excessive accelerations
- **Fault Detection**: Comprehensive error checking and reporting
- **Watchdog Timers**: Communication and control loop monitoring

## Performance

- **Control Loop**: Up to 10kHz deterministic control rates
- **Latency**: Sub-millisecond response times
- **Memory**: Zero-allocation control paths after initialization
- **CPU Usage**: Optimized for real-time performance
- **Scalability**: Supports multiple robots and complex configurations

## Migration from RobotSystem-Lite

Key differences from the original C version:

- **Memory Safety**: No buffer overflows or memory leaks
- **Error Handling**: Explicit error propagation with `Result<T, E>`
- **Concurrency**: Safe multi-threading with compile-time checks
- **Package Management**: Cargo handles dependencies
- **Testing**: Built-in testing framework with CI/CD integration
- **Documentation**: Integrated documentation with `cargo doc`

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Make changes and add tests
4. Run tests: `cargo test`
5. Check formatting: `cargo fmt`
6. Run linter: `cargo clippy`
7. Commit changes: `git commit -am 'Add my feature'`
8. Push branch: `git push origin feature/my-feature`
9. Create Pull Request

## License

This project is licensed under the MIT OR Apache-2.0 license - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Original RobotSystem-Lite by AeroTechLab
- Rust community for excellent libraries and tooling
- Contributors to the robotics and rehabilitation research communities

## Support

- **Documentation**: [Online Documentation](https://your-org.github.io/robot-system-ng)
- **Issues**: [GitHub Issues](https://github.com/your-org/robot-system-ng/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-org/robot-system-ng/discussions)
- **Email**: support@your-org.com

---

**RobotSystem-NG** - Safe, fast, and reliable robotic control in Rust 🦀