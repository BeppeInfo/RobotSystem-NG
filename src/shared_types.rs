use serde::{Deserialize, Serialize};
use std::time::{Duration, SystemTime};

/// Control modes for the robot system
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ControlMode {
    Idle,
    Position,
    Velocity, 
    Force,
    Impedance,
    Admittance,
    Custom(u8),
}

impl Default for ControlMode {
    fn default() -> Self {
        ControlMode::Idle
    }
}

/// Control variables for robot axes/joints following unit conventions:
/// - Position: meter (m) for translation, radian (rad) for rotation
/// - Velocity: m/s for translation, rad/s for rotation  
/// - Force/Torque: Newton (N) for translation, N⋅m for rotation
/// - Acceleration: m/s² for translation, rad/s² for rotation
/// - Stiffness: N/m for translation, N⋅m/rad for rotation
/// - Damping: N⋅s/m for translation, N⋅m⋅s/rad for rotation
/// - Inertia: kg (N⋅s²/m) for translation, kg⋅m² (N⋅m⋅s²/rad) for rotation
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ControlVariables {
    pub position: f64,
    pub velocity: f64,
    pub acceleration: f64,
    pub force: f64,
    pub stiffness: f64,
    pub damping: f64,
    pub inertia: f64,
}

impl Default for ControlVariables {
    fn default() -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            acceleration: 0.0,
            force: 0.0,
            stiffness: 1000.0,  // Default stiffness
            damping: 10.0,      // Default damping
            inertia: 1.0,       // Default inertia
        }
    }
}

/// State of a single robot axis (degrees of freedom)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AxisState {
    pub measurements: ControlVariables,
    pub setpoints: ControlVariables,
    pub errors: ControlVariables,
    pub enabled: bool,
    pub fault: bool,
    pub fault_message: Option<String>,
    pub timestamp: SystemTime,
}

impl Default for AxisState {
    fn default() -> Self {
        Self {
            measurements: ControlVariables::default(),
            setpoints: ControlVariables::default(),
            errors: ControlVariables::default(),
            enabled: false,
            fault: false,
            fault_message: None,
            timestamp: SystemTime::now(),
        }
    }
}

/// State of a robot joint (may contain multiple axes)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointState {
    pub measurements: ControlVariables,
    pub setpoints: ControlVariables,
    pub errors: ControlVariables,
    pub enabled: bool,
    pub fault: bool,
    pub fault_message: Option<String>,
    pub timestamp: SystemTime,
}

impl Default for JointState {
    fn default() -> Self {
        Self {
            measurements: ControlVariables::default(),
            setpoints: ControlVariables::default(),
            errors: ControlVariables::default(),
            enabled: false,
            fault: false,
            fault_message: None,
            timestamp: SystemTime::now(),
        }
    }
}

/// Overall robot state
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RobotState {
    pub control_mode: ControlMode,
    pub axes_states: Vec<AxisState>,
    pub joint_states: Vec<JointState>,
    pub is_active: bool,
    pub emergency_stop: bool,
    pub error_message: Option<String>,
}

impl Default for RobotState {
    fn default() -> Self {
        Self {
            control_mode: ControlMode::default(),
            axes_states: Vec::new(),
            joint_states: Vec::new(),
            is_active: false,
            emergency_stop: false,
            error_message: None,
        }
    }
}

/// Sensor measurement data
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SensorData {
    pub value: f64,
    pub timestamp: SystemTime,
    pub valid: bool,
    pub sensor_id: String,
}

/// Motor command data
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MotorCommand {
    pub value: f64,
    pub timestamp: SystemTime,
    pub motor_id: String,
}

/// Actuator state combining multiple sensors and motors
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ActuatorState {
    pub axes_states: Vec<AxisState>,
    pub joint_states: Vec<JointState>,
    pub enabled: bool,
    pub fault: bool,
    pub fault_message: Option<String>,
}

/// IPC message types for client communication
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum IpcMessage {
    // Robot control commands
    Activate,
    Deactivate,
    EmergencyStop,
    ResetEmergencyStop,
    SetControlMode(ControlMode),
    
    // State queries
    GetState,
    GetAxisCount,
    GetJointCount,
    
    // Configuration
    LoadRobot(String),
    UnloadRobot,
    
    // Data streaming
    AxisUpdate(Vec<ControlVariables>),
    JointUpdate(Vec<ControlVariables>),
    
    // Responses
    Response(IpcResponse),
    Error(String),
}

/// IPC response types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum IpcResponse {
    Ok,
    State(RobotState),
    Count(usize),
    Error(String),
}

/// Fixed-size message buffer for IPC (512 bytes as per original system)
pub const IPC_MESSAGE_SIZE: usize = 512;

#[derive(Debug, Clone)]
pub struct IpcMessageBuffer {
    pub data: [u8; IPC_MESSAGE_SIZE],
    pub length: usize,
}

impl IpcMessageBuffer {
    pub fn new() -> Self {
        Self {
            data: [0u8; IPC_MESSAGE_SIZE],
            length: 0,
        }
    }

    pub fn from_message(message: &IpcMessage) -> anyhow::Result<Self> {
        let serialized = bincode::serialize(message)?;
        if serialized.len() > IPC_MESSAGE_SIZE {
            anyhow::bail!("Message too large: {} bytes", serialized.len());
        }

        let mut buffer = Self::new();
        buffer.data[..serialized.len()].copy_from_slice(&serialized);
        buffer.length = serialized.len();
        Ok(buffer)
    }

    pub fn to_message(&self) -> anyhow::Result<IpcMessage> {
        let data = &self.data[..self.length];
        Ok(bincode::deserialize(data)?)
    }
}

/// Control output from robot controller
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlOutput {
    pub position: Option<f64>,
    pub velocity: Option<f64>,
    pub force: Option<f64>,
    pub stiffness: Option<f64>,
    pub damping: Option<f64>,
}

impl Default for ControlOutput {
    fn default() -> Self {
        Self {
            position: None,
            velocity: None,
            force: None,
            stiffness: None,
            damping: None,
        }
    }
}

/// Logging configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoggingConfig {
    pub enabled: bool,
    pub sample_rate_hz: f64,
    pub log_measurements: bool,
    pub log_setpoints: bool,
    pub log_errors: bool,
    pub log_control_outputs: bool,
}

impl Default for LoggingConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            sample_rate_hz: 1000.0,
            log_measurements: true,
            log_setpoints: true,
            log_errors: true,
            log_control_outputs: true,
        }
    }
}

/// Real-time performance metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceMetrics {
    pub control_loop_frequency: f64,
    pub control_loop_jitter: Duration,
    pub max_latency: Duration,
    pub missed_deadlines: u64,
    pub cpu_usage: f32,
}

impl Default for PerformanceMetrics {
    fn default() -> Self {
        Self {
            control_loop_frequency: 0.0,
            control_loop_jitter: Duration::ZERO,
            max_latency: Duration::ZERO,
            missed_deadlines: 0,
            cpu_usage: 0.0,
        }
    }
}