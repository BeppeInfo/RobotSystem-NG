use serde::{Deserialize, Serialize};
use std::time::{Duration, SystemTime};

/// Defined possible control states enumeration. Passed to generic or plugin specific robot control implementations
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ControlState 
{ 
  PASSIVE=0,            ///< State for fully compliant robot control/behaviour
  OFFSET=1,             ///< State for definition of reference (zero) for controller measurements 
  CALIBRATION=2,        ///< State for definition of limits (min-max) for controller measurements 
  PREPROCESSING=3,      ///< State for custom automatic preprocessing of controller parameters 
  OPERATION=4,          ///< State for normal controller operation 
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
/// Control used variables list indexes enumeration
pub struct ControlVariables {
    position: f64,
    velocity: f64,
    acceleration: f64,
    force: f64,
    stiffness: f64,
    damping: f64,
    inertia: f64
}

impl Default for ControlVariables {
    fn default() -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            acceleration: 0.0,
            force: 0.0,
            stiffness: 1000.0, // Default stiffness
            damping: 10.0,     // Default damping
            inertia: 1.0,      // Default inertia
        }
    }
}