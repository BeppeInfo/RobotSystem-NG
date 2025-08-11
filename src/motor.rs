////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2025 Leonardo Consoni                                       //
//                                                                            //
//  This file is part of RobotSystem-NG.                                      //
//                                                                            //
//  RobotSystem-NG is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobotSystem-NG is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobotSystem-NG. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

use serde::{Deserialize, Serialize};
use serde_json;

use std::path::Path;
// use std::sync::Arc;
use std::fs::File;
// use std::io::prelude::*;

// use anyhow::{Context, Result};
use evalexpr::{build_operator_tree, EvalexprResult, Node, DefaultNumericTypes, HashMapContext, Value};
// use tokio::sync::{Mutex, RwLock};

use crate::input::{Input,InputConfig};
// use tracing::{debug, info, warn};

#[derive(Serialize, Deserialize, Debug)]
pub struct OutputSinkConfig {
    implementation:String,
    #[serde(default)]
    config:String,
    #[serde(default)]
    channel:u32
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MotorConfig {
    intertace: OutputSinkConfig,
    reference: Option<InputConfig>,
    output: String
}

#[derive(Debug)]
pub struct Motor {
    //plugin
    device_id:i32,
    channel:u32,
    reference: Option<Input>,
    input_context: HashMapContext<DefaultNumericTypes>,
    transform_function: EvalexprResult<Node<DefaultNumericTypes>, DefaultNumericTypes>,
    is_offsetting: bool,
}

impl Motor {
    pub fn load(root_dir: &Path, config_name: &str, log_dir: &Path) -> Result<Self> {
        // info!("Loading motor: {}", config_name);

        // Load motor configuration
        let config_path = root_dir.join("config/motors").join(format!("{}.json", config_name));
        let config_file = File::open(config_path)?;
        let config:MotorConfig = serde_json::from_reader(config_file)?;

        // info!("Motor config loaded: {}", config_name);

        let mut reference_input:Option<Input> = None;
        if let Some(config_input) = config.reference {
            reference_input = Some(Input::load(config_input).unwrap()?);
        }

        let mut context = HashMapContext::<DefaultNumericTypes>::new();
        context.set_value("set".into(), Value::from_float(0.0)).unwrap()?;
        context.set_value("ref".into(), Value::from_float(0.0)).unwrap()?;

        let precompiled_expr = build_operator_tree::<DefaultNumericTypes>(config.output);

        let mut motor = Motor {
            device_id: 1,
            channel: config.interface.channel,
            reference: reference_input,
            input_context: context,
            transform_function: precompiled_expr,
            is_offsetting: false
        };

        // info!("Motor loaded successfully");
        return Ok(motor);
    }

    pub fn enable(&self) -> bool {
        return true;
    }

    pub fn disable(&self) {

    }

    pub fn set_offset(&self) {
        if let Some(reference_input) = self.reference {
            match self.input_context.set_value("ref".into(), Value::from_float(0.0)) {
                Ok(setpoint) => println!("Offset start"),
                Err(msg) => eprintln!("{}", msg)
            }
        }

        self.is_offsetting = true;
    }

    pub fn start_operation(&self) {
        if self.is_offsetting {
            if let Some(reference_input) = self.reference {
                let offset = Value::from_float(reference_input.update());
                match self.input_context.set_value("ref".into(), offset) {
                    Ok(setpoint) => println!("Offset={}", offset),
                    Err(msg) => eprintln!("{}", msg)
                }
            }
        }
  
        self.is_offsetting = false;
    }

    pub fn set_target(&self, target:f64) {
        self.input_context.set_value("set".into(), Value::from_float(target)).unwrap()?;

        if !self.is_offsetting {
            match self.transform_function.eval_with_context(&self.input_context) {
                Ok(setpoint) => println!("Setpoint={}", setpoint),
                Err(msg) => eprintln!("{}", msg)
            }
        }
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;
//     use tempfile::TempDir;

//     #[tokio::test]
//     async fn test_motor_command_creation() {
//         let motor_command = MotorCommand {
//             value: 5.0,
//             timestamp: std::time::SystemTime::now(),
//             motor_id: "test_motor".to_string(),
//         };

//         assert_eq!(motor_command.value, 5.0);
//         assert_eq!(motor_command.motor_id, "test_motor");
//     }

//     #[test]
//     fn test_limits_application() {
//         use crate::config::MotorLimits;

//         let limits = MotorLimits {
//             min_value: -10.0,
//             max_value: 10.0,
//             max_rate: 5.0,
//             max_acceleration: 2.0,
//         };

//         // Test value clamping
//         let test_value = 15.0;
//         let clamped = test_value.clamp(limits.min_value, limits.max_value);
//         assert_eq!(clamped, 10.0);

//         let test_value = -15.0;
//         let clamped = test_value.clamp(limits.min_value, limits.max_value);
//         assert_eq!(clamped, -10.0);
//     }

//     #[test]
//     fn test_conversion_expression() {
//         use evalexpr::*;

//         let mut context = HashMapContext::new();
//         context
//             .set_value("command".to_string(), 5.0.into())
//             .unwrap();
//         context
//             .set_value("reference".to_string(), 2.0.into())
//             .unwrap();

//         // Test a simple conversion expression
//         let result = eval_with_context("command * 2 + reference", &context).unwrap();
//         assert_eq!(result.as_float().unwrap(), 12.0);
//     }

//     #[tokio::test]
//     async fn test_motor_state_transitions() {
//         // Test basic motor state transitions
//         // This would require mock plugins for full testing
//         let enabled = Arc::new(RwLock::new(false));
//         let fault = Arc::new(RwLock::new(false));
//         let emergency_stopped = Arc::new(RwLock::new(false));

//         // Test initial state
//         assert!(!*enabled.read().await);
//         assert!(!*fault.read().await);
//         assert!(!*emergency_stopped.read().await);

//         // Test activation
//         {
//             let mut enabled_guard = enabled.write().await;
//             *enabled_guard = true;
//         }
//         assert!(*enabled.read().await);

//         // Test emergency stop
//         {
//             let mut emergency_guard = emergency_stopped.write().await;
//             *emergency_guard = true;
//             let mut enabled_guard = enabled.write().await;
//             *enabled_guard = false;
//             let mut fault_guard = fault.write().await;
//             *fault_guard = true;
//         }
//         assert!(!*enabled.read().await);
//         assert!(*fault.read().await);
//         assert!(*emergency_stopped.read().await);
//     }
// }
