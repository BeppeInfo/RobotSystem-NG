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

use crate::input::{Input, InputConfig, InputState};
// use tracing::{debug, info, warn};

#[derive(Serialize, Deserialize, Debug)]
pub struct SensorConfig {
    inputs: Vec<InputConfig>,
    output: String
}

#[derive(Debug)]
pub struct Sensor {
    inputs: Vec<Input>,
    input_context: HashMapContext<DefaultNumericTypes>,
    transform_function: EvalexprResult<Node<DefaultNumericTypes>, DefaultNumericTypes>,
}

impl Sensor {
    pub fn load(root_dir: &Path, config_name: &str, log_dir: &Path) -> Result<Self> {
        // info!("Loading sensor: {}", config_name);

        // Load motor configuration
        let config_path = root_dir.join("config/sensors").join(format!("{}.json", config_name));
        let config_file = File::open(config_path)?;
        let config:SensorConfig = serde_json::from_reader(config_file)?;

        // info!("Sensor config loaded: {}", config_name);

        let mut context = HashMapContext::<DefaultNumericTypes>::new();

        let mut inputs = Vec::<Input>::new();
        for config_input in config.inputs {
            inputs.push(Input::load(config_input).unwrap()?);
            context.set_value( ("in" + String::from(inputs.len()-1)).into(), Value::from_float(0.0)).unwrap()?;
        }

        let precompiled_expr = build_operator_tree::<DefaultNumericTypes>(config.output);

        let mut sensor = Sensor {
            inputs: inputs,
            input_context: context,
            transform_function: precompiled_expr
        };

        // info!("Sensor loaded successfully");
        return Ok(sensor);
    }

    pub fn update(&self) -> f64 {
        for input_index in 0..self.inputs.len() {
            let reading = self.inputs[input_index].update();
            if let Err(msg) = self.input_context.set_value( ("in" + String::from(input_index)).into(), Value::from_float(reading)) {
                eprintln!("{}", msg);
            }
        }

        match self.transform_function.eval_with_context(&self.input_context) {
            Ok(result) => return result,
            Err(msg) => return 0.0
        }
    }

    fn set_state(&self, value:InputState) {
        for input in self.inputs {
            input.set_state(value);
        }
    }

    pub fn set_offset(&self) {
        self.set_state(InputState::OFFSET);
    }

    pub fn set_calibration(&self) {
        self.set_state(InputState::CALIBRATION);
    }

    pub fn set_measurement(&self) {
        self.set_state(InputState::MEASUREMENT);
    }

}

// #[cfg(test)]
// mod tests {
//     use super::*;
//     use tempfile::TempDir;

//     #[tokio::test]
//     async fn test_sensor_data_creation() {
//         let sensor_data = SensorData {
//             value: 3.14,
//             timestamp: std::time::SystemTime::now(),
//             valid: true,
//             sensor_id: "test_sensor".to_string(),
//         };

//         assert_eq!(sensor_data.value, 3.14);
//         assert!(sensor_data.valid);
//         assert_eq!(sensor_data.sensor_id, "test_sensor");
//     }

//     #[test]
//     fn test_expression_evaluation() {
//         use evalexpr::*;

//         let mut context = HashMapContext::new();
//         context
//             .set_value("input1".to_string(), 10.0.into())
//             .unwrap();
//         context.set_value("input2".to_string(), 5.0.into()).unwrap();

//         let result = eval_with_context("input1 * 2 + input2", &context).unwrap();
//         assert_eq!(result.as_float().unwrap(), 25.0);
//     }

//     #[tokio::test]
//     async fn test_sensor_statistics() {
//         // Test statistics calculation with mock data
//         let samples = vec![1.0, 2.0, 3.0, 4.0, 5.0];
//         let mean = samples.iter().sum::<f64>() / samples.len() as f64;
//         assert_eq!(mean, 3.0);

//         let variance =
//             samples.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / samples.len() as f64;
//         assert_eq!(variance, 2.0);
//     }
// }
