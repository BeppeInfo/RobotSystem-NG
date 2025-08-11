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

use std::collections::HashMap;

use serde::{Deserialize, Serialize};
use serde_json;

use std::path::Path;
// use std::sync::Arc;
use std::fs::File;
// use std::io::prelude::*;

// use anyhow::{Context, Result};
// use tokio::sync::{Mutex, RwLock};
// use tracing::{debug, info, warn};

use crate::sensor::Sensor;
use crate::motor::Motor;

use crate::control_types::{ControlState, ControlVariables};

use kfilter::{Kalman1M, KalmanFilter, KalmanPredictInput};
use nalgebra::{DMatrix, SMatrix};

enum ControlIndex {POSITION=0, VELOCITY=1, ACCELERATION=2, FORCE=3}

const CONTROL_VARIABLE_NAMES:HashMap<&str,ControlIndex> = HashMap::from(
    [("POSITION",ControlIndex::POSITION), ("VELOCITY",ControlIndex::VELOCITY), ("ACCELERATION",ControlIndex::ACCELERATION), ("FORCE",ControlIndex::FORCE)]
);

fn default_limit() -> f64 {
    return -1.0;
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ActuatorIOConfig {
    variable: String,
    config: String,
    #[serde(default)]
    deviation: f64,
    #[serde(default="default_limit")]
    limit: f64
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ActuatorConfig {
    sensors: Vec<ActuatorIOConfig>,
    motor: ActuatorIOConfig
}

#[derive(Debug)]
pub struct Actuator {
    control_state: ControlState,
    control_mode: ControlIndex,
    sensors: Vec<Sensor>,
    motor: Motor,
    motion_filter: KalmanFilter,
    setpoint_limit: f64
}

impl Actuator {
    pub fn load(root_dir: &Path, config_name: &str, log_dir: &Path) -> Result<Self> {
        // info!("Loading actuator: {}", config_name);

        // Load motor configuration
        let config_path = root_dir.join("config/actuators").join(format!("{}.json", config_name));
        let config_file = File::open(config_path)?;
        let config:ActuatorConfig = serde_json::from_reader(config_file)?;

        // info!("Actuator config loaded: {} sensors, {} motor", config.sensors.len(), config.motor.config);

        const STATES_NUMBER:usize = CONTROL_VARIABLE_NAMES.len();
        let measures_number:usize = config.sensors.len();

        let mut state_transition = SMatrix::<f64,STATES_NUMBER,STATES_NUMBER>::zeros();
        let mut observer = DMatrix::from_element(measures_number, measures_number, 0.0);
        let mut prediction_covariance_noise = DMatrix::from_diagonal_element(measures_number, measures_number, 1.0);
        let mut error_covariance_noise = DMatrix::from_diagonal_element(measures_number, measures_number, 1.0);
        
        let mut measurement = DMatrix::from_element(measures_number, 1, 0.0);

        let mut sensors = Vec::<Sensor>::new();
        for config_sensor in config.sensors {
            if let Some(state_index) = CONTROL_VARIABLE_NAMES.get(config_sensor.variable) {
                sensors.push(Sensor::load(root_dir, config_sensor.config, log_dir).unwrap()?);
                observer[(sensors.len()-1, state_index)] = 1.0;
                error_covariance_noise[(sensors.len()-1, sensors.len()-1)] = config_sensor.deviation * config_sensor.deviation;
            }
        }

        let mut motor = Motor::load(root_dir, config.motor.config, log_dir).unwrap()?;
        let mut control_mode = CONTROL_VARIABLE_NAMES.get(config.motor.variable).unwrap()?;

        let mut filter = Kalman1M::new(
            state_transition, prediction_covariance_noise, observer, error_covariance_noise, measurement
        );

        let mut actuator = Actuator {
            control_state: ControlState::PASSIVE,
            control_mode: control_mode,
            sensors: sensors,
            motor: motor,
            motion_filter: filter,
            setpoint_limit: config.motor.limit
        };

        // info!("Actuator loaded successfully");

        return Ok(actuator);
    }

    fn enable(&self) -> bool {
        return self.motor.enable();
    }

    fn disable(&self) {
        self.motor.disable();
    }

    fn set_control_state(&mut self, new_state:ControlState) -> bool {

        if(new_state == self.control_state) {
            return false;
        }
        
        // self.motion_filter.reset();
        
        // DEBUG_PRINT( "setting actuator state to %s", ( newState == CONTROL_OFFSET ) ? "offset" : ( ( newState == CONTROL_CALIBRATION ) ? "calibration" : "operation" ) );
        match new_state {
            ControlState::OFFSET => {
                self.sensors.iter_mut().for_each(|sensor| sensor.set_offset());
                self.motor.set_offset();
            },
            ControlState::CALIBRATION => {
                self.sensors.iter_mut().for_each(|sensor| sensor.set_calibration());
                self.motor.start_operation();
            },
            _ => {
                self.sensors.iter_mut().for_each(|sensor| sensor.set_measurement());
                self.motor.start_operation();
            }
        }
        
        self.control_state = new_state;
        
        return true;
    }

    fn get_measures(&self, &mut measurements:ControlVariables, time_delta:f64 )
    {       
        //DEBUG_PRINT( "reading measures from %lu sensors", actuator->sensorsNumber );
        
        let &mut state_transition = self.motion_filter.system_mut();
        state_transition[(ControlIndex::POSITION, ControlIndex::VELOCITY)] = time_delta;
        state_transition[(ControlIndex::POSITION, ControlIndex::ACCELERATION)] = time_delta * time_delta / 2.0;
        state_transition[(ControlIndex::VELOCITY, ControlIndex::ACCELERATION)] = time_delta;
        
        let mut readings = Vec::<f64>::new();
        self.sensors.iter_mut().for_each(|sensor| readings.push(sensor.update()));
        
        self.motion_filter.predict();
        self.motion_filter.update(DMatrix::from_vec(self.sensors.len(), 1, readings));
        
        let &state = self.motion_filter.state();
        //DEBUG_PRINT( "p=%.5f, v=%.5f, f=%.5f", filteredMeasures[ POSITION ], filteredMeasures[ VELOCITY ], filteredMeasures[ FORCE ] );
        measurements.position = state[ControlIndex::POSITION];
        measurements.velocity = state[ControlIndex::VELOCITY];
        measurements.acceleration = state[ControlIndex::ACCELERATION];
        measurements.force = state[ControlIndex::FORCE];
        
        // Log_EnterNewLine( actuator->log, Time_GetExecSeconds() );
        // Log_RegisterList( actuator->log, CONTROL_VARS_NUMBER, (double*) filteredMeasures );
    }

    fn set_setpoints(&self, &mut setpoints:ControlVariables ) -> f64 {      
        let motor_setpoint = match self.control_mode {
            ControlIndex::POSITION => setpoints.position,
            ControlIndex::VELOCITY => setpoints.velocity,
            ControlIndex::ACCELERATION => setpoints.acceleration,
            ControlIndex::FORCE => setpoints.force
        };

        if self.setpoint_limit > 0.0 {
            if motor_setpoint < -self.setpoint_limit { 
                motor_setpoint = -self.setpoint_limit;
            } else if motor_setpoint > self.setpoint_limit {
                motor_setpoint = self.setpoint_limit;
            }
        }
        //DEBUG_PRINT( "writing mode %d setpoint %g to motor", actuator->controlMode, motorSetpoint );
        // If the motor is being actually controlled, write its control output
        if( self.control_state != ControlState::OFFSET ) {
            self.motor.set_target( motor_setpoint );
        }
        //DEBUG_PRINT( "setpoint %g written to motor", motorSetpoint );
        return motor_setpoint;
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;
//     use tempfile::TempDir;

//     #[tokio::test]
//     async fn test_actuator_state_transitions() {
//         // Test basic actuator state transitions without actual hardware
//         let temp_dir = TempDir::new().unwrap();

//         // This would require mock configurations and plugins
//         // Implementation depends on actual plugin architecture
//     }

//     #[test]
//     fn test_control_variables_mapping() {
//         let variables = vec!["position".to_string(), "velocity".to_string()];
//         assert_eq!(variables.len(), 2);

//         // Test that we can map measurement values to appropriate control variables
//         let measurement = 1.5;
//         match variables[0].as_str() {
//             "position" => assert_eq!(measurement, 1.5),
//             _ => panic!("Unexpected variable type"),
//         }
//     }
// }
