use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result};
use tokio::sync::{Mutex, RwLock};
use tracing::{debug, info, warn};

use crate::config::ActuatorConfig;
use crate::sensor::{Sensor, SensorId};
use crate::motor::{Motor, MotorId};
use crate::shared_types::{ActuatorState, AxisState, JointState, ControlOutput, ControlVariables};
use kalman_filter::KalmanFilter;
use plugin_loader::PluginLoader;
use data_logging::DataLogger;

pub type ActuatorId = String;

#[derive(Debug)]
pub struct Actuator {
    pub id: ActuatorId,
    pub config: ActuatorConfig,
    pub sensors: HashMap<SensorId, Arc<Sensor>>,
    pub motors: HashMap<MotorId, Arc<Motor>>,
    pub kalman_filter: Arc<Mutex<KalmanFilter>>,
    pub state: Arc<RwLock<ActuatorState>>,
    pub data_logger: Arc<DataLogger>,
    axes_count: usize,
    joints_count: usize,
}

impl Actuator {
    pub async fn load(
        root_dir: &Path,
        actuator_id: &str,
        log_dir: &Path,
        plugin_loader: &mut PluginLoader,
    ) -> Result<Self> {
        info!("Loading actuator: {}", actuator_id);

        // Load actuator configuration
        let config_path = root_dir.join("config/actuators").join(format!("{}.json", actuator_id));
        let config = ActuatorConfig::load(&config_path)
            .context("Failed to load actuator configuration")?;

        info!("Actuator config loaded: {} sensors, {} motors", 
              config.sensor_ids.len(), config.motor_ids.len());

        // Initialize data logger
        let actuator_log_dir = log_dir.join(actuator_id);
        tokio::fs::create_dir_all(&actuator_log_dir).await?;
        let data_logger = Arc::new(DataLogger::new(&actuator_log_dir)?);

        // Load sensors
        let mut sensors = HashMap::new();
        for sensor_id in &config.sensor_ids {
            debug!("Loading sensor: {}", sensor_id);
            let sensor = Arc::new(
                Sensor::load(root_dir, sensor_id, &actuator_log_dir, plugin_loader).await
                    .with_context(|| format!("Failed to load sensor: {}", sensor_id))?
            );
            sensors.insert(sensor_id.clone(), sensor);
        }

        // Load motors
        let mut motors = HashMap::new();
        for motor_id in &config.motor_ids {
            debug!("Loading motor: {}", motor_id);
            let motor = Arc::new(
                Motor::load(root_dir, motor_id, &actuator_log_dir, plugin_loader).await
                    .with_context(|| format!("Failed to load motor: {}", motor_id))?
            );
            motors.insert(motor_id.clone(), motor);
        }

        // Initialize Kalman filter
        let kalman_filter = Arc::new(Mutex::new(
            KalmanFilter::new(
                config.kalman_filter.state_dimension,
                config.kalman_filter.measurement_dimension,
                &config.kalman_filter.process_noise,
                &config.kalman_filter.measurement_noise,
            )?
        ));

        // Determine axes and joints count based on control variables
        let axes_count = config.control_variables.len();
        let joints_count = 1; // Typically 1 joint per actuator, but could be configurable

        // Initialize actuator state
        let initial_state = ActuatorState {
            axes_states: vec![AxisState::default(); axes_count],
            joint_states: vec![JointState::default(); joints_count],
            enabled: false,
            fault: false,
            fault_message: None,
        };

        let actuator = Actuator {
            id: actuator_id.to_string(),
            config,
            sensors,
            motors,
            kalman_filter,
            state: Arc::new(RwLock::new(initial_state)),
            data_logger,
            axes_count,
            joints_count,
        };

        info!("Actuator loaded successfully: {} axes, {} joints", axes_count, joints_count);
        Ok(actuator)
    }

    pub async fn activate(&self) -> Result<()> {
        info!("Activating actuator: {}", self.id);

        // Activate all sensors
        for (id, sensor) in &self.sensors {
            debug!("Activating sensor: {}", id);
            sensor.activate().await
                .with_context(|| format!("Failed to activate sensor: {}", id))?;
        }

        // Activate all motors
        for (id, motor) in &self.motors {
            debug!("Activating motor: {}", id);
            motor.activate().await
                .with_context(|| format!("Failed to activate motor: {}", id))?;
        }

        // Update state
        {
            let mut state = self.state.write().await;
            state.enabled = true;
            state.fault = false;
            state.fault_message = None;
        }

        info!("Actuator activated successfully");
        Ok(())
    }

    pub async fn deactivate(&self) -> Result<()> {
        info!("Deactivating actuator: {}", self.id);

        // Deactivate all motors first (safety)
        for (id, motor) in &self.motors {
            debug!("Deactivating motor: {}", id);
            if let Err(e) = motor.deactivate().await {
                warn!("Failed to deactivate motor {}: {}", id, e);
            }
        }

        // Deactivate sensors
        for (id, sensor) in &self.sensors {
            debug!("Deactivating sensor: {}", id);
            if let Err(e) = sensor.deactivate().await {
                warn!("Failed to deactivate sensor {}: {}", id, e);
            }
        }

        // Update state
        {
            let mut state = self.state.write().await;
            state.enabled = false;
        }

        info!("Actuator deactivated");
        Ok(())
    }

    pub async fn emergency_stop(&self) -> Result<()> {
        warn!("Emergency stop for actuator: {}", self.id);

        // Emergency stop all motors immediately
        for (id, motor) in &self.motors {
            if let Err(e) = motor.emergency_stop().await {
                warn!("Failed to emergency stop motor {}: {}", id, e);
            }
        }

        // Update state
        {
            let mut state = self.state.write().await;
            state.enabled = false;
            state.fault = true;
            state.fault_message = Some("Emergency stop activated".to_string());
        }

        Ok(())
    }

    pub async fn reset_emergency_stop(&self) -> Result<()> {
        info!("Resetting emergency stop for actuator: {}", self.id);

        // Reset emergency stop on all motors
        for (id, motor) in &self.motors {
            motor.reset_emergency_stop().await
                .with_context(|| format!("Failed to reset emergency stop on motor: {}", id))?;
        }

        // Update state
        {
            let mut state = self.state.write().await;
            state.fault = false;
            state.fault_message = None;
        }

        Ok(())
    }

    pub async fn read_state(&self) -> Result<ActuatorState> {
        // Read all sensor values
        let mut sensor_measurements = Vec::new();
        for sensor in self.sensors.values() {
            let measurement = sensor.read().await?;
            sensor_measurements.push(measurement.value);
        }

        // Apply Kalman filtering if enabled
        let filtered_measurements = if self.config.kalman_filter.enabled && !sensor_measurements.is_empty() {
            let mut kf = self.kalman_filter.lock().await;
            kf.predict()?;
            kf.update(&sensor_measurements)?;
            kf.get_state().to_vec()
        } else {
            sensor_measurements
        };

        // Convert measurements to control variables
        let mut axes_states = Vec::with_capacity(self.axes_count);
        for (i, &measurement) in filtered_measurements.iter().enumerate() {
            if i < self.axes_count {
                let variable_name = &self.config.control_variables[i];
                let mut control_vars = ControlVariables::default();
                
                match variable_name.as_str() {
                    "position" => control_vars.position = measurement,
                    "velocity" => control_vars.velocity = measurement,
                    "force" => control_vars.force = measurement,
                    "acceleration" => control_vars.acceleration = measurement,
                    _ => control_vars.position = measurement, // Default to position
                }

                axes_states.push(AxisState {
                    measurements: control_vars,
                    setpoints: ControlVariables::default(), // Will be set by controller
                    errors: ControlVariables::default(),    // Will be computed
                    enabled: true,
                    fault: false,
                    fault_message: None,
                    timestamp: std::time::SystemTime::now(),
                });
            }
        }

        // Compute joint states (simplified: average of axes)
        let mut joint_states = Vec::with_capacity(self.joints_count);
        if !axes_states.is_empty() {
            let avg_position = axes_states.iter().map(|a| a.measurements.position).sum::<f64>() / axes_states.len() as f64;
            let avg_velocity = axes_states.iter().map(|a| a.measurements.velocity).sum::<f64>() / axes_states.len() as f64;
            let avg_force = axes_states.iter().map(|a| a.measurements.force).sum::<f64>() / axes_states.len() as f64;

            joint_states.push(JointState {
                measurements: ControlVariables {
                    position: avg_position,
                    velocity: avg_velocity,
                    force: avg_force,
                    ..ControlVariables::default()
                },
                setpoints: ControlVariables::default(),
                errors: ControlVariables::default(),
                enabled: true,
                fault: false,
                fault_message: None,
                timestamp: std::time::SystemTime::now(),
            });
        }

        let current_state = self.state.read().await;
        let state = ActuatorState {
            axes_states,
            joint_states,
            enabled: current_state.enabled,
            fault: current_state.fault,
            fault_message: current_state.fault_message.clone(),
        };

        // Log the state
        if self.config.logging.enabled {
            self.data_logger.log_actuator_state(&self.id, &state).await?;
        }

        Ok(state)
    }

    pub async fn write_control_outputs(&self, outputs: &[ControlOutput]) -> Result<()> {
        if outputs.len() != self.motors.len() {
            anyhow::bail!("Output count ({}) doesn't match motor count ({})", 
                         outputs.len(), self.motors.len());
        }

        // Send commands to motors
        for (i, (motor_id, motor)) in self.motors.iter().enumerate() {
            if let Some(output) = outputs.get(i) {
                // Determine output value based on control mode
                let command_value = output.position
                    .or(output.velocity)
                    .or(output.force)
                    .unwrap_or(0.0);

                motor.write(command_value).await
                    .with_context(|| format!("Failed to write to motor: {}", motor_id))?;
            }
        }

        Ok(())
    }

    pub fn axes_count(&self) -> usize {
        self.axes_count
    }

    pub fn joints_count(&self) -> usize {
        self.joints_count
    }

    pub async fn run_diagnostics(&self) -> Result<HashMap<String, String>> {
        let mut diagnostics = HashMap::new();

        // Check sensor health
        for (id, sensor) in &self.sensors {
            match sensor.self_test().await {
                Ok(result) => {
                    diagnostics.insert(format!("sensor_{}", id), result);
                }
                Err(e) => {
                    diagnostics.insert(format!("sensor_{}", id), format!("Error: {}", e));
                }
            }
        }

        // Check motor health
        for (id, motor) in &self.motors {
            match motor.self_test().await {
                Ok(result) => {
                    diagnostics.insert(format!("motor_{}", id), result);
                }
                Err(e) => {
                    diagnostics.insert(format!("motor_{}", id), format!("Error: {}", e));
                }
            }
        }

        // Check Kalman filter status
        if self.config.kalman_filter.enabled {
            let kf = self.kalman_filter.lock().await;
            diagnostics.insert("kalman_filter".to_string(), 
                             format!("State dim: {}, Meas dim: {}", 
                                   kf.state_dimension(), kf.measurement_dimension()));
        }

        Ok(diagnostics)
    }

    pub async fn shutdown(&self) -> Result<()> {
        info!("Shutting down actuator: {}", self.id);

        // Shutdown motors first
        for (id, motor) in &self.motors {
            debug!("Shutting down motor: {}", id);
            if let Err(e) = motor.shutdown().await {
                warn!("Failed to shutdown motor {}: {}", id, e);
            }
        }

        // Shutdown sensors
        for (id, sensor) in &self.sensors {
            debug!("Shutting down sensor: {}", id);
            if let Err(e) = sensor.shutdown().await {
                warn!("Failed to shutdown sensor {}: {}", id, e);
            }
        }

        // Shutdown data logging
        self.data_logger.shutdown().await?;

        info!("Actuator shutdown complete");
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[tokio::test]
    async fn test_actuator_state_transitions() {
        // Test basic actuator state transitions without actual hardware
        let temp_dir = TempDir::new().unwrap();
        
        // This would require mock configurations and plugins
        // Implementation depends on actual plugin architecture
    }

    #[test]
    fn test_control_variables_mapping() {
        let variables = vec!["position".to_string(), "velocity".to_string()];
        assert_eq!(variables.len(), 2);
        
        // Test that we can map measurement values to appropriate control variables
        let measurement = 1.5;
        match variables[0].as_str() {
            "position" => assert_eq!(measurement, 1.5),
            _ => panic!("Unexpected variable type"),
        }
    }
}