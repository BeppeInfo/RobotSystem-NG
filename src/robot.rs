use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use tokio::sync::{Mutex, RwLock};
use tracing::{debug, info, warn};

use crate::actuator::{Actuator, ActuatorId};
use crate::config::RobotConfig;
use crate::shared_types::{AxisState, ControlMode, RobotState, JointState};
use data_logging::DataLogger;
use plugin_loader::PluginLoader;
use robot_control_interface::{RobotController, RobotControllerPlugin};

pub type RobotId = String;

#[derive(Debug)]
pub struct Robot {
    pub id: RobotId,
    pub config: RobotConfig,
    pub state: Arc<RwLock<RobotState>>,
    pub actuators: HashMap<ActuatorId, Arc<Actuator>>,
    pub controller: Arc<dyn RobotController>,
    pub data_logger: Arc<DataLogger>,
    pub plugin_loader: PluginLoader,
    axes_count: usize,
    joints_count: usize,
}

impl Robot {
    pub async fn load(
        root_dir: &Path, 
        robot_name: &str, 
        log_dir: &Path
    ) -> Result<Self> {
        info!("Loading robot: {}", robot_name);
        
        // Load robot configuration
        let config_path = root_dir.join("config/robots").join(format!("{}.json", robot_name));
        let config = RobotConfig::load(&config_path)
            .context("Failed to load robot configuration")?;
        
        info!("Robot config loaded: {} actuators", config.actuator_ids.len());

        // Initialize plugin loader
        let plugin_dir = root_dir.join("plugins");
        let mut plugin_loader = PluginLoader::new(&plugin_dir)?;

        // Load robot controller plugin
        let controller = plugin_loader
            .load_robot_controller(&config.control_plugin)
            .context("Failed to load robot controller plugin")?;

        // Initialize data logger
        let robot_log_dir = log_dir.join(&robot_name);
        tokio::fs::create_dir_all(&robot_log_dir).await?;
        let data_logger = Arc::new(DataLogger::new(&robot_log_dir)?);

        // Load actuators
        let mut actuators = HashMap::new();
        let mut total_axes = 0;
        let mut total_joints = 0;

        for actuator_id in &config.actuator_ids {
            info!("Loading actuator: {}", actuator_id);
            
            let actuator = Arc::new(
                Actuator::load(
                    root_dir, 
                    actuator_id, 
                    &robot_log_dir,
                    &mut plugin_loader
                ).await
                .with_context(|| format!("Failed to load actuator: {}", actuator_id))?
            );

            total_axes += actuator.axes_count();
            total_joints += actuator.joints_count();
            
            actuators.insert(actuator_id.clone(), actuator);
        }

        // Initialize robot state
        let initial_state = RobotState {
            control_mode: ControlMode::Idle,
            axes_states: vec![AxisState::default(); total_axes],
            joint_states: vec![JointState::default(); total_joints],
            is_active: false,
            emergency_stop: false,
            error_message: None,
        };

        let robot = Robot {
            id: robot_name.to_string(),
            config,
            state: Arc::new(RwLock::new(initial_state)),
            actuators,
            controller,
            data_logger,
            plugin_loader,
            axes_count: total_axes,
            joints_count: total_joints,
        };

        // Initialize controller with robot parameters
        robot.controller.initialize(total_axes, total_joints).await?;

        info!("Robot loaded successfully: {} axes, {} joints", total_axes, total_joints);
        Ok(robot)
    }

    pub async fn activate(&self) -> Result<()> {
        info!("Activating robot: {}", self.id);

        // Activate all actuators
        for (id, actuator) in &self.actuators {
            debug!("Activating actuator: {}", id);
            actuator.activate().await
                .with_context(|| format!("Failed to activate actuator: {}", id))?;
        }

        // Update robot state
        {
            let mut state = self.state.write().await;
            state.is_active = true;
            state.control_mode = ControlMode::Position;
        }

        // Initialize controller
        self.controller.start().await?;

        info!("Robot activated successfully");
        Ok(())
    }

    pub async fn deactivate(&self) -> Result<()> {
        info!("Deactivating robot: {}", self.id);

        // Stop controller first
        self.controller.stop().await?;

        // Deactivate all actuators
        for (id, actuator) in &self.actuators {
            debug!("Deactivating actuator: {}", id);
            if let Err(e) = actuator.deactivate().await {
                warn!("Failed to deactivate actuator {}: {}", id, e);
            }
        }

        // Update robot state
        {
            let mut state = self.state.write().await;
            state.is_active = false;
            state.control_mode = ControlMode::Idle;
        }

        info!("Robot deactivated");
        Ok(())
    }

    pub async fn emergency_stop(&self) -> Result<()> {
        warn!("Emergency stop activated for robot: {}", self.id);

        // Stop controller immediately
        self.controller.emergency_stop().await?;

        // Emergency stop all actuators
        for (id, actuator) in &self.actuators {
            if let Err(e) = actuator.emergency_stop().await {
                warn!("Failed to emergency stop actuator {}: {}", id, e);
            }
        }

        // Update robot state
        {
            let mut state = self.state.write().await;
            state.emergency_stop = true;
            state.is_active = false;
            state.control_mode = ControlMode::Idle;
        }

        warn!("Emergency stop completed");
        Ok(())
    }

    pub async fn reset_emergency_stop(&self) -> Result<()> {
        info!("Resetting emergency stop for robot: {}", self.id);

        // Reset emergency stop on all actuators
        for (id, actuator) in &self.actuators {
            actuator.reset_emergency_stop().await
                .with_context(|| format!("Failed to reset emergency stop on actuator: {}", id))?;
        }

        // Reset controller
        self.controller.reset().await?;

        // Update robot state
        {
            let mut state = self.state.write().await;
            state.emergency_stop = false;
        }

        info!("Emergency stop reset");
        Ok(())
    }

    pub async fn set_control_mode(&self, mode: ControlMode) -> Result<()> {
        info!("Setting control mode to: {:?}", mode);

        // Validate transition
        let current_state = self.state.read().await;
        if current_state.emergency_stop {
            anyhow::bail!("Cannot change control mode while in emergency stop");
        }

        drop(current_state);

        // Set control mode on controller
        self.controller.set_control_mode(mode).await?;

        // Update robot state
        {
            let mut state = self.state.write().await;
            state.control_mode = mode;
        }

        info!("Control mode set to: {:?}", mode);
        Ok(())
    }

    pub async fn update_control_step(&self) -> Result<()> {
        // Read current states from all actuators
        let mut axes_states = Vec::with_capacity(self.axes_count);
        let mut joint_states = Vec::with_capacity(self.joints_count);

        for actuator in self.actuators.values() {
            let actuator_state = actuator.read_state().await?;
            axes_states.extend(actuator_state.axes_states);
            joint_states.extend(actuator_state.joint_states);
        }

        // Update robot state
        {
            let mut state = self.state.write().await;
            state.axes_states = axes_states.clone();
            state.joint_states = joint_states.clone();
        }

        // Run control algorithm
        let control_outputs = self.controller
            .compute_control(&axes_states, &joint_states)
            .await?;

        // Distribute control outputs to actuators
        let mut output_idx = 0;
        for actuator in self.actuators.values() {
            let actuator_outputs = control_outputs
                .iter()
                .skip(output_idx)
                .take(actuator.axes_count())
                .cloned()
                .collect();
            
            actuator.write_control_outputs(&actuator_outputs).await?;
            output_idx += actuator.axes_count();
        }

        // Log data
        self.data_logger.log_robot_state(&axes_states, &joint_states).await?;

        Ok(())
    }

    pub async fn get_state(&self) -> RobotState {
        self.state.read().await.clone()
    }

    pub fn axes_count(&self) -> usize {
        self.axes_count
    }

    pub fn joints_count(&self) -> usize {
        self.joints_count
    }

    pub async fn shutdown(&self) -> Result<()> {
        info!("Shutting down robot: {}", self.id);

        // Deactivate if still active
        if self.state.read().await.is_active {
            self.deactivate().await?;
        }

        // Shutdown all actuators
        for (id, actuator) in &self.actuators {
            debug!("Shutting down actuator: {}", id);
            if let Err(e) = actuator.shutdown().await {
                warn!("Failed to shutdown actuator {}: {}", id, e);
            }
        }

        // Shutdown data logging
        self.data_logger.shutdown().await?;

        info!("Robot shutdown complete");
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[tokio::test]
    async fn test_robot_state_transitions() {
        // Test basic robot state transitions without actual hardware
        let temp_dir = TempDir::new().unwrap();
        
        // This would require mock configurations and plugins
        // Implementation depends on actual plugin architecture
    }
}