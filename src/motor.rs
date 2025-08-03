use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result};
use tokio::sync::{Mutex, RwLock};
use tracing::{debug, info, warn};
use evalexpr::{eval, ContextWithMutableVariables, HashMapContext};

use crate::config::{MotorConfig, InputSourceConfig};
use crate::shared_types::{MotorCommand, LoggingConfig};
use signal_io_interface::{SignalOutput, SignalOutputPlugin, SignalInput};
use plugin_loader::PluginLoader;
use data_logging::DataLogger;

pub type MotorId = String;

#[derive(Debug)]
pub struct ReferenceInput {
    pub plugin: Arc<dyn SignalInput>,
    pub channel: String,
    pub gain: f64,
    pub offset: f64,
}

impl ReferenceInput {
    pub async fn read(&self) -> Result<f64> {
        let raw_value = self.plugin.read(&self.channel).await?;
        Ok(raw_value * self.gain + self.offset)
    }
}

#[derive(Debug)]
pub struct Motor {
    pub id: MotorId,
    pub config: MotorConfig,
    pub output_plugin: Arc<dyn SignalOutput>,
    pub reference_input: Option<ReferenceInput>,
    pub expression_context: Arc<Mutex<HashMapContext>>,
    pub current_command: Arc<RwLock<MotorCommand>>,
    pub data_logger: Option<Arc<DataLogger>>,
    pub enabled: Arc<RwLock<bool>>,
    pub fault: Arc<RwLock<bool>>,
    pub emergency_stopped: Arc<RwLock<bool>>,
}

impl Motor {
    pub async fn load(
        root_dir: &Path,
        motor_id: &str,
        log_dir: &Path,
        plugin_loader: &mut PluginLoader,
    ) -> Result<Self> {
        info!("Loading motor: {}", motor_id);

        // Load motor configuration
        let config_path = root_dir.join("config/motors").join(format!("{}.json", motor_id));
        let config = MotorConfig::load(&config_path)
            .context("Failed to load motor configuration")?;

        info!("Motor config loaded: output plugin = {}", config.output_plugin);

        // Load output plugin
        let output_plugin = plugin_loader
            .load_signal_output(&config.output_plugin)
            .context("Failed to load signal output plugin")?;

        // Initialize data logger if enabled
        let data_logger = if config.logging.enabled {
            let motor_log_dir = log_dir.join(motor_id);
            tokio::fs::create_dir_all(&motor_log_dir).await?;
            Some(Arc::new(DataLogger::new(&motor_log_dir)?))
        } else {
            None
        };

        // Load reference input if configured
        let reference_input = if let Some(ref ref_config) = config.reference_input {
            debug!("Loading reference input: {}", ref_config.id);
            
            let input_plugin = plugin_loader
                .load_signal_input(&ref_config.plugin)
                .context("Failed to load reference input plugin")?;

            Some(ReferenceInput {
                plugin: input_plugin,
                channel: ref_config.channel.clone(),
                gain: ref_config.gain,
                offset: ref_config.offset,
            })
        } else {
            None
        };

        // Initialize expression context for conversion
        let expression_context = Arc::new(Mutex::new(HashMapContext::new()));

        // Initialize current command
        let initial_command = MotorCommand {
            value: 0.0,
            timestamp: std::time::SystemTime::now(),
            motor_id: motor_id.to_string(),
        };

        let motor = Motor {
            id: motor_id.to_string(),
            config,
            output_plugin,
            reference_input,
            expression_context,
            current_command: Arc::new(RwLock::new(initial_command)),
            data_logger,
            enabled: Arc::new(RwLock::new(false)),
            fault: Arc::new(RwLock::new(false)),
            emergency_stopped: Arc::new(RwLock::new(false)),
        };

        info!("Motor loaded successfully");
        Ok(motor)
    }

    pub async fn activate(&self) -> Result<()> {
        info!("Activating motor: {}", self.id);

        // Check if emergency stopped
        if *self.emergency_stopped.read().await {
            anyhow::bail!("Cannot activate motor while in emergency stop state");
        }

        // Activate output plugin
        self.output_plugin.activate(&self.config.output_channel).await
            .context("Failed to activate output plugin")?;

        // Activate reference input if present
        if let Some(ref reference) = self.reference_input {
            reference.plugin.activate(&reference.channel).await
                .context("Failed to activate reference input")?;
        }

        // Set initial safe output
        self.output_plugin.write(&self.config.output_channel, 0.0).await
            .context("Failed to set initial output")?;

        // Update state
        {
            let mut enabled = self.enabled.write().await;
            *enabled = true;
        }
        {
            let mut fault = self.fault.write().await;
            *fault = false;
        }

        info!("Motor activated successfully");
        Ok(())
    }

    pub async fn deactivate(&self) -> Result<()> {
        info!("Deactivating motor: {}", self.id);

        // Set output to zero before deactivating
        if let Err(e) = self.output_plugin.write(&self.config.output_channel, 0.0).await {
            warn!("Failed to zero output during deactivation: {}", e);
        }

        // Deactivate output plugin
        if let Err(e) = self.output_plugin.deactivate(&self.config.output_channel).await {
            warn!("Failed to deactivate output plugin: {}", e);
        }

        // Deactivate reference input if present
        if let Some(ref reference) = self.reference_input {
            if let Err(e) = reference.plugin.deactivate(&reference.channel).await {
                warn!("Failed to deactivate reference input: {}", e);
            }
        }

        // Update state
        {
            let mut enabled = self.enabled.write().await;
            *enabled = false;
        }

        info!("Motor deactivated");
        Ok(())
    }

    pub async fn emergency_stop(&self) -> Result<()> {
        warn!("Emergency stop for motor: {}", self.id);

        // Immediately set output to zero
        if let Err(e) = self.output_plugin.write(&self.config.output_channel, 0.0).await {
            warn!("Failed to zero output during emergency stop: {}", e);
        }

        // Update state
        {
            let mut emergency_stopped = self.emergency_stopped.write().await;
            *emergency_stopped = true;
        }
        {
            let mut enabled = self.enabled.write().await;
            *enabled = false;
        }
        {
            let mut fault = self.fault.write().await;
            *fault = true;
        }

        warn!("Emergency stop completed for motor: {}", self.id);
        Ok(())
    }

    pub async fn reset_emergency_stop(&self) -> Result<()> {
        info!("Resetting emergency stop for motor: {}", self.id);

        // Clear emergency stop state
        {
            let mut emergency_stopped = self.emergency_stopped.write().await;
            *emergency_stopped = false;
        }
        {
            let mut fault = self.fault.write().await;
            *fault = false;
        }

        info!("Emergency stop reset for motor: {}", self.id);
        Ok(())
    }

    pub async fn write(&self, command_value: f64) -> Result<()> {
        // Check if motor is enabled and not in fault state
        if !*self.enabled.read().await {
            anyhow::bail!("Motor is not enabled");
        }
        
        if *self.fault.read().await {
            anyhow::bail!("Motor is in fault state");
        }

        if *self.emergency_stopped.read().await {
            anyhow::bail!("Motor is emergency stopped");
        }

        // Apply limits
        let limited_value = self.apply_limits(command_value).await?;

        // Get reference input if available
        let reference_value = if let Some(ref reference) = self.reference_input {
            reference.read().await.unwrap_or(0.0)
        } else {
            0.0
        };

        // Evaluate conversion expression
        let output_value = if !self.config.conversion_expression.is_empty() {
            // Update expression context
            {
                let mut context = self.expression_context.lock().await;
                context.set_value("command".to_string(), limited_value.into())?;
                context.set_value("reference".to_string(), reference_value.into())?;
            }

            // Evaluate conversion expression
            let context = self.expression_context.lock().await;
            let result = eval(&self.config.conversion_expression)
                .context("Failed to evaluate conversion expression")?;
            
            result.as_float()
                .context("Conversion expression result is not a number")?
        } else {
            limited_value
        };

        // Write to output plugin
        self.output_plugin.write(&self.config.output_channel, output_value).await
            .context("Failed to write to output plugin")?;

        // Update current command
        let command = MotorCommand {
            value: command_value,
            timestamp: std::time::SystemTime::now(),
            motor_id: self.id.clone(),
        };

        {
            let mut current = self.current_command.write().await;
            *current = command.clone();
        }

        // Log command if enabled
        if let Some(ref logger) = self.data_logger {
            logger.log_motor_command(&command).await?;
        }

        Ok(())
    }

    async fn apply_limits(&self, value: f64) -> Result<f64> {
        let limits = &self.config.limits;
        
        // Apply value limits
        let mut limited_value = value.clamp(limits.min_value, limits.max_value);

        // Apply rate limiting
        let current_command = self.current_command.read().await;
        let time_diff = std::time::SystemTime::now()
            .duration_since(current_command.timestamp)
            .unwrap_or_default()
            .as_secs_f64();

        if time_diff > 0.0 {
            let max_change = limits.max_rate * time_diff;
            let actual_change = limited_value - current_command.value;
            
            if actual_change.abs() > max_change {
                limited_value = current_command.value + actual_change.signum() * max_change;
            }
        }

        // Apply acceleration limits
        // This would require storing velocity history for proper implementation
        // For now, we'll use a simplified approach
        
        Ok(limited_value)
    }

    pub async fn read_current_command(&self) -> MotorCommand {
        self.current_command.read().await.clone()
    }

    pub async fn is_enabled(&self) -> bool {
        *self.enabled.read().await
    }

    pub async fn is_fault(&self) -> bool {
        *self.fault.read().await
    }

    pub async fn is_emergency_stopped(&self) -> bool {
        *self.emergency_stopped.read().await
    }

    pub async fn self_test(&self) -> Result<String> {
        let mut test_results = Vec::new();

        // Test output plugin
        match self.output_plugin.self_test(&self.config.output_channel).await {
            Ok(result) => {
                test_results.push(format!("Output: {}", result));
            }
            Err(e) => {
                test_results.push(format!("Output: FAIL ({})", e));
            }
        }

        // Test reference input if present
        if let Some(ref reference) = self.reference_input {
            match reference.read().await {
                Ok(value) => {
                    test_results.push(format!("Reference: OK (value: {:.3})", value));
                }
                Err(e) => {
                    test_results.push(format!("Reference: FAIL ({})", e));
                }
            }
        }

        // Test conversion expression if present
        if !self.config.conversion_expression.is_empty() {
            let mut context = self.expression_context.lock().await;
            context.set_value("command".to_string(), 1.0.into())?;
            context.set_value("reference".to_string(), 0.0.into())?;
            
            match eval(&self.config.conversion_expression) {
                Ok(result) => {
                    test_results.push(format!("Conversion: OK (test result: {:.3})", 
                                            result.as_float().unwrap_or(0.0)));
                }
                Err(e) => {
                    test_results.push(format!("Conversion: FAIL ({})", e));
                }
            }
        }

        // Test limits
        let test_value = self.config.limits.max_value + 1.0;
        match self.apply_limits(test_value).await {
            Ok(limited) => {
                if limited <= self.config.limits.max_value {
                    test_results.push("Limits: OK".to_string());
                } else {
                    test_results.push("Limits: FAIL (max limit not enforced)".to_string());
                }
            }
            Err(e) => {
                test_results.push(format!("Limits: FAIL ({})", e));
            }
        }

        Ok(test_results.join("; "))
    }

    pub async fn get_status(&self) -> MotorStatus {
        MotorStatus {
            id: self.id.clone(),
            enabled: *self.enabled.read().await,
            fault: *self.fault.read().await,
            emergency_stopped: *self.emergency_stopped.read().await,
            current_command: self.current_command.read().await.clone(),
            limits: self.config.limits.clone(),
        }
    }

    pub async fn calibrate(&self) -> Result<()> {
        info!("Calibrating motor: {}", self.id);

        if !*self.enabled.read().await {
            anyhow::bail!("Motor must be enabled for calibration");
        }

        // Perform calibration sequence
        // This is a simplified calibration - in practice, this would be more complex
        
        // Move to minimum position
        self.write(self.config.limits.min_value).await?;
        tokio::time::sleep(tokio::time::Duration::from_millis(1000)).await;

        // Move to maximum position
        self.write(self.config.limits.max_value).await?;
        tokio::time::sleep(tokio::time::Duration::from_millis(1000)).await;

        // Return to center
        let center = (self.config.limits.min_value + self.config.limits.max_value) / 2.0;
        self.write(center).await?;
        tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;

        // Return to zero
        self.write(0.0).await?;

        info!("Motor calibration completed");
        Ok(())
    }

    pub async fn shutdown(&self) -> Result<()> {
        info!("Shutting down motor: {}", self.id);

        // Deactivate if still active
        if *self.enabled.read().await {
            self.deactivate().await?;
        }

        // Shutdown data logging
        if let Some(ref logger) = self.data_logger {
            logger.shutdown().await?;
        }

        info!("Motor shutdown complete");
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct MotorStatus {
    pub id: MotorId,
    pub enabled: bool,
    pub fault: bool,
    pub emergency_stopped: bool,
    pub current_command: MotorCommand,
    pub limits: crate::config::MotorLimits,
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[tokio::test]
    async fn test_motor_command_creation() {
        let motor_command = MotorCommand {
            value: 5.0,
            timestamp: std::time::SystemTime::now(),
            motor_id: "test_motor".to_string(),
        };

        assert_eq!(motor_command.value, 5.0);
        assert_eq!(motor_command.motor_id, "test_motor");
    }

    #[test]
    fn test_limits_application() {
        use crate::config::MotorLimits;
        
        let limits = MotorLimits {
            min_value: -10.0,
            max_value: 10.0,
            max_rate: 5.0,
            max_acceleration: 2.0,
        };

        // Test value clamping
        let test_value = 15.0;
        let clamped = test_value.clamp(limits.min_value, limits.max_value);
        assert_eq!(clamped, 10.0);

        let test_value = -15.0;
        let clamped = test_value.clamp(limits.min_value, limits.max_value);
        assert_eq!(clamped, -10.0);
    }

    #[test]
    fn test_conversion_expression() {
        use evalexpr::*;
        
        let mut context = HashMapContext::new();
        context.set_value("command".to_string(), 5.0.into()).unwrap();
        context.set_value("reference".to_string(), 2.0.into()).unwrap();
        
        // Test a simple conversion expression
        let result = eval_with_context("command * 2 + reference", &context).unwrap();
        assert_eq!(result.as_float().unwrap(), 12.0);
    }

    #[tokio::test]
    async fn test_motor_state_transitions() {
        // Test basic motor state transitions
        // This would require mock plugins for full testing
        let enabled = Arc::new(RwLock::new(false));
        let fault = Arc::new(RwLock::new(false));
        let emergency_stopped = Arc::new(RwLock::new(false));

        // Test initial state
        assert!(!*enabled.read().await);
        assert!(!*fault.read().await);
        assert!(!*emergency_stopped.read().await);

        // Test activation
        {
            let mut enabled_guard = enabled.write().await;
            *enabled_guard = true;
        }
        assert!(*enabled.read().await);

        // Test emergency stop
        {
            let mut emergency_guard = emergency_stopped.write().await;
            *emergency_guard = true;
            let mut enabled_guard = enabled.write().await;
            *enabled_guard = false;
            let mut fault_guard = fault.write().await;
            *fault_guard = true;
        }
        assert!(!*enabled.read().await);
        assert!(*fault.read().await);
        assert!(*emergency_stopped.read().await);
    }
}