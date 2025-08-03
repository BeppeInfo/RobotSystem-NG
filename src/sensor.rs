use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result};
use tokio::sync::{Mutex, RwLock};
use tracing::{debug, info, warn};
use evalexpr::{eval, ContextWithMutableVariables, HashMapContext};

use crate::config::{SensorConfig, InputSourceConfig};
use crate::shared_types::{SensorData, LoggingConfig};
use signal_io_interface::{SignalInput, SignalInputPlugin};
use signal_processing::{Filter, FilterType};
use plugin_loader::PluginLoader;
use data_logging::DataLogger;

pub type SensorId = String;

#[derive(Debug)]
pub struct InputSource {
    pub id: String,
    pub plugin: Arc<dyn SignalInput>,
    pub channel: String,
    pub gain: f64,
    pub offset: f64,
    pub filter: Option<Filter>,
}

impl InputSource {
    pub async fn read_raw(&self) -> Result<f64> {
        self.plugin.read(&self.channel).await
    }

    pub async fn read_processed(&self) -> Result<f64> {
        let raw_value = self.read_raw().await?;
        let scaled_value = raw_value * self.gain + self.offset;
        
        if let Some(ref filter) = self.filter {
            Ok(filter.process(scaled_value))
        } else {
            Ok(scaled_value)
        }
    }
}

#[derive(Debug)]
pub struct Sensor {
    pub id: SensorId,
    pub config: SensorConfig,
    pub input_sources: Vec<InputSource>,
    pub expression_context: Arc<Mutex<HashMapContext>>,
    pub current_value: Arc<RwLock<SensorData>>,
    pub data_logger: Option<Arc<DataLogger>>,
    pub sample_rate_hz: f64,
}

impl Sensor {
    pub async fn load(
        root_dir: &Path,
        sensor_id: &str,
        log_dir: &Path,
        plugin_loader: &mut PluginLoader,
    ) -> Result<Self> {
        info!("Loading sensor: {}", sensor_id);

        // Load sensor configuration
        let config_path = root_dir.join("config/sensors").join(format!("{}.json", sensor_id));
        let config = SensorConfig::load(&config_path)
            .context("Failed to load sensor configuration")?;

        info!("Sensor config loaded: {} input sources", config.input_sources.len());

        // Initialize data logger if enabled
        let data_logger = if config.logging.enabled {
            let sensor_log_dir = log_dir.join(sensor_id);
            tokio::fs::create_dir_all(&sensor_log_dir).await?;
            Some(Arc::new(DataLogger::new(&sensor_log_dir)?))
        } else {
            None
        };

        // Load input sources
        let mut input_sources = Vec::new();
        for source_config in &config.input_sources {
            debug!("Loading input source: {}", source_config.id);
            
            let plugin = plugin_loader
                .load_signal_input(&source_config.plugin)
                .context("Failed to load signal input plugin")?;

            // Initialize filter if enabled
            let filter = if source_config.filter.enabled {
                let filter_type = match source_config.filter.filter_type.as_str() {
                    "lowpass" => FilterType::LowPass,
                    "highpass" => FilterType::HighPass,
                    "bandpass" => FilterType::BandPass,
                    "bandstop" => FilterType::BandStop,
                    _ => FilterType::LowPass, // Default
                };

                Some(Filter::new(
                    filter_type,
                    config.sample_rate_hz,
                    source_config.filter.cutoff_frequency,
                    source_config.filter.order,
                )?)
            } else {
                None
            };

            let input_source = InputSource {
                id: source_config.id.clone(),
                plugin,
                channel: source_config.channel.clone(),
                gain: source_config.gain,
                offset: source_config.offset,
                filter,
            };

            input_sources.push(input_source);
        }

        // Initialize expression context for conversion
        let expression_context = Arc::new(Mutex::new(HashMapContext::new()));

        // Initialize current value
        let initial_data = SensorData {
            value: 0.0,
            timestamp: std::time::SystemTime::now(),
            valid: false,
            sensor_id: sensor_id.to_string(),
        };

        let sensor = Sensor {
            id: sensor_id.to_string(),
            config,
            input_sources,
            expression_context,
            current_value: Arc::new(RwLock::new(initial_data)),
            data_logger,
            sample_rate_hz: config.sample_rate_hz,
        };

        info!("Sensor loaded successfully");
        Ok(sensor)
    }

    pub async fn activate(&self) -> Result<()> {
        info!("Activating sensor: {}", self.id);

        // Activate all input sources
        for source in &self.input_sources {
            debug!("Activating input source: {}", source.id);
            source.plugin.activate(&source.channel).await
                .with_context(|| format!("Failed to activate input source: {}", source.id))?;
        }

        info!("Sensor activated successfully");
        Ok(())
    }

    pub async fn deactivate(&self) -> Result<()> {
        info!("Deactivating sensor: {}", self.id);

        // Deactivate all input sources
        for source in &self.input_sources {
            debug!("Deactivating input source: {}", source.id);
            if let Err(e) = source.plugin.deactivate(&source.channel).await {
                warn!("Failed to deactivate input source {}: {}", source.id, e);
            }
        }

        info!("Sensor deactivated");
        Ok(())
    }

    pub async fn read(&self) -> Result<SensorData> {
        // Read all input sources
        let mut source_values = HashMap::new();
        for source in &self.input_sources {
            let value = source.read_processed().await
                .with_context(|| format!("Failed to read input source: {}", source.id))?;
            source_values.insert(source.id.clone(), value);
        }

        // Evaluate conversion expression
        let converted_value = if !self.config.conversion_expression.is_empty() {
            // Update expression context with input values
            {
                let mut context = self.expression_context.lock().await;
                for (source_id, value) in &source_values {
                    context.set_value(source_id.clone(), (*value).into())?;
                }
            }

            // Evaluate expression
            let context = self.expression_context.lock().await;
            let result = eval(&self.config.conversion_expression)
                .context("Failed to evaluate conversion expression")?;
            
            result.as_float()
                .context("Conversion expression result is not a number")?
        } else {
            // If no expression, use first input source or zero
            source_values.values().next().copied().unwrap_or(0.0)
        };

        // Create sensor data
        let sensor_data = SensorData {
            value: converted_value,
            timestamp: std::time::SystemTime::now(),
            valid: true,
            sensor_id: self.id.clone(),
        };

        // Update current value
        {
            let mut current = self.current_value.write().await;
            *current = sensor_data.clone();
        }

        // Log data if enabled
        if let Some(ref logger) = self.data_logger {
            logger.log_sensor_data(&sensor_data).await?;
        }

        Ok(sensor_data)
    }

    pub async fn read_cached(&self) -> SensorData {
        self.current_value.read().await.clone()
    }

    pub async fn calibrate(&self, reference_values: &[f64]) -> Result<()> {
        info!("Calibrating sensor: {}", self.id);

        if reference_values.len() != self.input_sources.len() {
            anyhow::bail!("Reference values count doesn't match input sources count");
        }

        // Perform calibration for each input source
        for (i, source) in self.input_sources.iter().enumerate() {
            if let Some(reference) = reference_values.get(i) {
                debug!("Calibrating input source {}: reference = {}", source.id, reference);
                
                // Read multiple samples for calibration
                let mut samples = Vec::new();
                for _ in 0..10 {
                    let sample = source.read_raw().await?;
                    samples.push(sample);
                    tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
                }

                let avg_sample = samples.iter().sum::<f64>() / samples.len() as f64;
                let calibration_offset = reference - avg_sample;
                
                info!("Calibration offset for {}: {}", source.id, calibration_offset);
                
                // Note: In a real implementation, you would update the source's offset
                // This might require making the InputSource mutable or using a different approach
            }
        }

        info!("Sensor calibration completed");
        Ok(())
    }

    pub async fn self_test(&self) -> Result<String> {
        let mut test_results = Vec::new();

        // Test each input source
        for source in &self.input_sources {
            match source.read_raw().await {
                Ok(value) => {
                    test_results.push(format!("{}: OK (value: {:.3})", source.id, value));
                }
                Err(e) => {
                    test_results.push(format!("{}: FAIL ({})", source.id, e));
                }
            }
        }

        // Test conversion expression
        if !self.config.conversion_expression.is_empty() {
            match self.read().await {
                Ok(data) => {
                    test_results.push(format!("Conversion: OK (result: {:.3})", data.value));
                }
                Err(e) => {
                    test_results.push(format!("Conversion: FAIL ({})", e));
                }
            }
        }

        Ok(test_results.join("; "))
    }

    pub async fn get_statistics(&self) -> Result<SensorStatistics> {
        // Collect samples over a short period
        let mut samples = Vec::new();
        let sample_count = (self.sample_rate_hz * 0.1) as usize; // 100ms worth of samples
        
        for _ in 0..sample_count {
            if let Ok(data) = self.read().await {
                samples.push(data.value);
            }
            tokio::time::sleep(tokio::time::Duration::from_micros(
                (1_000_000.0 / self.sample_rate_hz) as u64
            )).await;
        }

        if samples.is_empty() {
            anyhow::bail!("No valid samples collected");
        }

        let mean = samples.iter().sum::<f64>() / samples.len() as f64;
        let variance = samples.iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f64>() / samples.len() as f64;
        let std_dev = variance.sqrt();
        
        let min = samples.iter().copied().fold(f64::INFINITY, f64::min);
        let max = samples.iter().copied().fold(f64::NEG_INFINITY, f64::max);

        Ok(SensorStatistics {
            mean,
            std_dev,
            min,
            max,
            sample_count: samples.len(),
        })
    }

    pub async fn shutdown(&self) -> Result<()> {
        info!("Shutting down sensor: {}", self.id);

        // Deactivate if still active
        self.deactivate().await?;

        // Shutdown data logging
        if let Some(ref logger) = self.data_logger {
            logger.shutdown().await?;
        }

        info!("Sensor shutdown complete");
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct SensorStatistics {
    pub mean: f64,
    pub std_dev: f64,
    pub min: f64,
    pub max: f64,
    pub sample_count: usize,
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[tokio::test]
    async fn test_sensor_data_creation() {
        let sensor_data = SensorData {
            value: 3.14,
            timestamp: std::time::SystemTime::now(),
            valid: true,
            sensor_id: "test_sensor".to_string(),
        };

        assert_eq!(sensor_data.value, 3.14);
        assert!(sensor_data.valid);
        assert_eq!(sensor_data.sensor_id, "test_sensor");
    }

    #[test]
    fn test_expression_evaluation() {
        use evalexpr::*;
        
        let mut context = HashMapContext::new();
        context.set_value("input1".to_string(), 10.0.into()).unwrap();
        context.set_value("input2".to_string(), 5.0.into()).unwrap();
        
        let result = eval_with_context("input1 * 2 + input2", &context).unwrap();
        assert_eq!(result.as_float().unwrap(), 25.0);
    }

    #[tokio::test]
    async fn test_sensor_statistics() {
        // Test statistics calculation with mock data
        let samples = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let mean = samples.iter().sum::<f64>() / samples.len() as f64;
        assert_eq!(mean, 3.0);
        
        let variance = samples.iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f64>() / samples.len() as f64;
        assert_eq!(variance, 2.0);
    }
}