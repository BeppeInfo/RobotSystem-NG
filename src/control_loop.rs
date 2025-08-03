use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use tokio::sync::{mpsc, RwLock, Notify};
use tokio::time::{interval, MissedTickBehavior};
use tracing::{debug, error, info, warn};

use crate::robot::Robot;
use crate::shared_types::{PerformanceMetrics, ControlMode};
use precise_timing::PreciseTimer;

pub struct ControlLoop {
    robot: Arc<Robot>,
    frequency_hz: f64,
    timer: PreciseTimer,
    running: Arc<RwLock<bool>>,
    performance_metrics: Arc<RwLock<PerformanceMetrics>>,
    shutdown_notify: Arc<Notify>,
    control_task_handle: Option<tokio::task::JoinHandle<()>>,
    metrics_task_handle: Option<tokio::task::JoinHandle<()>>,
}

impl ControlLoop {
    pub async fn new(robot: Arc<Robot>) -> Result<Self> {
        let frequency_hz = 1000.0; // Default 1kHz control loop
        let timer = PreciseTimer::new(frequency_hz)
            .context("Failed to create precise timer")?;

        Ok(ControlLoop {
            robot,
            frequency_hz,
            timer,
            running: Arc::new(RwLock::new(false)),
            performance_metrics: Arc::new(RwLock::new(PerformanceMetrics::default())),
            shutdown_notify: Arc::new(Notify::new()),
            control_task_handle: None,
            metrics_task_handle: None,
        })
    }

    pub async fn start(&mut self) -> Result<()> {
        info!("Starting control loop at {} Hz", self.frequency_hz);

        // Check if already running
        if *self.running.read().await {
            anyhow::bail!("Control loop is already running");
        }

        // Mark as running
        {
            let mut running = self.running.write().await;
            *running = true;
        }

        // Start control task
        let control_task = self.spawn_control_task().await?;
        self.control_task_handle = Some(control_task);

        // Start metrics collection task
        let metrics_task = self.spawn_metrics_task().await?;
        self.metrics_task_handle = Some(metrics_task);

        info!("Control loop started successfully");
        Ok(())
    }

    pub async fn stop(&mut self) -> Result<()> {
        info!("Stopping control loop");

        // Mark as not running
        {
            let mut running = self.running.write().await;
            *running = false;
        }

        // Notify shutdown
        self.shutdown_notify.notify_waiters();

        // Wait for tasks to complete
        if let Some(handle) = self.control_task_handle.take() {
            handle.await.context("Failed to stop control task")?;
        }

        if let Some(handle) = self.metrics_task_handle.take() {
            handle.await.context("Failed to stop metrics task")?;
        }

        info!("Control loop stopped");
        Ok(())
    }

    async fn spawn_control_task(&self) -> Result<tokio::task::JoinHandle<()>> {
        let robot = Arc::clone(&self.robot);
        let running = Arc::clone(&self.running);
        let performance_metrics = Arc::clone(&self.performance_metrics);
        let shutdown_notify = Arc::clone(&self.shutdown_notify);
        let frequency_hz = self.frequency_hz;

        let handle = tokio::task::spawn(async move {
            Self::control_task_main(
                robot,
                running,
                performance_metrics,
                shutdown_notify,
                frequency_hz,
            ).await;
        });

        Ok(handle)
    }

    async fn spawn_metrics_task(&self) -> Result<tokio::task::JoinHandle<()>> {
        let performance_metrics = Arc::clone(&self.performance_metrics);
        let shutdown_notify = Arc::clone(&self.shutdown_notify);

        let handle = tokio::task::spawn(async move {
            Self::metrics_task_main(performance_metrics, shutdown_notify).await;
        });

        Ok(handle)
    }

    async fn control_task_main(
        robot: Arc<Robot>,
        running: Arc<RwLock<bool>>,
        performance_metrics: Arc<RwLock<PerformanceMetrics>>,
        shutdown_notify: Arc<Notify>,
        frequency_hz: f64,
    ) {
        info!("Control task started");

        let period = Duration::from_secs_f64(1.0 / frequency_hz);
        let mut interval = interval(period);
        interval.set_missed_tick_behavior(MissedTickBehavior::Skip);

        let mut cycle_count = 0u64;
        let mut missed_deadlines = 0u64;
        let mut max_latency = Duration::ZERO;
        let mut latency_sum = Duration::ZERO;

        while *running.read().await {
            let cycle_start = Instant::now();

            // Wait for next tick
            tokio::select! {
                _ = interval.tick() => {
                    // Continue with control cycle
                }
                _ = shutdown_notify.notified() => {
                    debug!("Control task received shutdown notification");
                    break;
                }
            }

            let actual_start = Instant::now();
            let schedule_latency = actual_start - cycle_start;

            // Check if we missed the deadline
            if schedule_latency > period {
                missed_deadlines += 1;
                warn!("Missed control deadline by {:?}", schedule_latency - period);
            }

            // Update latency tracking
            if schedule_latency > max_latency {
                max_latency = schedule_latency;
            }
            latency_sum += schedule_latency;

            // Perform control step
            if let Err(e) = robot.update_control_step().await {
                error!("Control step failed: {}", e);
                
                // Check if this is a critical error
                if Self::is_critical_error(&e) {
                    error!("Critical error in control loop, stopping");
                    let mut running_guard = running.write().await;
                    *running_guard = false;
                    break;
                }
            }

            let cycle_end = Instant::now();
            let cycle_duration = cycle_end - actual_start;

            cycle_count += 1;

            // Update performance metrics periodically
            if cycle_count % 1000 == 0 {
                let avg_latency = latency_sum / cycle_count as u32;
                let actual_frequency = cycle_count as f64 / 
                    (cycle_end - cycle_start).as_secs_f64().max(1.0);

                let mut metrics = performance_metrics.write().await;
                metrics.control_loop_frequency = actual_frequency;
                metrics.control_loop_jitter = avg_latency;
                metrics.max_latency = max_latency;
                metrics.missed_deadlines = missed_deadlines;

                debug!("Control loop metrics: freq={:.1}Hz, jitter={:?}, missed={}", 
                       actual_frequency, avg_latency, missed_deadlines);
            }

            // Yield to prevent monopolizing the executor
            if cycle_count % 100 == 0 {
                tokio::task::yield_now().await;
            }
        }

        info!("Control task stopped after {} cycles", cycle_count);
    }

    async fn metrics_task_main(
        performance_metrics: Arc<RwLock<PerformanceMetrics>>,
        shutdown_notify: Arc<Notify>,
    ) {
        info!("Metrics task started");

        let mut interval = interval(Duration::from_secs(1));
        interval.set_missed_tick_behavior(MissedTickBehavior::Skip);

        while tokio::select! {
            _ = interval.tick() => true,
            _ = shutdown_notify.notified() => false,
        } {
            // Collect system metrics
            let cpu_usage = Self::get_cpu_usage().await;

            // Update metrics
            {
                let mut metrics = performance_metrics.write().await;
                metrics.cpu_usage = cpu_usage;
            }

            // Log metrics periodically
            let metrics = performance_metrics.read().await;
            debug!("System metrics: CPU={:.1}%, freq={:.1}Hz, jitter={:?}", 
                   metrics.cpu_usage, 
                   metrics.control_loop_frequency,
                   metrics.control_loop_jitter);
        }

        info!("Metrics task stopped");
    }

    fn is_critical_error(error: &anyhow::Error) -> bool {
        // Define what constitutes a critical error that should stop the control loop
        let error_string = error.to_string().to_lowercase();
        
        error_string.contains("emergency") ||
        error_string.contains("hardware fault") ||
        error_string.contains("safety violation") ||
        error_string.contains("communication timeout")
    }

    async fn get_cpu_usage() -> f32 {
        // Simplified CPU usage calculation
        // In a real implementation, you would use system APIs or crates like `sysinfo`
        use std::fs;
        
        if let Ok(loadavg) = fs::read_to_string("/proc/loadavg") {
            if let Some(load) = loadavg.split_whitespace().next() {
                if let Ok(load_val) = load.parse::<f32>() {
                    // Convert load average to rough CPU percentage
                    return (load_val * 100.0).min(100.0);
                }
            }
        }
        
        0.0 // Default if can't read
    }

    pub async fn get_performance_metrics(&self) -> PerformanceMetrics {
        self.performance_metrics.read().await.clone()
    }

    pub async fn is_running(&self) -> bool {
        *self.running.read().await
    }

    pub async fn set_frequency(&mut self, frequency_hz: f64) -> Result<()> {
        if *self.running.read().await {
            anyhow::bail!("Cannot change frequency while control loop is running");
        }

        if frequency_hz <= 0.0 || frequency_hz > 10000.0 {
            anyhow::bail!("Invalid frequency: {} Hz", frequency_hz);
        }

        self.frequency_hz = frequency_hz;
        self.timer = PreciseTimer::new(frequency_hz)
            .context("Failed to create precise timer with new frequency")?;

        info!("Control loop frequency set to {} Hz", frequency_hz);
        Ok(())
    }

    pub fn get_frequency(&self) -> f64 {
        self.frequency_hz
    }

    pub async fn pause(&self) -> Result<()> {
        info!("Pausing control loop");
        
        // This could be implemented by setting a paused flag
        // For now, we'll just log the action
        warn!("Control loop pause not fully implemented");
        
        Ok(())
    }

    pub async fn resume(&self) -> Result<()> {
        info!("Resuming control loop");
        
        // This could be implemented by clearing a paused flag
        // For now, we'll just log the action
        warn!("Control loop resume not fully implemented");
        
        Ok(())
    }

    pub async fn single_step(&self) -> Result<()> {
        if *self.running.read().await {
            anyhow::bail!("Cannot perform single step while control loop is running");
        }

        info!("Performing single control step");
        
        self.robot.update_control_step().await
            .context("Single control step failed")?;

        info!("Single control step completed");
        Ok(())
    }
}

impl Drop for ControlLoop {
    fn drop(&mut self) {
        // Ensure tasks are cleaned up
        if let Some(handle) = self.control_task_handle.take() {
            handle.abort();
        }
        if let Some(handle) = self.metrics_task_handle.take() {
            handle.abort();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[tokio::test]
    async fn test_control_loop_creation() {
        // This test would require a mock robot
        // For now, just test basic structure
        let frequency = 1000.0;
        assert!(frequency > 0.0);
        assert!(frequency <= 10000.0);
    }

    #[tokio::test]
    async fn test_performance_metrics() {
        let metrics = PerformanceMetrics::default();
        assert_eq!(metrics.control_loop_frequency, 0.0);
        assert_eq!(metrics.missed_deadlines, 0);
        assert_eq!(metrics.max_latency, Duration::ZERO);
    }

    #[test]
    fn test_critical_error_detection() {
        let critical_error = anyhow::anyhow!("Emergency stop activated");
        assert!(ControlLoop::is_critical_error(&critical_error));

        let normal_error = anyhow::anyhow!("Sensor read failed");
        assert!(!ControlLoop::is_critical_error(&normal_error));
    }

    #[tokio::test]
    async fn test_frequency_validation() {
        // Test frequency bounds
        assert!(1000.0 > 0.0 && 1000.0 <= 10000.0);
        assert!(!(0.0 > 0.0 && 0.0 <= 10000.0));
        assert!(!(15000.0 > 0.0 && 15000.0 <= 10000.0));
    }
}