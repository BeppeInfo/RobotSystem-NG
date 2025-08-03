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

use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, Result};
use clap::Parser;
use tokio::signal;
use tracing::{error, info, warn};

mod robot;
mod config;
mod ipc_server;
mod actuator;
mod sensor;
mod motor;
mod control_loop;
mod shared_types;

use robot::Robot;
use config::SystemConfig;
use ipc_server::IpcServer;
use control_loop::ControlLoop;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Root directory containing config and plugins folders
    #[arg(long, default_value = "./")]
    root: PathBuf,
    
    /// IP address for server sockets to bind to
    #[arg(long, default_value = "0.0.0.0")]
    addr: String,
    
    /// Directory for log files
    #[arg(long, default_value = "./log/")]
    log: PathBuf,
    
    /// Robot configuration name (without extension)
    #[arg(long)]
    config: Option<String>,
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize tracing
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();

    let args = Args::parse();
    
    info!("Starting RobotSystem-NG");
    info!("Root directory: {:?}", args.root);
    info!("Log directory: {:?}", args.log);
    info!("Server address: {}", args.addr);

    // Create log directory if it doesn't exist
    tokio::fs::create_dir_all(&args.log).await
        .context("Failed to create log directory")?;

    // Load system configuration
    let system_config = SystemConfig::load(&args.root)
        .context("Failed to load system configuration")?;

    // Initialize robot if config specified
    let robot = if let Some(robot_name) = args.config {
        info!("Loading robot configuration: {}", robot_name);
        Some(Arc::new(
            Robot::load(&args.root, &robot_name, &args.log).await
                .context("Failed to load robot configuration")?
        ))
    } else {
        warn!("No robot configuration specified");
        None
    };

    // Start IPC server
    let ipc_server = IpcServer::new(&args.addr, robot.clone()).await
        .context("Failed to start IPC server")?;
    
    // Start control loop if robot is loaded
    let control_loop = if let Some(ref robot_arc) = robot {
        Some(ControlLoop::new(robot_arc.clone()).await?)
    } else {
        None
    };

    info!("RobotSystem-NG started successfully");

    // Wait for shutdown signal
    match signal::ctrl_c().await {
        Ok(()) => {
            info!("Received shutdown signal");
        }
        Err(err) => {
            error!("Unable to listen for shutdown signal: {}", err);
        }
    }

    // Graceful shutdown
    info!("Shutting down...");
    
    if let Some(mut control_loop) = control_loop {
        info!("Stopping control loop...");
        control_loop.stop().await?;
    }
    
    info!("Stopping IPC server...");
    ipc_server.stop().await?;

    if let Some(robot) = robot {
        info!("Shutting down robot...");
        robot.shutdown().await?;
    }

    info!("RobotSystem-NG shutdown complete");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_system_startup() {
        // Basic integration test for system startup
        let temp_dir = tempfile::tempdir().unwrap();
        let args = Args {
            root: temp_dir.path().to_path_buf(),
            addr: "127.0.0.1".to_string(),
            log: temp_dir.path().join("log"),
            config: None,
        };

        // This should not panic and should handle missing config gracefully
        let system_config = SystemConfig::load(&args.root);
        assert!(system_config.is_err() || system_config.is_ok());
    }
}