use std::sync::Arc;
use std::collections::HashMap;

use anyhow::{Context, Result};
use tokio::sync::{mpsc, RwLock, Notify};
use tokio::net::{TcpListener, TcpStream};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tracing::{debug, error, info, warn};
use zmq::{Context as ZmqContext, Socket as ZmqSocket, SocketType};

use crate::robot::Robot;
use crate::shared_types::{
    IpcMessage, IpcResponse, IpcMessageBuffer, ControlMode, AxisState, JointState, ControlVariables
};

pub struct IpcServer {
    robot: Option<Arc<Robot>>,
    request_reply_socket: ZmqSocket,
    publisher_socket: ZmqSocket,
    subscriber_socket: ZmqSocket,
    tcp_listener: Option<TcpListener>,
    running: Arc<RwLock<bool>>,
    shutdown_notify: Arc<Notify>,
    task_handles: Vec<tokio::task::JoinHandle<()>>,
}

impl IpcServer {
    pub async fn new(bind_address: &str, robot: Option<Arc<Robot>>) -> Result<Self> {
        info!("Starting IPC server on address: {}", bind_address);

        // Initialize ZMQ context
        let zmq_context = ZmqContext::new();

        // Create request-reply socket
        let request_reply_socket = zmq_context.socket(SocketType::REP)
            .context("Failed to create request-reply socket")?;
        let req_rep_addr = format!("tcp://{}:5555", bind_address);
        request_reply_socket.bind(&req_rep_addr)
            .context("Failed to bind request-reply socket")?;

        // Create publisher socket for real-time data
        let publisher_socket = zmq_context.socket(SocketType::PUB)
            .context("Failed to create publisher socket")?;
        let pub_addr = format!("tcp://{}:5556", bind_address);
        publisher_socket.bind(&pub_addr)
            .context("Failed to bind publisher socket")?;

        // Create subscriber socket for commands
        let subscriber_socket = zmq_context.socket(SocketType::SUB)
            .context("Failed to create subscriber socket")?;
        let sub_addr = format!("tcp://{}:5557", bind_address);
        subscriber_socket.bind(&sub_addr)
            .context("Failed to bind subscriber socket")?;
        
        // Subscribe to all messages
        subscriber_socket.set_subscribe(b"")
            .context("Failed to set subscriber filter")?;

        // Optional TCP fallback listener
        let tcp_listener = if bind_address != "0.0.0.0" {
            Some(TcpListener::bind(format!("{}:8080", bind_address)).await
                .context("Failed to bind TCP listener")?)
        } else {
            None
        };

        info!("IPC server bound to:");
        info!("  Request-Reply: {}", req_rep_addr);
        info!("  Publisher: {}", pub_addr);
        info!("  Subscriber: {}", sub_addr);
        if tcp_listener.is_some() {
            info!("  TCP Fallback: {}:8080", bind_address);
        }

        let mut server = IpcServer {
            robot,
            request_reply_socket,
            publisher_socket,
            subscriber_socket,
            tcp_listener,
            running: Arc::new(RwLock::new(true)),
            shutdown_notify: Arc::new(Notify::new()),
            task_handles: Vec::new(),
        };

        server.start_tasks().await?;

        Ok(server)
    }

    async fn start_tasks(&mut self) -> Result<()> {
        // Start request-reply handler
        let req_rep_handle = self.spawn_request_reply_task().await?;
        self.task_handles.push(req_rep_handle);

        // Start subscriber handler
        let sub_handle = self.spawn_subscriber_task().await?;
        self.task_handles.push(sub_handle);

        // Start data publisher
        let pub_handle = self.spawn_publisher_task().await?;
        self.task_handles.push(pub_handle);

        // Start TCP handler if enabled
        if self.tcp_listener.is_some() {
            let tcp_handle = self.spawn_tcp_task().await?;
            self.task_handles.push(tcp_handle);
        }

        info!("All IPC server tasks started");
        Ok(())
    }

    async fn spawn_request_reply_task(&self) -> Result<tokio::task::JoinHandle<()>> {
        let robot = self.robot.clone();
        let running = Arc::clone(&self.running);
        let shutdown_notify = Arc::clone(&self.shutdown_notify);

        // Note: In a real implementation, we'd need to properly handle ZMQ sockets in async context
        // This is a simplified version for demonstration
        let handle = tokio::task::spawn(async move {
            Self::request_reply_task_main(robot, running, shutdown_notify).await;
        });

        Ok(handle)
    }

    async fn spawn_subscriber_task(&self) -> Result<tokio::task::JoinHandle<()>> {
        let robot = self.robot.clone();
        let running = Arc::clone(&self.running);
        let shutdown_notify = Arc::clone(&self.shutdown_notify);

        let handle = tokio::task::spawn(async move {
            Self::subscriber_task_main(robot, running, shutdown_notify).await;
        });

        Ok(handle)
    }

    async fn spawn_publisher_task(&self) -> Result<tokio::task::JoinHandle<()>> {
        let robot = self.robot.clone();
        let running = Arc::clone(&self.running);
        let shutdown_notify = Arc::clone(&self.shutdown_notify);

        let handle = tokio::task::spawn(async move {
            Self::publisher_task_main(robot, running, shutdown_notify).await;
        });

        Ok(handle)
    }

    async fn spawn_tcp_task(&self) -> Result<tokio::task::JoinHandle<()>> {
        let robot = self.robot.clone();
        let running = Arc::clone(&self.running);
        let shutdown_notify = Arc::clone(&self.shutdown_notify);
        let listener = self.tcp_listener.as_ref().unwrap().try_clone()
            .context("Failed to clone TCP listener")?;

        let handle = tokio::task::spawn(async move {
            Self::tcp_task_main(robot, running, shutdown_notify, listener).await;
        });

        Ok(handle)
    }

    async fn request_reply_task_main(
        robot: Option<Arc<Robot>>,
        running: Arc<RwLock<bool>>,
        shutdown_notify: Arc<Notify>,
    ) {
        info!("Request-reply task started");

        while *running.read().await {
            tokio::select! {
                // In a real implementation, we'd poll the ZMQ socket here
                _ = tokio::time::sleep(tokio::time::Duration::from_millis(10)) => {
                    // Simulate message processing
                    if let Some(ref robot) = robot {
                        // Process any pending request-reply messages
                        // This would involve reading from the ZMQ socket and processing commands
                    }
                }
                _ = shutdown_notify.notified() => {
                    debug!("Request-reply task received shutdown notification");
                    break;
                }
            }
        }

        info!("Request-reply task stopped");
    }

    async fn subscriber_task_main(
        robot: Option<Arc<Robot>>,
        running: Arc<RwLock<bool>>,
        shutdown_notify: Arc<Notify>,
    ) {
        info!("Subscriber task started");

        while *running.read().await {
            tokio::select! {
                // In a real implementation, we'd poll the ZMQ socket here
                _ = tokio::time::sleep(tokio::time::Duration::from_millis(1)) => {
                    // Process incoming control updates
                    if let Some(ref robot) = robot {
                        // Handle axis/joint updates from clients
                    }
                }
                _ = shutdown_notify.notified() => {
                    debug!("Subscriber task received shutdown notification");
                    break;
                }
            }
        }

        info!("Subscriber task stopped");
    }

    async fn publisher_task_main(
        robot: Option<Arc<Robot>>,
        running: Arc<RwLock<bool>>,
        shutdown_notify: Arc<Notify>,
    ) {
        info!("Publisher task started");

        let mut interval = tokio::time::interval(tokio::time::Duration::from_millis(1)); // 1kHz
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

        while *running.read().await {
            tokio::select! {
                _ = interval.tick() => {
                    if let Some(ref robot) = robot {
                        // Publish robot state data
                        if let Err(e) = Self::publish_robot_state(robot).await {
                            error!("Failed to publish robot state: {}", e);
                        }
                    }
                }
                _ = shutdown_notify.notified() => {
                    debug!("Publisher task received shutdown notification");
                    break;
                }
            }
        }

        info!("Publisher task stopped");
    }

    async fn tcp_task_main(
        robot: Option<Arc<Robot>>,
        running: Arc<RwLock<bool>>,
        shutdown_notify: Arc<Notify>,
        listener: TcpListener,
    ) {
        info!("TCP task started");

        while *running.read().await {
            tokio::select! {
                accept_result = listener.accept() => {
                    match accept_result {
                        Ok((stream, addr)) => {
                            info!("TCP client connected: {}", addr);
                            let robot_clone = robot.clone();
                            let running_clone = Arc::clone(&running);
                            
                            tokio::spawn(async move {
                                if let Err(e) = Self::handle_tcp_client(stream, robot_clone, running_clone).await {
                                    error!("TCP client error: {}", e);
                                }
                            });
                        }
                        Err(e) => {
                            error!("Failed to accept TCP connection: {}", e);
                        }
                    }
                }
                _ = shutdown_notify.notified() => {
                    debug!("TCP task received shutdown notification");
                    break;
                }
            }
        }

        info!("TCP task stopped");
    }

    async fn handle_tcp_client(
        mut stream: TcpStream,
        robot: Option<Arc<Robot>>,
        running: Arc<RwLock<bool>>,
    ) -> Result<()> {
        let mut buffer = vec![0u8; 1024];

        while *running.read().await {
            tokio::select! {
                read_result = stream.read(&mut buffer) => {
                    match read_result {
                        Ok(0) => {
                            debug!("TCP client disconnected");
                            break;
                        }
                        Ok(n) => {
                            // Process TCP message
                            if let Some(response) = Self::process_tcp_message(&buffer[..n], &robot).await {
                                if let Err(e) = stream.write_all(&response).await {
                                    error!("Failed to write TCP response: {}", e);
                                    break;
                                }
                            }
                        }
                        Err(e) => {
                            error!("TCP read error: {}", e);
                            break;
                        }
                    }
                }
                _ = tokio::time::sleep(tokio::time::Duration::from_secs(30)) => {
                    // Timeout - send keepalive or close
                    debug!("TCP connection timeout");
                    break;
                }
            }
        }

        Ok(())
    }

    async fn process_tcp_message(
        data: &[u8],
        robot: &Option<Arc<Robot>>,
    ) -> Option<Vec<u8>> {
        // Try to parse as JSON first, then as binary
        if let Ok(json_str) = std::str::from_utf8(data) {
            if let Ok(message) = serde_json::from_str::<IpcMessage>(json_str) {
                let response = Self::process_ipc_message(message, robot).await;
                return serde_json::to_vec(&response).ok();
            }
        }

        // Try binary format
        if let Ok(message) = bincode::deserialize::<IpcMessage>(data) {
            let response = Self::process_ipc_message(message, robot).await;
            return bincode::serialize(&response).ok();
        }

        None
    }

    async fn process_ipc_message(
        message: IpcMessage,
        robot: &Option<Arc<Robot>>,
    ) -> IpcResponse {
        match message {
            IpcMessage::Activate => {
                if let Some(ref robot) = robot {
                    match robot.activate().await {
                        Ok(()) => IpcResponse::Ok,
                        Err(e) => IpcResponse::Error(e.to_string()),
                    }
                } else {
                    IpcResponse::Error("No robot loaded".to_string())
                }
            }
            IpcMessage::Deactivate => {
                if let Some(ref robot) = robot {
                    match robot.deactivate().await {
                        Ok(()) => IpcResponse::Ok,
                        Err(e) => IpcResponse::Error(e.to_string()),
                    }
                } else {
                    IpcResponse::Error("No robot loaded".to_string())
                }
            }
            IpcMessage::EmergencyStop => {
                if let Some(ref robot) = robot {
                    match robot.emergency_stop().await {
                        Ok(()) => IpcResponse::Ok,
                        Err(e) => IpcResponse::Error(e.to_string()),
                    }
                } else {
                    IpcResponse::Error("No robot loaded".to_string())
                }
            }
            IpcMessage::ResetEmergencyStop => {
                if let Some(ref robot) = robot {
                    match robot.reset_emergency_stop().await {
                        Ok(()) => IpcResponse::Ok,
                        Err(e) => IpcResponse::Error(e.to_string()),
                    }
                } else {
                    IpcResponse::Error("No robot loaded".to_string())
                }
            }
            IpcMessage::SetControlMode(mode) => {
                if let Some(ref robot) = robot {
                    match robot.set_control_mode(mode).await {
                        Ok(()) => IpcResponse::Ok,
                        Err(e) => IpcResponse::Error(e.to_string()),
                    }
                } else {
                    IpcResponse::Error("No robot loaded".to_string())
                }
            }
            IpcMessage::GetState => {
                if let Some(ref robot) = robot {
                    let state = robot.get_state().await;
                    IpcResponse::State(state)
                } else {
                    IpcResponse::Error("No robot loaded".to_string())
                }
            }
            IpcMessage::GetAxisCount => {
                if let Some(ref robot) = robot {
                    IpcResponse::Count(robot.axes_count())
                } else {
                    IpcResponse::Error("No robot loaded".to_string())
                }
            }
            IpcMessage::GetJointCount => {
                if let Some(ref robot) = robot {
                    IpcResponse::Count(robot.joints_count())
                } else {
                    IpcResponse::Error("No robot loaded".to_string())
                }
            }
            IpcMessage::LoadRobot(_robot_name) => {
                // Robot loading would be handled at the application level
                IpcResponse::Error("Robot loading not supported via IPC".to_string())
            }
            IpcMessage::UnloadRobot => {
                // Robot unloading would be handled at the application level
                IpcResponse::Error("Robot unloading not supported via IPC".to_string())
            }
            IpcMessage::AxisUpdate(_updates) => {
                // Axis updates would be processed by the control system
                IpcResponse::Ok
            }
            IpcMessage::JointUpdate(_updates) => {
                // Joint updates would be processed by the control system
                IpcResponse::Ok
            }
            IpcMessage::Response(_) | IpcMessage::Error(_) => {
                IpcResponse::Error("Invalid message type".to_string())
            }
        }
    }

    async fn publish_robot_state(robot: &Arc<Robot>) -> Result<()> {
        let state = robot.get_state().await;
        
        // Create state update message
        let message = IpcMessage::Response(IpcResponse::State(state));
        
        // In a real implementation, we would publish via ZMQ
        // For now, just log at debug level
        debug!("Publishing robot state update");
        
        Ok(())
    }

    pub async fn stop(mut self) -> Result<()> {
        info!("Stopping IPC server");

        // Mark as not running
        {
            let mut running = self.running.write().await;
            *running = false;
        }

        // Notify all tasks to shutdown
        self.shutdown_notify.notify_waiters();

        // Wait for all tasks to complete
        for handle in self.task_handles {
            if let Err(e) = handle.await {
                warn!("Task join error: {}", e);
            }
        }

        // Close sockets
        // Note: In a real implementation, we'd properly close ZMQ sockets here

        info!("IPC server stopped");
        Ok(())
    }

    pub async fn is_running(&self) -> bool {
        *self.running.read().await
    }

    pub async fn get_connection_count(&self) -> usize {
        // In a real implementation, we'd track active connections
        0
    }

    pub async fn send_broadcast(&self, message: &IpcMessage) -> Result<()> {
        // In a real implementation, we'd broadcast via the publisher socket
        debug!("Broadcasting message: {:?}", message);
        Ok(())
    }
}

impl Drop for IpcServer {
    fn drop(&mut self) {
        // Ensure all tasks are cleaned up
        for handle in &self.task_handles {
            handle.abort();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shared_types::*;

    #[tokio::test]
    async fn test_message_processing() {
        let robot = None;
        
        // Test GetAxisCount without robot
        let message = IpcMessage::GetAxisCount;
        let response = IpcServer::process_ipc_message(message, &robot).await;
        
        match response {
            IpcResponse::Error(msg) => assert_eq!(msg, "No robot loaded"),
            _ => panic!("Expected error response"),
        }
    }

    #[test]
    fn test_ipc_message_serialization() {
        let message = IpcMessage::Activate;
        
        // Test JSON serialization
        let json = serde_json::to_string(&message).unwrap();
        let deserialized: IpcMessage = serde_json::from_str(&json).unwrap();
        
        match deserialized {
            IpcMessage::Activate => (),
            _ => panic!("Serialization/deserialization failed"),
        }
        
        // Test binary serialization
        let binary = bincode::serialize(&message).unwrap();
        let deserialized: IpcMessage = bincode::deserialize(&binary).unwrap();
        
        match deserialized {
            IpcMessage::Activate => (),
            _ => panic!("Binary serialization/deserialization failed"),
        }
    }

    #[tokio::test]
    async fn test_ipc_buffer() {
        let message = IpcMessage::GetState;
        let buffer = IpcMessageBuffer::from_message(&message).unwrap();
        
        assert!(buffer.length > 0);
        assert!(buffer.length <= crate::shared_types::IPC_MESSAGE_SIZE);
        
        let reconstructed = buffer.to_message().unwrap();
        match reconstructed {
            IpcMessage::GetState => (),
            _ => panic!("Buffer conversion failed"),
        }
    }
}