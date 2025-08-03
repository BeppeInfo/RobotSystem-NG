////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2025 BeppeInfo                                             //
//                                                                            //
//  This file is part of RobotSystem-NG.                                     //
//                                                                            //
//  RobotSystem-NG is free software: you can redistribute it and/or modify   //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobotSystem-NG is distributed in the hope that it will be useful,        //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License //
//  along with RobotSystem-NG. If not, see <http://www.gnu.org/licenses/>.   //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Created: 2025-08-03 00:24:08 UTC                                         //
//  Author: BeppeInfo                                                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

//! RobotSystem-NG - A modern Rust implementation of RobotSystem-Lite
//! 
//! This library provides a robust framework for robotic rehabilitation systems,
//! focusing on real-time control, safety, and comprehensive data collection.

pub mod robot;
pub mod control;
pub mod sensors;
pub mod telemetry;
pub mod network;
pub mod session;
pub mod communications;

pub use robot::Robot;
pub use control::Controller;