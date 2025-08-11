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

use iir_filters::filter_design::FilterType;
use iir_filters::filter_design::butter;
use iir_filters::sos::zpk2sos;
use iir_filters::filter::DirectForm2Transposed;
use iir_filters::filter::Filter;

#[derive(Serialize, Deserialize, Debug)]
pub struct InputSourceConfig {
    implementation:String,
    #[serde(default)]
    config:String,
    #[serde(default)]
    channel:u32
}

#[derive(Serialize, Deserialize, Debug)]
pub struct FilterConfig {
    #[serde(default)]
    min_frequency:f64,
    #[serde(default)]
    max_frequency:f64
}

#[derive(Serialize, Deserialize, Debug)]
pub struct InputConfig {
    interface:InputSourceConfig,
    signal_processing:FilterConfig
}

pub enum InputState {OFFSET=0, CALIBRATION=1, MEASUREMENT=2}

pub struct Input {
    //plugin
    pub device_id:i32,
    pub channel:u32,
    pub buffer:Vec<f64>,
    pub value:f64,
    pub filter:DirectForm2Transposed,
}

impl Input {
    pub fn load(config:InputConfig) -> Result<Self> {
        let _ = config.interface.implementation;

        let cutoff_low = config.signal_processing.min_frequency;
        let cutoff_high = config.signal_processing.max_frequency;
        let filter_type = if cutoff_high > 0.0 { 
            FilterType::BandPass(cutoff_low, cutoff_high) 
        } else { 
            FilterType::LowPass(cutoff_low) 
        };
        let zpk = butter(2, filter_type,1.0)?;
        let sos = zpk2sos(&zpk, None)?;

        return Input{ 
            device_id: 1, 
            channel: config.interface.channel, 
            buffer: vec![0.0;10], 
            value: 0.0, 
            filter: DirectForm2Transposed::new(&sos) 
        };
    }

    pub fn update(&self) -> f64 {
        for sample in self.buffer {
            self.value = self.filter.filter(sample);
        }
    }

    pub fn has_error(&self) -> bool {
        return false;
    }

    pub fn reset(&self) {
        for i in 0..self.buffer.len() {
            self.buffer[i] = 0.0;
            self.filter.filter(0.0);
        }
    }

    pub fn set_state(&self, value:InputState) {

    }
}