//! # openpilot
//!
//! `openpilot` is a Rust crate that provides functionalities related to autonomous driving and sensor fusion.
//! This crate includes modules for working with sensor readings, simple sensors, GPS sensors, and an implementation
//! of a 1D Extended Kalman Filter (EKF). These components are designed to be used in the development of autonomous
//! vehicle systems and sensor fusion applications.
//!
//! ## Modules
//!
//! `openpilot` is organized into several modules, each serving a specific purpose:
//!
//! - [SensorReading](common/ext_kal_fltr/struct.SensorReading.html): Represents a sensor reading with observed data, observation model,
//!   and covariance matrix.
//!
//! - [SimpleSensor](common/ext_kal_fltr/struct.SimpleSensor.html): Represents a simple sensor with an observation model and covariance matrix.
//!
//! - [GPS](common/ext_kal_fltr/struct.GPS.html): Represents a GPS sensor with specific parameters for position calculations.
//!
//! - [EKF Trait](common/ext_kal_fltr/trait.EKF.html): Defines a trait for Extended Kalman Filter (EKF) functionality.
//!
//! - [FastEKF1D](common/ext_kal_fltr/struct.FastEKF1D.html): Represents a fast 1D Extended Kalman Filter (EKF) implementation.
//!
//! ## Usage
//!
//! To use the `openpilot` crate in your project, add the following line to your `Cargo.toml` file:
//!
//! ```toml
//! [dependencies]
//! openpilot = "0.0.3"
//! ```
//!
//! Then, you can import the necessary modules and use the provided functionalities in your code.
//!
//! ## Example
//!
//! ```rust
//! use openpilot::common::ext_kal_fltr::{EKF, FastEKF1D, SensorReading};
//! use ndarray::{arr2, arr1};
//!
//! // Create a FastEKF1D instance
//! let mut fast_ekf_1d = FastEKF1D::new(0.1, 0.01, 0.001);
//!
//! // Create a sensor reading
//! let reading = SensorReading::new(arr2(&[[1.0]]), arr2(&[[0.1]]), arr2(&[[0.01]]));
//!
//! // Update the FastEKF1D with the sensor reading
//! fast_ekf_1d.update_scalar(&reading);
//!
//! // Access the updated state and covariance
//! let updated_state = fast_ekf_1d.state;
//! let updated_covar = fast_ekf_1d.covar;
//!
//! // Perform other operations as needed
//! // ...
//! ```
//!
//! ## Contributing
//!
//! Contributions and feedback are welcome! If you'd like to contribute, report an issue, or suggest an enhancement,
//! please engage with the project on [GitHub](https://github.com/wiseaidev/openpilot).
//! Your contributions help improve this crate for the community.
//!
//! ## License
//!
//! This project is licensed under the [MIT License](LICENSE).

pub mod common;
pub mod selfdrive;
