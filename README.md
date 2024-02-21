# 🚗 openpilot

[![Crates.io](https://img.shields.io/crates/v/openpilot.svg)](https://crates.io/crates/openpilot)
[![docs](https://docs.rs/openpilot/badge.svg)](https://docs.rs/openpilot/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

`openpilot` provides functionalities for building and simulating sensor models, GPS sensors, and Extended Kalman Filters (EKF) in Rust. It includes modules for representing sensor readings, simple sensors, GPS sensors, and a fast 1D Extended Kalman Filter (EKF) implementation.

## 🚀 Quick Start

Get started with the `openpilot` library by following these simple steps:

1. Install the `openpilot` crate by adding the following line to your `Cargo.toml` file:

```toml
[dependencies]
openpilot = "0.0.1"
```

1. Import the necessary modules and use the provided functionality in your Rust project:

```rust
use ndarray::{arr2, Array2, s};
use openpilot::ext_kal_fltr::{SensorReading, SimpleSensor, GPS, EKF, FastEKF1D};

// Example usage
let data = arr2(&[[1.0, 2.0], [3.0, 4.0]]);
let obs_model = arr2(&[[1.0, 0.0], [0.0, 1.0]]);
let covar = arr2(&[[0.1, 0.0], [0.0, 0.1]]);
let sensor_reading = SensorReading::new(data.clone(), obs_model.clone(), covar.clone());

// Create a simple sensor
let simple_sensor = SimpleSensor::new(obs_model.clone(), covar.clone(), 2);
let reading = simple_sensor.read(data.clone(), None);

// Create a GPS sensor
let gps_sensor = GPS::new((0, 1), 2, 0.01);
let latlon = &[37.7749, -122.4194];
let reading = gps_sensor.read(latlon, None);

// Create and use a fast 1D Extended Kalman Filter
let mut fast_ekf_1d = FastEKF1D::new(0.1, 0.01, 0.001);
fast_ekf_1d.update_scalar(&sensor_reading);
fast_ekf_1d.predict(0.1);
let (tf, tfj) = fast_ekf_1d.calc_transfer_fun(0.1);
```

## 🧪 Testing

Run tests for the `openpilot` crate using:

```bash
cargo test
```

## 🌐 GitHub Repository

You can access the source code for the `openpilot` crate on [GitHub](https://github.com/wiseaidev/openpilot).

## 🤝 Contributing

Contributions and feedback are welcome! If you'd like to contribute, report an issue, or suggest an enhancement, please engage with the project on [GitHub](https://github.com/wiseaidev/openpilot). Your contributions help improve this crate for the community.

# 📘 Documentation

Full documentation for `openpilot` is available on [docs.rs](https://docs.rs/openpilot/).

# 📄 License

This project is licensed under the [MIT License](LICENSE).
