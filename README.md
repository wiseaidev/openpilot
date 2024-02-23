# 🚗 openpilot

[![Crates.io](https://img.shields.io/crates/v/openpilot.svg)](https://crates.io/crates/openpilot)
[![docs](https://docs.rs/openpilot/badge.svg)](https://docs.rs/openpilot/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

`openpilot` is a comprehensive Rust crate designed to assist in building fully autonomous vehicles. The primary focus of this crate is to provide tools for constructing and simulating sensor models, and GPS sensors, and implementing Extended Kalman Filters (EKF). The library includes modules tailored for managing sensor readings, simple sensors, GPS sensors, and features a high-performance 1D Extended Kalman Filter (EKF) implementation.

In addition to these foundational functionalities, `openpilot` introduces the `selfdrive` package. This package is specifically designed for autonomous vehicle control, providing modules and structs that facilitate object tracking, cluster management, and detailed lead vehicle analysis.

## 🦀 Why Rust?

![I Like Yo Cut G!](https://github.com/wiseaidev/openpilot/assets/62179149/cc8c8ff4-94cb-4fa5-aab1-8cd43c3f1791)

> Python is mostly Kids Oriented Programming (KOP)!

Choosing Rust for `openpilot` is like strapping a jet engine to your autonomous vehicle's code. Rust brings a level of memory safety and concurrency control that turns potential coding nightmares into a serene drive through bug-free landscapes. With its high-performance 1D Extended Kalman Filter and modules tailored for sensor management and autonomous control, `openpilot` in Rust isn't just a toolbox; it's the James Bond of autonomous vehicle development. So, why Rust? Because when your code is navigating the streets, you want a language that won't just drive, it'll drive with the precision of a surgical robot and the reliability of a Swiss watch. Welcome to the fast lane of autonomy, where Rust is the steering wheel that ensures you reach your destination, bug-free and in style.

## ✨ Features

- **Autonomous Vehicle:** The `selfdrive` package introduces modules and structs designed for autonomous vehicle control. This includes functionalities for object tracking, cluster management, and detailed lead vehicle analysis. These tools are crucial components for building a fully autonomous system.

![Peak Auto Pilot Momentum](https://github.com/wiseaidev/openpilot/assets/62179149/26b04782-ea3d-4bd6-981e-642818810cc9)

> Look! My Honda is driving by itself!

- **Sensor Models:** Easily construct and simulate sensor models with the flexibility to customize observation models and covariance matrices.

- **Simple Sensors:** Create and utilize simple sensors for efficient data processing, allowing seamless integration into your autonomous system.

- **GPS Sensors:** Implement GPS sensors with accurate readings, supporting latitude and longitude coordinates for precise location tracking.

- **1D Extended Kalman Filter (EKF):** Leverage the fast and efficient 1D EKF implementation for robust state estimation and tracking applications.

## 🚀 Quick Start

Get started with the `openpilot` library by following these simple steps:

1. Install the `openpilot` crate by adding the following line to your `Cargo.toml` file:

```toml
[dependencies]
openpilot = "0.0.4"
```

1. Import the necessary modules and use the provided functionality in your Rust project:

```rust
use ndarray::arr2;
use openpilot::common::ext_kal_fltr::{SensorReading, SimpleSensor, GPS, EKF, FastEKF1D};
use openpilot::selfdrive::controls::radar_helpers::{Track, Cluster, Lead};

fn main() {
    // Example usage
    let data = arr2(&[[1.0, 2.0], [3.0, 4.0]]);
    let obs_model = arr2(&[[1.0, 0.0], [0.0, 1.0]]);
    let covar = arr2(&[[0.1, 0.0], [0.0, 0.1]]);
    let sensor_reading = SensorReading::new(data.clone(), obs_model.clone(), covar.clone());

    // Create a simple sensor
    let simple_sensor = SimpleSensor::new(obs_model.clone(), covar.clone(), 2);
    let reading = simple_sensor.read(data.clone(), None);
    println!("Simple Sensor Reading: {:?}", reading);

    // Create a GPS sensor
    let gps_sensor = GPS::new((0, 1), 2, 0.01);
    let latlon = &[37.7749, -122.4194];
    let reading = gps_sensor.read(latlon, None);
    println!("GPS Sensor Reading: {:?}", reading);

    // Create and use a fast 1D Extended Kalman Filter
    let mut fast_ekf_1d = FastEKF1D::new(0.1, 0.01, 0.001);
    fast_ekf_1d.update_scalar(&sensor_reading);
    fast_ekf_1d.predict(0.1);
    let (tf, tfj) = fast_ekf_1d.calc_transfer_fun(0.1);
    println!("EKF Transfer Function: {:?}, Jacobian: {:?}", tf, tfj);

    // Use Track, Cluster, and Lead structs and functions
    let mut track = Track::new();
    track.update(10.0, -5.0, 20.0, 15.0, 30.0, 0.1, 0.5, 0.5, 15.0, 20.0, 10.0);
    let key = track.get_key_for_cluster();
    println!("Track Key for Cluster: {:?}", key);

    let mut cluster = Cluster::new();
    let track1 = Track::new();
    let track2 = Track::new();
    cluster.add(track1);
    cluster.add(track2);
    let d_rel = cluster.d_rel();
    println!("Cluster Relative Distance: {:?}", d_rel);

    let mut lead = Lead {
        d_rel: 0.0,
        y_rel: 0.0,
        v_rel: 0.0,
        a_rel: 0.0,
        v_lead: 0.0,
        a_lead: 0.0,
        d_path: 0.0,
        v_lat: 0.0,
        v_lead_k: 0.0,
        a_lead_k: 0.0,
        status: false,
        fcw: true,
    };
    cluster.to_live20(&mut lead);
    println!("Lead Vehicle Information: {:?}", lead);
}
```

## 🧪 Testing

Run tests for the `openpilot` crate using:

```bash
cargo test
```

## 🌐 GitHub Repository

You can access the source code for `openpilot` on [GitHub](https://github.com/wiseaidev/openpilot).

## 🤝 Contributing

Contributions and feedback are welcome! If you'd like to contribute, report an issue, or suggest an enhancement, please engage with the project on [GitHub](https://github.com/wiseaidev/openpilot). Your contributions help improve this crate for the community.

## 💰 Support 

![My Meet Ain't Meeting](https://github.com/wiseaidev/openpilot/assets/62179149/ddb4966a-3449-4266-8e4a-7bb6914fbe4b)

If you like this project, consider tossing a coin into the virtual tip jar. Your donations help keep the coffee flowing and the code cracking. So, if you want to be the hero this project deserves, why not sprinkle some gold coins in the developer's virtual hat?

# 📘 Documentation

Full documentation for `openpilot` is available on [docs.rs](https://docs.rs/openpilot/).

# 📄 License

This project is licensed under the [MIT License](LICENSE).
