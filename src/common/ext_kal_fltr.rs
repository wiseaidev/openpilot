use ndarray::{arr1, arr2, s, Array, Array2};

/// Represents a sensor reading, including observed data, observation model, and covariance matrix.
#[derive(Debug)]
pub struct SensorReading {
    /// Observed data associated with the sensor reading.
    pub data: Array2<f64>,
    /// Observation model matrix for the sensor reading.
    pub obs_model: Array2<f64>,
    /// Covariance matrix for the sensor reading.
    pub covar: Array2<f64>,
}

impl SensorReading {
    /// Creates a new `SensorReading` instance.
    ///
    /// # Arguments
    ///
    /// * `data` - Observed data associated with the sensor reading.
    /// * `obs_model` - Observation model matrix for the sensor reading.
    /// * `covar` - Covariance matrix for the sensor reading.
    ///
    /// # Returns
    ///
    /// (`SensorReading`): A new `SensorReading` instance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use ndarray::{arr2, Array2};
    /// use openpilot::common::ext_kal_fltr::SensorReading;
    ///
    /// let data = arr2(&[[1.0, 2.0], [3.0, 4.0]]);
    /// let obs_model = arr2(&[[1.0, 0.0], [0.0, 1.0]]);
    /// let covar = arr2(&[[0.1, 0.0], [0.0, 0.1]]);
    /// let sensor_reading = SensorReading::new(data, obs_model.clone(), covar.clone());
    ///
    /// assert_eq!(obs_model.shape(), covar.clone().shape(), "Observation model and covariance matrix dimensions do not match");
    /// ```
    pub fn new(data: Array2<f64>, obs_model: Array2<f64>, covar: Array2<f64>) -> Self {
        assert!(
            obs_model.shape() == covar.shape(),
            "Observation model and covariance matrix dimensions do not match"
        );
        SensorReading {
            data,
            obs_model,
            covar,
        }
    }
}

/// Represents a simple sensor with an observation model and covariance matrix.
#[derive(Debug)]
pub struct SimpleSensor {
    /// Observation model matrix for the simple sensor.
    pub obs_model: Array2<f64>,
    /// Covariance matrix for the simple sensor.
    pub covar: Array2<f64>,
}

impl SimpleSensor {
    /// Creates a new `SimpleSensor` instance.
    ///
    /// # Arguments
    ///
    /// * `obs_model` - Observation model matrix for the simple sensor.
    /// * `covar` - Covariance matrix for the simple sensor.
    /// * `dims` - Number of dimensions for the observation model.
    ///
    /// # Returns
    ///
    /// (`SimpleSensor`): A new `SimpleSensor` instance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use ndarray::{arr2, Array2};
    /// use openpilot::common::ext_kal_fltr::SimpleSensor;
    ///
    /// let obs_model = arr2(&[[1.0, 0.0], [0.0, 1.0]]);
    /// let covar = arr2(&[[0.1, 0.0], [0.0, 0.1]]);
    /// let dims = 2;
    /// let simple_sensor = SimpleSensor::new(obs_model, covar, dims);
    /// ```
    pub fn new(obs_model: Array2<f64>, covar: Array2<f64>, dims: usize) -> Self {
        let obs_model = if dims == obs_model.shape()[1] {
            obs_model
        } else {
            let obs_model = Array2::zeros((obs_model.shape()[0], dims));
            obs_model
                .clone()
                .slice_mut(s![.., obs_model.shape()[1]..])
                .assign(&Array2::eye(obs_model.shape()[0]));
            obs_model
        };

        let covar = if covar.shape() == [1, 1] {
            Array2::eye(dims) * covar[[0, 0]]
        } else if covar.shape() == [dims, dims] {
            covar
        } else {
            Array2::eye(obs_model.shape()[0]) * covar[[0, 0]]
        };

        SimpleSensor { obs_model, covar }
    }

    /// Reads data from the sensor and returns a `SensorReading` instance.
    ///
    /// # Arguments
    ///
    /// * `data` - Observed data from the sensor.
    /// * `covar` - Optional covariance matrix. If not provided, the default covariance of the sensor is used.
    ///
    /// # Returns
    ///
    /// (`SensorReading`): A new `SensorReading` instance based on the sensor's data and covariance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use ndarray::{arr2, Array2};
    /// use openpilot::common::ext_kal_fltr::{SimpleSensor, SensorReading};
    ///
    /// let obs_model = arr2(&[[1.0, 0.0], [0.0, 1.0]]);
    /// let covar = arr2(&[[0.1, 0.0], [0.0, 0.1]]);
    /// let dims = 2;
    /// let simple_sensor = SimpleSensor::new(obs_model, covar, dims);
    /// let data = arr2(&[[1.0, 2.0]]);
    /// let sensor_reading = simple_sensor.read(data, None);
    /// ```
    pub fn read(&self, data: Array2<f64>, covar: Option<Array2<f64>>) -> SensorReading {
        let covar = covar.unwrap_or_else(|| self.covar.clone());
        SensorReading::new(data, self.obs_model.clone(), covar)
    }
}

/// Represents a GPS sensor with specific parameters.
#[derive(Debug)]
pub struct GPS {
    /// Earth radius constant for distance calculations.
    pub earth_r: f64,
    /// Observation model matrix for the GPS sensor.
    pub obs_model: Array2<f64>,
    /// Covariance matrix for the GPS sensor.
    pub covar: Array2<f64>,
    /// Initial latitude for position calculations.
    pub init_lat: f64,
    /// Initial longitude for position calculations.
    pub init_lon: f64,
}

impl GPS {
    /// Creates a new `GPS` instance with specified parameters.
    ///
    /// # Arguments
    ///
    /// * `xy_idx` - Tuple representing the indices of X and Y coordinates in the observation model matrix.
    /// * `dims` - Number of dimensions for the observation model.
    /// * `var` - Variance for the covariance matrix.
    ///
    /// # Returns
    ///
    /// (`GPS`): A new `GPS` instance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::common::ext_kal_fltr::GPS;
    ///
    /// let xy_idx = (0, 1);
    /// let dims = 2;
    /// let var = 1e4;
    /// let gps = GPS::new(xy_idx, dims, var);
    /// ```
    pub fn new(xy_idx: (usize, usize), dims: usize, var: f64) -> Self {
        let mut obs_model = Array2::zeros((2, dims));
        obs_model
            .slice_mut(s![.., xy_idx.0])
            .assign(&Array2::eye(2).column(0));

        let covar = Array2::eye(2) * var;
        GPS {
            earth_r: 6371e3,
            obs_model,
            covar,
            init_lat: 0.0,
            init_lon: 0.0,
        }
    }

    /// Initializes the GPS sensor with the given latitude and longitude.
    ///
    /// # Arguments
    ///
    /// * `latlon` - Array containing latitude and longitude values.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::common::ext_kal_fltr::GPS;
    ///
    /// let mut gps = GPS::new((0, 1), 2, 1e4);
    /// gps.init_pos(&[37.7749, -122.4194]);
    /// ```
    pub fn init_pos(&mut self, latlon: &[f64]) {
        self.init_lat = latlon[0].to_radians();
        self.init_lon = latlon[1].to_radians();
    }

    /// Calculates the Haversine distance between two points on the Earth's surface.
    ///
    /// # Arguments
    ///
    /// * `lat1` - Latitude of the first point.
    /// * `lon1` - Longitude of the first point.
    /// * `lat2` - Latitude of the second point.
    /// * `lon2` - Longitude of the second point.
    ///
    /// # Returns
    ///
    /// (`f64`): Haversine distance between the two points.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::common::ext_kal_fltr::GPS;
    ///
    /// let gps = GPS::new((0, 1), 2, 1e4);
    /// let distance = gps._haversine(37.7749, -122.4194, 38.0000, -121.0000);
    /// ```
    pub fn _haversine(&self, lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
        let lat_diff = lat2 - lat1;
        let lon_diff = lon2 - lon1;
        let d = (lat_diff / 2.0).sin().powi(2)
            + lat1.cos() * lat2.cos() * (lon_diff / 2.0).sin().powi(2);
        2.0 * self.earth_r * d.sqrt().asin()
    }

    /// Converts latitude and longitude coordinates to meters.
    ///
    /// # Arguments
    ///
    /// * `lat` - Latitude coordinate.
    /// * `lon` - Longitude coordinate.
    ///
    /// # Returns
    ///
    /// (`(f64, f64)`): Tuple containing converted X and Y coordinates in meters.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::common::ext_kal_fltr::GPS;
    ///
    /// let gps = GPS::new((0, 1), 2, 1e4);
    /// let (xs, ys) = gps.convert_deg2m(37.7749, -122.4194);
    /// ```
    pub fn convert_deg2m(&self, lat: f64, lon: f64) -> (f64, f64) {
        let lat_rad = lat.to_radians();
        let lon_rad = lon.to_radians();

        let xs = (lon_rad - self.init_lon) * lat_rad.cos() * self.earth_r;
        let ys = (lat_rad - self.init_lat) * self.earth_r;

        (xs, ys)
    }

    /// Converts X and Y coordinates in meters to latitude and longitude coordinates.
    ///
    /// # Arguments
    ///
    /// * `xs` - X coordinate in meters.
    /// * `ys` - Y coordinate in meters.
    ///
    /// # Returns
    ///
    /// (`(f64, f64)`): Tuple containing converted latitude and longitude coordinates.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::common::ext_kal_fltr::GPS;
    ///
    /// let gps = GPS::new((0, 1), 2, 1e4);
    /// let (lat, lon) = gps._convert_m2deg(100.0, 200.0);
    /// ```
    pub fn _convert_m2deg(&self, xs: f64, ys: f64) -> (f64, f64) {
        let lat = ys / self.earth_r + self.init_lat;
        let lon = xs / (self.earth_r * self.init_lat.cos()) + self.init_lon;

        (lat.to_degrees(), lon.to_degrees())
    }

    /// Reads GPS data and returns a `SensorReading` instance.
    ///
    /// # Arguments
    ///
    /// * `latlon` - Array containing latitude and longitude values.
    /// * `accuracy` - Optional accuracy value. If not provided, the default covariance of the sensor is used.
    ///
    /// # Returns
    ///
    /// (`SensorReading`): A new `SensorReading` instance based on GPS data and covariance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::common::ext_kal_fltr::GPS;
    ///
    /// let mut gps = GPS::new((0, 1), 2, 1e4);
    /// gps.init_pos(&[37.7749, -122.4194]);
    /// let sensor_reading = gps.read(&[37.7749, -122.4194], Some(10.0));
    /// ```
    pub fn read(&self, latlon: &[f64], accuracy: Option<f64>) -> SensorReading {
        let (x_dist, y_dist) = self.convert_deg2m(latlon[0], latlon[1]);
        let covar = match accuracy {
            Some(acc) => Array2::eye(2) * acc.powi(2),
            None => self.covar.clone(),
        };

        SensorReading::new(
            arr2(&[[x_dist], [y_dist]]).reversed_axes(),
            covar,
            self.obs_model.clone(),
        )
    }
}

/// Represents a trait for Extended Kalman Filter (EKF) functionality.
pub trait EKF {
    /// Updates the EKF with the given sensor reading.
    fn update(&mut self, reading: &SensorReading);

    /// Updates a scalar EKF with the given sensor reading.
    fn update_scalar(&mut self, reading: &SensorReading);

    /// Predicts the state of the EKF for the given time step.
    fn predict(&mut self, dt: f64);

    /// Calculates the transfer function matrices for the given time step.
    fn calc_transfer_fun(&self, dt: f64) -> (Array2<f64>, Array2<f64>);
}

/// Represents a fast 1D Extended Kalman Filter (EKF) implementation.
#[derive(Debug)]
pub struct FastEKF1D {
    /// State vector of the EKF.
    pub state: Array<f64, ndarray::Ix1>,
    /// Covariance vector of the EKF.
    pub covar: Array<f64, ndarray::Ix1>,
    /// Product of time step and process noise variance for state transition.
    pub dt_q0: f64,
    /// Product of time step and process noise variance for covariance update.
    pub dt_q1: f64,
}

impl FastEKF1D {
    /// Creates a new `FastEKF1D` instance.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step for prediction.
    /// * `var_init` - Initial variance for the state vector.
    /// * `q` - Process noise variance for state transition.
    ///
    /// # Returns
    ///
    /// (`FastEKF1D`): A new `FastEKF1D` instance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::common::ext_kal_fltr::FastEKF1D;
    ///
    /// let ekf = FastEKF1D::new(0.1, 1.0, 0.1);
    /// ```
    pub fn new(dt: f64, var_init: f64, q: f64) -> Self {
        let state = arr1(&[0.0, 0.0]);
        let covar = arr1(&[var_init, var_init, 0.0]);

        FastEKF1D {
            state,
            covar,
            dt_q0: dt * q,
            dt_q1: dt * q,
        }
    }
}

impl EKF for FastEKF1D {
    /// Updates the 1D EKF with the given sensor reading (not implemented).
    fn update(&mut self, _reading: &SensorReading) {
        unimplemented!();
    }

    /// Updates the 1D EKF with the given scalar sensor reading.
    ///
    /// # Arguments
    ///
    /// * `reading` - Scalar sensor reading containing observed data and covariance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::common::ext_kal_fltr::{FastEKF1D, SensorReading, EKF};
    /// use ndarray::arr2;
    ///
    /// let mut ekf = FastEKF1D::new(0.1, 1.0, 0.1);
    /// let reading = SensorReading::new(arr2(&[[1.0]]), arr2(&[[0.1]]), arr2(&[[0.01]]));
    /// ekf.update_scalar(&reading);
    /// ```
    fn update_scalar(&mut self, reading: &SensorReading) {
        let rcov = reading.covar[[0, 0]];

        let mut x = self.state.clone();
        let mut s = self.covar.clone();

        let innovation = *reading.data.get((0, 0)).unwrap() - x[0];
        let innovation_covar = s[0] + rcov;

        let k0 = s[0] / innovation_covar;
        let k1 = s[2] / innovation_covar;

        x[0] += k0 * innovation;
        x[1] += k1 * innovation;

        let mk = 1.0 - k0;
        s[1] += k1 * (k1 * (s[0] + rcov) - 2.0 * s[2]);
        s[2] = mk * (s[2] - k1 * s[0]) + rcov * k0 * k1;
        s[0] = mk * mk * s[0] + rcov * k0 * k0;

        self.state = x;
        self.covar = s;
    }

    /// Predicts the state of the 1D EKF for the given time step.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step for prediction.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::common::ext_kal_fltr::{EKF, FastEKF1D};
    ///
    /// let mut ekf = FastEKF1D::new(0.1, 1.0, 0.1);
    /// ekf.predict(0.1);
    /// ```
    fn predict(&mut self, dt: f64) {
        let mut x = self.state.clone();

        x[0] += dt * x[1];

        let mut s = self.covar.clone();
        s[0] += dt * (2.0 * s[2] + dt * s[1]) + self.dt_q0;
        s[2] += dt * s[1];
        s[1] += self.dt_q1;

        s.mapv_inplace(|val| val.max(-1e10).min(1e10));

        self.state = x;
        self.covar = s;
    }

    /// Calculates the transfer function matrices for the 1D EKF for the given time step.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step for prediction.
    ///
    /// # Returns
    ///
    /// (`(Array2<f64>, Array2<f64>)`): Tuple containing the state transition matrix and Jacobian matrix.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::common::ext_kal_fltr::{FastEKF1D, EKF};
    /// use ndarray::Array2;
    ///
    /// let ekf = FastEKF1D::new(0.1, 1.0, 0.1);
    /// let (tf, tfj) = ekf.calc_transfer_fun(0.1);
    /// ```
    fn calc_transfer_fun(&self, dt: f64) -> (Array2<f64>, Array2<f64>) {
        let mut tf = Array2::eye(2);
        tf[[0, 1]] = dt;

        let tfj = tf.clone();
        (tf, tfj)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_sensor_reading_creation() {
        let data = arr2(&[[1.0, 2.0], [3.0, 4.0]]);
        let obs_model = arr2(&[[1.0, 0.0], [0.0, 1.0]]);
        let covar = arr2(&[[0.1, 0.0], [0.0, 0.1]]);
        let sensor_reading = SensorReading::new(data.clone(), obs_model.clone(), covar.clone());

        assert_eq!(sensor_reading.data, data);
        assert_eq!(sensor_reading.obs_model, obs_model);
        assert_eq!(sensor_reading.covar, covar);
    }

    #[test]
    #[should_panic(expected = "Observation model and covariance matrix dimensions do not match")]
    fn test_sensor_reading_creation_panics_on_mismatched_dimensions() {
        let data = arr2(&[[1.0, 2.0], [3.0, 4.0]]);
        let obs_model = arr2(&[[1.0, 0.0]]);
        let covar = arr2(&[[0.1, 0.0], [0.0, 0.1]]);
        SensorReading::new(data, obs_model, covar);
    }

    #[test]
    fn test_simple_sensor_creation() {
        let obs_model = arr2(&[[1.0, 0.0], [0.0, 1.0]]);
        let covar = arr2(&[[0.1, 0.0], [0.0, 0.1]]);
        let simple_sensor = SimpleSensor::new(obs_model.clone(), covar.clone(), 2);

        assert_eq!(simple_sensor.obs_model, obs_model);
        assert_eq!(simple_sensor.covar, covar);
    }

    #[test]
    fn test_simple_sensor_read() {
        let obs_model = arr2(&[[1.0, 0.0], [0.0, 1.0]]);
        let covar = arr2(&[[0.1, 0.0], [0.0, 0.1]]);
        let simple_sensor = SimpleSensor::new(obs_model.clone(), covar.clone(), 2);
        let data = arr2(&[[1.0], [2.0]]);
        let sensor_reading = simple_sensor.read(data.clone(), None);

        assert_eq!(sensor_reading.data, data);
        assert_eq!(sensor_reading.obs_model, obs_model);
        assert_eq!(sensor_reading.covar, covar);
    }

    #[test]
    fn test_gps_creation() {
        let obs_model = arr2(&[[1.0, 0.0], [0.0, 0.0]]);
        let covar = arr2(&[[0.1, 0.0], [0.0, 0.1]]);
        let gps = GPS::new((0, 1), 2, 0.1);

        assert_eq!(gps.obs_model, obs_model);
        assert_eq!(gps.covar, covar);
        assert_eq!(gps.earth_r, 6371e3);
        assert_eq!(gps.init_lat, 0.0);
        assert_eq!(gps.init_lon, 0.0);
    }

    #[test]
    fn test_gps_read() {
        let obs_model = arr2(&[[0.01, 0.0], [0.0, 0.01]]);
        let covar = arr2(&[[1.0, 0.0], [0.0, 0.0]]);
        let mut gps = GPS::new((0, 1), 2, 0.1);
        gps.init_pos(&[0.0, 0.0]);

        let latlon = [1.0, 2.0];
        let sensor_reading = gps.read(&latlon, Some(0.1));

        let (x_dist, y_dist) = gps.convert_deg2m(latlon[0], latlon[1]);
        let expected_data = arr2(&[[x_dist], [y_dist]]).reversed_axes();

        assert_eq!(sensor_reading.data, expected_data);
        assert_abs_diff_eq!(
            sensor_reading.obs_model[[0, 0]],
            obs_model[[0, 0]],
            epsilon = 1e-4
        );
        assert_abs_diff_eq!(
            sensor_reading.obs_model[[0, 1]],
            obs_model[[0, 1]],
            epsilon = 1e-4
        );
        assert_abs_diff_eq!(
            sensor_reading.obs_model[[1, 0]],
            obs_model[[1, 0]],
            epsilon = 1e-4
        );
        assert_abs_diff_eq!(
            sensor_reading.obs_model[[1, 1]],
            obs_model[[1, 1]],
            epsilon = 1e-4
        );
        assert_eq!(sensor_reading.covar, covar);
    }

    #[test]
    fn test_fast_ekf_1d_creation() {
        let fast_ekf_1d = FastEKF1D::new(0.1, 0.01, 0.001);

        assert_eq!(fast_ekf_1d.state, arr1(&[0.0, 0.0]));
        assert_eq!(fast_ekf_1d.covar, arr1(&[0.01, 0.01, 0.0]));
        assert_eq!(fast_ekf_1d.dt_q0, 0.0001);
        assert_eq!(fast_ekf_1d.dt_q1, 0.0001);
    }

    #[test]
    fn test_fast_ekf_1d_update_scalar() {
        let mut fast_ekf_1d = FastEKF1D::new(0.1, 0.01, 0.001);
        let reading = SensorReading::new(arr2(&[[1.0]]), arr2(&[[0.1]]), arr2(&[[0.01]]));

        fast_ekf_1d.update_scalar(&reading);

        assert_abs_diff_eq!(fast_ekf_1d.state[0], 0.5, epsilon = 1e-6);
        assert_abs_diff_eq!(fast_ekf_1d.state[1], 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(fast_ekf_1d.covar[0], 0.005, epsilon = 1e-6);
        assert_abs_diff_eq!(fast_ekf_1d.covar[1], 0.01, epsilon = 1e-6);
        assert_abs_diff_eq!(fast_ekf_1d.covar[2], 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_fast_ekf_1d_predict() {
        let mut fast_ekf_1d = FastEKF1D::new(0.1, 0.01, 0.001);

        fast_ekf_1d.predict(0.1);

        assert_abs_diff_eq!(fast_ekf_1d.state[0], 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(fast_ekf_1d.state[1], 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(fast_ekf_1d.covar[0], 0.0102, epsilon = 1e-6);
    }
    #[test]
    fn test_fast_ekf_1d_calc_transfer_fun() {
        let fast_ekf_1d = FastEKF1D::new(0.1, 0.01, 0.001);
        let (tf, tfj) = fast_ekf_1d.calc_transfer_fun(0.1);

        assert_eq!(tf, arr2(&[[1.0, 0.1], [0.0, 1.0]]));
        assert_eq!(tfj, arr2(&[[1.0, 0.1], [0.0, 1.0]]));
    }
}
