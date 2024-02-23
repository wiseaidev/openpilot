use ndarray::prelude::*;

/// Calculates the curvature based on ego speed, steering angle, vehicle parameters, and an optional angle offset.
///
/// # Arguments
///
/// * `v_ego` - Ego vehicle speed.
/// * `angle_steers` - Steering angle.
/// * `vp` - Vehicle parameters.
/// * `angle_offset` - Optional angle offset.
///
/// # Returns
///
/// The calculated curvature.
///
/// # Examples
///
/// ```rust
/// use openpilot::selfdrive::controls::lat_control::{VP, calc_curvature};
///
/// let v_ego = 20.0;
/// let angle_steers = 10.0;
/// let vp = VP { steer_ratio: 15.0, wheelbase: 2.5, slip_factor: 0.02 };
/// let angle_offset = 2.0;
///
/// let curvature = calc_curvature(v_ego, angle_steers, &vp, angle_offset);
/// assert!(curvature > 0.0);
/// ```
pub fn calc_curvature(v_ego: f64, angle_steers: f64, vp: &VP, angle_offset: f64) -> f64 {
    let deg_to_rad = std::f64::consts::PI / 180.0;
    let angle_steers_rad = (angle_steers - angle_offset) * deg_to_rad;
    angle_steers_rad / (vp.steer_ratio * vp.wheelbase * (1. + vp.slip_factor * v_ego.powi(2)))
}

/// Calculates the lookahead distance based on ego speed.
///
/// # Arguments
///
/// * `v_ego` - Ego vehicle speed.
///
/// # Returns
///
/// The calculated lookahead distance.
///
/// # Examples
///
/// ```rust
/// use openpilot::selfdrive::controls::lat_control::calc_d_lookahead;
///
/// let v_ego = 25.0;
///
/// let d_lookahead = calc_d_lookahead(v_ego);
/// assert!(d_lookahead > 0.0);
/// ```
pub fn calc_d_lookahead(v_ego: f64) -> f64 {
    let offset_lookahead = 1.0;
    let coeff_lookahead = 4.4;
    offset_lookahead + (v_ego.max(0.0)).sqrt() * coeff_lookahead
}

/// Calculates the lateral offset and curvature given the ego speed, steering angle, lookahead distance, vehicle parameters, and angle offset.
///
/// # Arguments
///
/// * `v_ego` - Ego vehicle speed.
/// * `angle_steers` - Steering angle.
/// * `d_lookahead` - Lookahead distance.
/// * `vp` - Vehicle parameters.
/// * `angle_offset` - Angle offset.
///
/// # Returns
///
/// A tuple containing the lateral offset and curvature.
///
/// # Examples
///
/// ```rust
/// use openpilot::selfdrive::controls::lat_control::{calc_lookahead_offset, VP};
///
/// let v_ego = 30.0;
/// let angle_steers = 5.0;
/// let d_lookahead = 15.0;
/// let vp = VP { steer_ratio: 12.0, wheelbase: 2.7, slip_factor: 0.01 };
/// let angle_offset = 1.0;
///
/// let (y_actual, curvature) = calc_lookahead_offset(v_ego, angle_steers, d_lookahead, &vp, angle_offset);
/// assert!(y_actual > 0.0 && curvature > 0.0);
/// ```
pub fn calc_lookahead_offset(
    v_ego: f64,
    angle_steers: f64,
    d_lookahead: f64,
    vp: &VP,
    angle_offset: f64,
) -> (f64, f64) {
    let curvature = calc_curvature(v_ego, angle_steers, vp, angle_offset);
    let y_actual = d_lookahead * (d_lookahead * curvature).asin().tan() / 2.0;
    (y_actual, curvature)
}

/// Performs lateral control using a PID controller and returns the updated steering output.
///
/// # Arguments
///
/// * `v_ego` - Ego vehicle speed.
/// * `y_actual` - Actual lateral offset.
/// * `y_des` - Desired lateral offset.
/// * `ui_steer` - Integral term of the controller.
/// * `steer_max` - Maximum steering angle.
/// * `steer_override` - Indicates if steering override is active.
/// * `sat_count` - Counter for lateral control saturation.
/// * `enabled` - Indicates if lateral control is enabled.
/// * `half_pid` - Indicates whether to use full or half PID gains.
/// * `rate` - Control update rate.
///
/// # Returns
///
/// A tuple containing the updated steering output, proportional term, integral term, lateral control saturation status, saturation count, and a flag indicating continuous saturation.
///
/// # Examples
///
/// ```rust
/// use openpilot::selfdrive::controls::lat_control::{pid_lateral_control, CS};
///
/// let v_ego = 15.0;
/// let y_actual = 0.5;
/// let y_des = 0.0;
/// let ui_steer = 0.2;
/// let steer_max = 1.0;
/// let steer_override = false;
/// let sat_count = 0.0;
/// let enabled = true;
/// let half_pid = false;
/// let rate = 50.0;
///
/// let (output_steer, _, _, lateral_control_sat, _, _) = pid_lateral_control(v_ego, y_actual, y_des, ui_steer, steer_max, steer_override, sat_count, enabled, half_pid, rate);
/// assert!(output_steer <= steer_max && output_steer >= -steer_max && !lateral_control_sat);
/// ```
#[allow(clippy::too_many_arguments)]
pub fn pid_lateral_control(
    v_ego: f64,
    y_actual: f64,
    y_des: f64,
    ui_steer: f64,
    steer_max: f64,
    steer_override: bool,
    sat_count: f64,
    enabled: bool,
    half_pid: bool,
    rate: f64,
) -> (f64, f64, f64, bool, f64, bool) {
    let sat_count_rate = 1.0 / rate;
    let sat_count_limit = 0.8;

    let error_steer = y_des - y_actual;
    let ui_unwind_speed = 0.3 / rate;

    let (kp, ki) = if !half_pid { (12.0, 1.0) } else { (6.0, 0.5) };

    let up_steer = error_steer * kp;
    let ui_steer_new = ui_steer + error_steer * ki * 1.0 / rate;
    let output_steer_new = ui_steer_new + up_steer;

    let updated_ui_steer = if (error_steer >= 0. && (output_steer_new < steer_max || ui_steer < 0.))
        || (error_steer <= 0. && (output_steer_new > -steer_max || ui_steer > 0.))
            && !steer_override
    {
        // Update the integrator
        ui_steer_new
    } else {
        // Unwind integrator if driver is maneuvering the steering wheel
        if steer_override {
            ui_steer - ui_unwind_speed * ui_steer.signum()
        } else {
            ui_steer
        }
    };

    // Ensure that the integral term does not exceed the specified limits
    let clamped_ui_steer = updated_ui_steer.max(-steer_max).min(steer_max);

    // Calculate the final steering output
    let output_steer = up_steer + clamped_ui_steer;

    // Do not run steer control if at very low speed or if not enabled
    let final_ui_steer = if v_ego < 0.3 || !enabled {
        0.0
    } else {
        clamped_ui_steer
    };

    // Check if lateral control is saturated
    let lateral_control_sat = output_steer > steer_max;

    // Clip the final steering output within the specified limits
    let final_output_steer = output_steer.max(-steer_max).min(steer_max);

    // If lateral control is saturated for a certain period, set an alert for taking control of the car
    let updated_sat_count =
        if lateral_control_sat && !steer_override && v_ego > 10.0 && error_steer.abs() > 0.1 {
            sat_count + sat_count_rate
        } else {
            sat_count - sat_count_rate
        };

    // Ensure that the saturation count is within the specified limits
    let clamped_sat_count = updated_sat_count.max(0.0).min(1.0);

    // Set a flag indicating continuous saturation
    let sat_flag = clamped_sat_count >= sat_count_limit;

    (
        final_output_steer,
        up_steer,
        final_ui_steer,
        lateral_control_sat,
        clamped_sat_count,
        sat_flag,
    )
}

/// Vehicle parameters struct.
#[derive(Debug, Clone)]
pub struct VP {
    /// Steering ratio of the vehicle.
    pub steer_ratio: f64,
    /// Wheelbase of the vehicle.
    pub wheelbase: f64,
    /// Slip factor of the vehicle.
    pub slip_factor: f64,
}

/// Control state struct.
#[derive(Debug, Clone)]
pub struct CS {
    /// Ego vehicle speed.
    pub v_ego: f64,
    /// Steering angle of the vehicle.
    pub angle_steers: f64,
    /// Indicates if torque modification is active.
    pub torque_mod: bool,
    /// Indicates if steering override is active.
    pub steer_override: bool,
    /// Vehicle parameters.
    pub vp: VP,
}

/// Represents lateral control logic.
pub struct LatControl {
    /// Integral term of lateral control.
    pub ui_steer: f64,
    /// Saturation count for lateral control.
    pub sat_count: f64,
    /// Desired lateral offset.
    pub y_des: f64,
    /// Indicates if lateral control is saturated.
    pub lateral_control_sat: bool,
}

impl LatControl {
    /// Creates a new `LatControl` instance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::lat_control::LatControl;
    /// let mut lat_control = LatControl::new();
    /// ```
    pub fn new() -> Self {
        LatControl {
            ui_steer: 0.0,
            sat_count: 0.0,
            y_des: 0.0,
            lateral_control_sat: false,
        }
    }

    /// Resets the integral term of the lateral control.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::lat_control::LatControl;
    ///
    /// let mut lat_control = LatControl::new();
    /// lat_control.ui_steer = 0.5;
    /// lat_control.reset();
    /// assert_eq!(lat_control.ui_steer, 0.0);
    /// ```
    pub fn reset(&mut self) {
        self.ui_steer = 0.0;
    }

    /// Updates lateral control based on the control state, polynomial, and angle offset.
    ///
    /// # Arguments
    ///
    /// * `enabled` - Indicates if lateral control is enabled.
    /// * `CS` - Control state.
    /// * `d_poly` - Polynomial coefficients for lateral control.
    /// * `angle_offset` - Angle offset.
    ///
    /// # Returns
    ///
    /// A tuple containing the final steering output and a flag indicating continuous lateral control saturation.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::lat_control::{LatControl, CS, VP};
    /// use ndarray::prelude::*;
    ///
    /// let mut lat_control = LatControl::new();
    /// lat_control.ui_steer = 0.2;
    ///
    /// let cs = CS {
    ///     v_ego: 20.0,
    ///     angle_steers: 5.0,
    ///     torque_mod: false,
    ///     steer_override: false,
    ///     vp: VP {
    ///         steer_ratio: 12.0,
    ///         wheelbase: 2.7,
    ///         slip_factor: 0.01,
    ///     },
    /// };
    ///
    /// let d_poly = Array1::from(vec![0.1, 0.05, -0.02]);
    /// let angle_offset = 1.0;
    ///
    /// let (final_steer, sat_flag) = lat_control.update(true, &cs, &d_poly, angle_offset);
    ///
    /// assert!(final_steer <= 1.0 && final_steer >= -1.0); // Asserting final steering within limits
    /// assert!(sat_flag == false); // Asserting no continuous lateral control saturation
    /// ```
    pub fn update(
        &mut self,
        enabled: bool,
        cs: &CS,
        d_poly: &Array1<f64>,
        angle_offset: f64,
    ) -> (f64, bool) {
        let rate = 100.0;
        let steer_max = 1.0;

        // how far we look ahead is a function of speed
        let d_lookahead = calc_d_lookahead(cs.v_ego);

        // calculate actual offset at the lookahead point
        let (y_actual, _) =
            calc_lookahead_offset(cs.v_ego, cs.angle_steers, d_lookahead, &cs.vp, angle_offset);

        // desired lookahead offset
        self.y_des = d_poly.iter().enumerate().fold(0.0, |acc, (i, &coeff)| {
            acc + coeff * d_lookahead.powi(i as i32)
        });

        let (output_steer, _, ui_steer, lateral_control_sat, sat_count, sat_flag) =
            pid_lateral_control(
                cs.v_ego,
                y_actual,
                self.y_des,
                self.ui_steer,
                steer_max,
                cs.steer_override,
                self.sat_count,
                enabled,
                cs.torque_mod,
                rate,
            );

        self.ui_steer = ui_steer;
        self.sat_count = sat_count;
        self.lateral_control_sat = lateral_control_sat;

        (output_steer, sat_flag)
    }
}

impl Default for LatControl {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_calc_curvature() {
        let v_ego = 10.0;
        let angle_steers = 5.0;
        let vp = VP {
            steer_ratio: 12.0,
            wheelbase: 2.7,
            slip_factor: 0.01,
        };

        let curvature = calc_curvature(v_ego, angle_steers, &vp, 0.0);
        assert_abs_diff_eq!(curvature, 0.0013, epsilon = 1e-4);
    }

    #[test]
    fn test_calc_d_lookahead() {
        let v_ego = 20.0;
        let d_lookahead = calc_d_lookahead(v_ego);
        assert_abs_diff_eq!(d_lookahead, 20.6773, epsilon = 1e-4);
    }

    #[test]
    fn test_calc_lookahead_offset() {
        let v_ego = 20.0;
        let angle_steers = 5.0;
        let d_lookahead = 2.1;
        let vp = VP {
            steer_ratio: 12.0,
            wheelbase: 2.7,
            slip_factor: 0.01,
        };

        let (y_actual, curvature) =
            calc_lookahead_offset(v_ego, angle_steers, d_lookahead, &vp, 0.0);
        assert_abs_diff_eq!(y_actual, 0.0011, epsilon = 1e-4);
        assert_abs_diff_eq!(curvature, -0.0005, epsilon = 1e-2);
    }

    #[test]
    fn test_pid_lateral_control() {
        let v_ego = 20.0;
        let y_actual = 0.1;
        let y_des = 0.0;
        let ui_steer = 0.2;
        let steer_max = 1.0;
        let steer_override = false;
        let sat_count = 0.0;
        let enabled = true;
        let half_pid = false;
        let rate = 100.0;

        let (output_steer, up_steer, new_ui_steer, lateral_control_sat, new_sat_count, sat_flag) =
            pid_lateral_control(
                v_ego,
                y_actual,
                y_des,
                ui_steer,
                steer_max,
                steer_override,
                sat_count,
                enabled,
                half_pid,
                rate,
            );

        assert_eq!(output_steer, -1.0);

        assert_abs_diff_eq!(up_steer, -1.20, epsilon = 1e-4);
        assert_eq!(new_ui_steer, 0.199);
        assert_eq!(lateral_control_sat, false);
        assert_eq!(new_sat_count, 0.0);
        assert_eq!(sat_flag, false);
    }

    #[test]
    fn test_lat_control_new() {
        let lat_control = LatControl::new();
        assert_eq!(lat_control.ui_steer, 0.0);
    }

    #[test]
    fn test_lat_control_reset() {
        let mut lat_control = LatControl::new();
        lat_control.ui_steer = 0.5;
        lat_control.reset();
        assert_eq!(lat_control.ui_steer, 0.0);
    }

    #[test]
    fn test_lat_control_update() {
        let mut lat_control = LatControl::new();
        let enabled = true;
        let cs = CS {
            v_ego: 20.0,
            angle_steers: 5.0,
            steer_override: false,
            torque_mod: false,
            vp: VP {
                steer_ratio: 12.0,
                wheelbase: 2.7,
                slip_factor: 0.01,
            },
        };
        let d_poly = Array1::from(vec![0.1, 0.05, -0.02]);
        let angle_offset = 0.0;

        let (output_steer, sat_flag) = lat_control.update(enabled, &cs, &d_poly, angle_offset);

        assert_eq!(output_steer, -1.0);
        assert_eq!(sat_flag, false);
    }
}
