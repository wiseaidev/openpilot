use interp::interp;
use ndarray::Array1;

/// Applies rate limiting to a new value based on the last value, down step, and up step.
///
/// # Arguments
///
/// * `new_value` - The new value to be rate-limited.
/// * `last_value` - The last value before rate limiting.
/// * `dw_step` - The downward step for rate limiting.
/// * `up_step` - The upward step for rate limiting.
///
/// # Returns
///
/// The rate-limited value.
///
/// # Examples
///
/// ```rust
/// use openpilot::selfdrive::controls::drive_helpers::rate_limit;
///
/// let new_value = rate_limit(1.5, 1.0, 0.1, 0.2);
/// assert_eq!(new_value, 1.2); // Asserting rate-limited value
/// ```
pub fn rate_limit(new_value: f64, last_value: f64, dw_step: f64, up_step: f64) -> f64 {
    new_value
        .max(last_value + dw_step)
        .min(last_value + up_step)
}

/// Learns the angle offset to keep the car going straight using a simple integral controller.
///
/// # Arguments
///
/// * `lateral_control` - Indicates if lateral control is active.
/// * `v_ego` - Ego vehicle speed in m/s.
/// * `angle_offset` - Current angle offset.
/// * `d_poly` - Polynomial coefficients for lateral control.
/// * `y_des` - Desired lateral position offset.
/// * `steer_override` - Indicates if the driver is manually steering.
///
/// # Returns
///
/// The updated angle offset after learning.
///
/// # Examples
///
/// ```rust
/// use openpilot::selfdrive::controls::drive_helpers::learn_angle_offset;
/// use ndarray::Array1;
///
/// let lateral_control = true;
/// let v_ego = 20.0;
/// let angle_offset = 0.1;
/// let d_poly = Array1::from(vec![1.0, 2.0, 3.0, 4.0]);
/// let y_des = 0.5;
/// let steer_override = false;
///
/// let new_angle_offset = learn_angle_offset(lateral_control, v_ego, angle_offset, &d_poly, y_des, steer_override);
/// ```
pub fn learn_angle_offset(
    lateral_control: bool,
    v_ego: f64,
    angle_offset: f64,
    d_poly: &Array1<f64>,
    y_des: f64,
    steer_override: bool,
) -> f64 {
    // Constants
    const MIN_OFFSET: f64 = -1.0; // deg
    const MAX_OFFSET: f64 = 1.0; // deg
    const ALPHA: f64 = 1.0 / 36000.0; // correct by 1 deg in 2 mins, at 30m/s, with 50cm of error, at 20Hz
    const MIN_LEARN_SPEED: f64 = 1.0;

    // Learn less at low speed or when turning
    let alpha_v = ALPHA * (v_ego - MIN_LEARN_SPEED).max(0.0) / (1.0 + 0.5 * f64::abs(y_des));

    // Only learn if lateral control is active and if the driver is not overriding
    let mut new_angle_offset = angle_offset + d_poly[3] * alpha_v;
    new_angle_offset = new_angle_offset.max(MIN_OFFSET).min(MAX_OFFSET);

    if lateral_control && !steer_override {
        new_angle_offset
    } else {
        angle_offset
    }
}

/// Applies actuator hysteresis logic to avoid brake blinking.
///
/// # Arguments
///
/// * `final_brake` - The final brake command.
/// * `braking` - Indicates if the vehicle is currently braking.
/// * `brake_steady` - The steady brake command.
/// * `v_ego` - Ego vehicle speed in m/s.
/// * `civic` - Indicates if the vehicle is a Civic model.
///
/// # Returns
///
/// A tuple containing the new final brake command, a flag indicating braking status, and the updated steady brake command.
///
/// # Examples
///
/// ```rust
/// use openpilot::selfdrive::controls::drive_helpers::actuator_hystereses;
///
/// let final_brake = 0.2;
/// let braking = true;
/// let brake_steady = 0.1;
/// let v_ego = 30.0;
/// let civic = false;
///
/// let (new_final_brake, new_braking, new_brake_steady) =
///     actuator_hystereses(final_brake, braking, brake_steady, v_ego, civic);
///
/// assert_eq!(new_braking, true); // Asserting braking status
/// assert_eq!(new_brake_steady, 0.19); // Asserting updated steady brake command
/// ```
pub fn actuator_hystereses(
    final_brake: f64,
    braking: bool,
    brake_steady: f64,
    v_ego: f64,
    civic: bool,
) -> (f64, bool, f64) {
    // Hysteresis parameters
    let brake_hyst_on = if civic { 0.055 } else { 0.1 }; // to activate brakes exceed this value
    let brake_hyst_off = 0.005; // to deactivate brakes below this value
    let brake_hyst_gap = 0.01; // don't change brake command for small oscillations within this value

    // Hysteresis logic to avoid brake blinking
    let (mut new_final_brake, new_braking, mut new_brake_steady) =
        if (final_brake < brake_hyst_on && !braking) || final_brake < brake_hyst_off {
            (0.0, false, 0.0)
        } else {
            (final_brake, true, brake_steady)
        };

    // For small brake oscillations within brake_hyst_gap, don't change the brake command
    if final_brake == 0.0 {
        new_brake_steady = 0.0;
    } else if final_brake > new_brake_steady + brake_hyst_gap {
        new_brake_steady = final_brake - brake_hyst_gap;
    } else if final_brake < new_brake_steady - brake_hyst_gap {
        new_brake_steady = final_brake + brake_hyst_gap;
    }

    // Offset the brake command for threshold in the brake system. No brake torque perceived below it
    if !civic {
        let brake_on_offset_v = [0.25, 0.15]; // min brake command on brake activation. below this no decel is perceived
        let brake_on_offset_bp = [15.0, 30.0]; // offset changes VS speed to not have too abrupt decels at high speeds
                                               // Offset the brake command for the threshold in the brake system. No brake torque perceived below it
        let brake_on_offset = interp(&brake_on_offset_v, &brake_on_offset_bp, v_ego);
        let brake_offset = brake_on_offset - brake_hyst_on;
        if new_final_brake > 0.0 {
            new_final_brake += brake_offset;
        }
    }

    (new_final_brake, new_braking, new_brake_steady)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_rate_limit() {
        let new_value = rate_limit(1.5, 1.0, 0.1, 0.2);
        assert!((new_value - 1.2).abs() < f64::EPSILON);
    }

    #[test]
    fn test_learn_angle_offset() {
        let lateral_control = true;
        let v_ego = 20.0;
        let angle_offset = 0.1;
        let d_poly = Array1::from(vec![1.0, 2.0, 3.0, 4.0]);
        let y_des = 0.5;
        let steer_override = false;

        let new_angle_offset = learn_angle_offset(
            lateral_control,
            v_ego,
            angle_offset,
            &d_poly,
            y_des,
            steer_override,
        );
        assert_abs_diff_eq!(new_angle_offset, 0.1016, epsilon = 1e-4);
    }

    #[test]
    fn test_actuator_hystereses() {
        let final_brake = 0.2;
        let braking = true;
        let brake_steady = 0.1;
        let v_ego = 30.0;
        let civic = false;

        let (new_final_brake, new_braking, new_brake_steady) =
            actuator_hystereses(final_brake, braking, brake_steady, v_ego, civic);

        assert_abs_diff_eq!(new_final_brake, -4447.4, epsilon = 1e-4);
        assert_eq!(new_braking, true);
        assert!((new_brake_steady - 0.19).abs() < f64::EPSILON);
    }
}
