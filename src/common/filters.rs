/// Represents a first-order lowpass filter for discrete time signals.
///
/// This filter is used for smoothing or attenuating high-frequency components of a signal.
///
/// # Examples
///
/// ```rust
/// use openpilot::common::filters::FirstOrderLowpassFilter;
///
/// let fc = 5.0; // Cutoff frequency in Hertz
/// let dt = 0.02; // Time step in seconds
/// let mut lowpass_filter = FirstOrderLowpassFilter::new(fc, dt, 0.0);
///
/// let input_signal = 10.0;
/// let filtered_output = lowpass_filter.apply(input_signal);
/// println!("Filtered Output: {}", filtered_output);
/// ```
#[derive(Clone, Debug)]
pub struct FirstOrderLowpassFilter {
    /// Filter constant
    kf: f64,
    /// State variable
    x1: f64,
}

impl FirstOrderLowpassFilter {
    /// Creates a new `FirstOrderLowpassFilter` instance.
    ///
    /// # Arguments
    ///
    /// * `fc` - Cutoff frequency in Hertz.
    /// * `dt` - Time step in seconds.
    /// * `x1` - Initial state (default is 0.0).
    ///
    /// # Returns
    ///
    /// A new `FirstOrderLowpassFilter` instance.
    pub fn new(fc: f64, dt: f64, x1: f64) -> Self {
        let kf =
            2.0 * std::f64::consts::PI * fc * dt / (1.0 + 2.0 * std::f64::consts::PI * fc * dt);
        Self { kf, x1 }
    }

    /// Applies the lowpass filter to the input signal.
    ///
    /// # Arguments
    ///
    /// * `x` - Input signal value.
    ///
    /// # Returns
    ///
    /// The filtered output signal value.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::common::filters::FirstOrderLowpassFilter;
    ///
    /// let fc = 5.0; // Cutoff frequency in Hertz
    /// let dt = 0.02; // Time step in seconds
    /// let mut lowpass_filter = FirstOrderLowpassFilter::new(fc, dt, 0.0);
    ///
    /// let input_signal = 10.0;
    /// let filtered_output = lowpass_filter.apply(input_signal);
    /// println!("Filtered Output: {}", filtered_output);
    /// ```
    pub fn apply(&mut self, x: f64) -> f64 {
        self.x1 = (1.0 - self.kf) * self.x1 + self.kf * x;

        // If previous or current is NaN, reset filter.
        if self.x1.is_nan() {
            self.x1 = 0.0;
        }

        self.x1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lowpass_filter_creation() {
        let fc = 5.0;
        let dt = 0.02;
        let filter = FirstOrderLowpassFilter::new(fc, dt, 0.0);

        assert_eq!(filter.kf, 0.38586954509503757);
        assert_eq!(filter.x1, 0.0);
    }

    #[test]
    fn test_lowpass_filter_application() {
        let fc = 5.0;
        let dt = 0.02;
        let mut filter = FirstOrderLowpassFilter::new(fc, dt, 0.0);

        let filtered_output = filter.apply(10.0);
        assert_eq!(filtered_output, 3.8586954509503757);

        // Applying again to test state persistence
        let filtered_output2 = filter.apply(15.0);
        assert_eq!(filtered_output2, 8.157785569057427);
    }

    #[test]
    fn test_lowpass_filter_nan_reset() {
        let fc = 5.0;
        let dt = 0.02;
        let mut filter = FirstOrderLowpassFilter::new(fc, dt, 0.0);

        // Apply with NaN to trigger reset
        let filtered_output = filter.apply(std::f64::NAN);
        assert_eq!(filtered_output, 0.0);

        // Apply again with a valid value
        let filtered_output2 = filter.apply(15.0);
        assert_eq!(filtered_output2, 5.788043176425564);
    }
}
