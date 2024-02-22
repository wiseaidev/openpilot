use crate::common::ext_kal_fltr::{FastEKF1D, SimpleSensor, EKF};
use interp::interp;
use ndarray::{arr1, arr2, Array1};

// Radar tracks
const SPEED: usize = 0;
const ACCEL: usize = 1;
/// Converts radar distance to lidar distance.
const RDR_TO_LDR: f64 = 2.7;

/// Represents a track with Kalman filtering for object tracking.
#[derive(Debug)]
pub struct Track {
    /// Kalman filter for 1D tracking
    pub ekf: Option<FastEKF1D>,
    /// Indicates if the object is stationary
    pub stationary: bool,
    /// Indicates if the track has been initialized
    pub initted: bool,
    /// Previous relative longitudinal distance
    pub d_rel_prev: f64,
    /// Previous negative lateral distance
    pub y_rel_prev: f64,
    /// Previous relative speed
    pub v_rel_prev: f64,
    /// Current relative longitudinal distance
    pub d_rel: f64,
    /// Current negative lateral distance
    pub y_rel: f64,
    /// Current relative speed
    pub v_rel: f64,
    /// Computed distance to the path
    pub d_path: f64,
    /// Lead vehicle relative speed
    pub v_lead: f64,
    /// Relative acceleration
    pub a_rel: f64,
    /// Filtered lateral velocity
    pub v_lat: f64,
    /// Lead vehicle acceleration
    pub a_lead: f64,
    /// Indicates if the lead vehicle is oncoming
    pub oncoming: bool,
    /// Sensor for lead vehicle speed
    pub lead_sensor: SimpleSensor,
    /// Previous lead vehicle relative speed
    pub v_lead_prev: f64,
    /// Previous relative acceleration
    pub a_rel_prev: f64,
    /// Previous filtered lateral velocity
    pub v_lat_prev: f64,
    /// Previous lead vehicle acceleration
    pub a_lead_prev: f64,
    /// Kalman filter predicted lead vehicle relative speed
    pub v_lead_k: f64,
    /// Kalman filter predicted lead vehicle acceleration
    pub a_lead_k: f64,
    /// Counter for track updates
    pub cnt: usize,
    /// Indicates if vision data is available
    pub vision: bool,
    /// Counter for vision updates
    pub vision_cnt: usize,
}

impl Track {
    /// Creates a new `Track` instance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::Track;
    /// let track = Track::new();
    /// ```
    pub fn new() -> Self {
        Track {
            ekf: None,
            stationary: true,
            initted: false,
            d_rel_prev: 0.0,
            y_rel_prev: 0.0,
            v_rel_prev: 0.0,
            d_rel: 0.0,
            y_rel: 0.0,
            v_rel: 0.0,
            d_path: 0.0,
            v_lead: 0.0,
            a_rel: 0.0,
            v_lat: 0.0,
            a_lead: 0.0,
            oncoming: false,
            lead_sensor: SimpleSensor::new(
                arr2(&[[SPEED as f64, SPEED as f64], [SPEED as f64, SPEED as f64]]),
                arr2(&[[1.0, 1.0], [1.0, 1.0]]),
                2,
            ),
            v_lead_prev: 0.0,
            a_rel_prev: 0.0,
            v_lat_prev: 0.0,
            a_lead_prev: 0.0,
            v_lead_k: 0.0,
            a_lead_k: 0.0,
            cnt: 0,
            vision: false,
            vision_cnt: 0,
        }
    }

    /// Updates the track with new sensor data and ego vehicle information.
    ///
    /// # Arguments
    ///
    /// * `d_rel` - Relative longitudinal distance.
    /// * `y_rel` - Negative lateral distance.
    /// * `v_rel` - Relative speed.
    /// * `d_path` - Computed distance to the path.
    /// * `v_ego_t_aligned` - Aligned ego vehicle speed.
    /// * `ts` - Time step.
    /// * `k_v_lat` - Coefficient for lateral velocity filtering.
    /// * `k_a_lead` - Coefficient for lead vehicle acceleration filtering.
    /// * `v_stationary_thr` - Threshold for classifying stationary objects.
    /// * `v_oncoming_thr` - Threshold for classifying oncoming objects.
    /// * `v_ego_stationary` - Threshold for classifying ego vehicle as stationary.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::Track;
    /// let mut track = Track::new();
    /// track.update(10.0, -2.0, 15.0, 5.0, 20.0, 0.1, 0.8, 0.6, 5.0, 10.0, 2.0);
    /// ```
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        d_rel: f64,
        y_rel: f64,
        v_rel: f64,
        d_path: f64,
        v_ego_t_aligned: f64,
        ts: f64,
        k_v_lat: f64,
        k_a_lead: f64,
        v_stationary_thr: f64,
        v_oncoming_thr: f64,
        v_ego_stationary: f64,
    ) {
        if self.initted {
            self.d_rel_prev = self.d_rel;
            self.v_lead_prev = self.v_lead;
            self.v_rel_prev = self.v_rel;
        }

        self.d_rel = d_rel;
        self.y_rel = y_rel;
        self.v_rel = v_rel;
        self.d_path = d_path;
        self.v_lead = self.v_rel + v_ego_t_aligned;

        if !self.initted {
            self.a_rel = 0.0;
            self.v_lat = 0.0;
            self.a_lead = 0.0;
        } else {
            let a_rel_unfilt = (self.v_rel - self.v_rel_prev) / ts;
            self.a_rel = k_a_lead * a_rel_unfilt.clamp(-10.0, 10.0) + (1.0 - k_a_lead) * self.a_rel;

            let v_lat_unfilt = (self.d_path - self.d_rel_prev) / ts;
            self.v_lat = k_v_lat * v_lat_unfilt + (1.0 - k_v_lat) * self.v_lat;

            let a_lead_unfilt = (self.v_lead - self.v_lead_prev) / ts;
            self.a_lead =
                k_a_lead * a_lead_unfilt.clamp(-10.0, 10.0) + (1.0 - k_a_lead) * self.a_lead;
        }

        if self.stationary {
            self.stationary =
                v_ego_t_aligned > v_ego_stationary && (self.v_lead).abs() < v_stationary_thr;
        }
        self.oncoming = self.v_lead < v_oncoming_thr;

        if self.ekf.is_none() {
            let mut ekf = FastEKF1D::new(ts, 1e3, 0.1);
            ekf.state[SPEED] = self.v_lead;
            ekf.state[ACCEL] = 0.0;
            self.ekf = Some(ekf);
            self.v_lead_k = self.v_lead;
            self.a_lead_k = self.a_lead;
        } else {
            let ekf = self.ekf.as_mut().unwrap();
            ekf.update_scalar(&self.lead_sensor.read(arr2(&[[self.v_lead]]), None));
            ekf.predict(ts);
            self.v_lead_k = ekf.state[SPEED];
            self.a_lead_k = ekf.state[ACCEL];
        }

        if !self.initted {
            self.cnt = 1;
            self.vision_cnt = 0;
        } else {
            self.cnt += 1;
        }

        self.initted = true;
        self.vision = false;
    }

    /// Mixes vision data with sensor data.
    ///
    /// # Arguments
    ///
    /// * `dist_to_vision` - Distance to vision point.
    /// * `rel_speed_diff` - Relative speed difference.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::Track;
    /// let mut track = Track::new();
    /// track.mix_vision(3.0, 5.0);
    /// ```
    pub fn mix_vision(&mut self, dist_to_vision: f64, rel_speed_diff: f64) {
        if dist_to_vision < 4.0 && rel_speed_diff < 10.0 {
            self.stationary = false;
            self.vision = true;
            self.vision_cnt += 1;
        }
    }

    /// Generates a key for clustering based on track parameters.
    ///
    /// # Returns
    ///
    /// (`Array1<f64>`): Array containing track parameters for clustering.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::Track;
    /// use ndarray::arr1;
    /// let track = Track::new();
    /// let key = track.get_key_for_cluster();
    /// assert_eq!(key, arr1(&[0.0; 3]));
    /// ```
    pub fn get_key_for_cluster(&self) -> Array1<f64> {
        // Weigh y higher since radar is inaccurate in this dimension
        arr1(&[self.d_rel, self.d_path * 2.0, self.v_rel])
    }
}

impl Default for Track {
    fn default() -> Self {
        Self::new()
    }
}

/// Represents a cluster of tracks.
#[derive(Debug)]
pub struct Cluster {
    // Tracks in the cluster
    pub tracks: Vec<Track>,
}

impl Cluster {
    /// Creates a new `Cluster` instance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::Cluster;
    /// let cluster = Cluster::new();
    /// ```
    pub fn new() -> Self {
        Cluster { tracks: Vec::new() }
    }

    /// Adds a track to the cluster.
    ///
    /// # Arguments
    ///
    /// * `track` - Track to be added to the cluster.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let mut cluster = Cluster::new();
    /// let track = Track::new();
    /// cluster.add(track);
    /// ```
    pub fn add(&mut self, track: Track) {
        self.tracks.push(track);
    }

    /// Calculates the mean of relative longitudinal distance for the cluster.
    ///
    /// # Returns
    ///
    /// (`f64`): Mean relative longitudinal distance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let track1 = Track::new();
    /// let track2 = Track::new();
    /// let mut cluster = Cluster::new();
    /// cluster.add(track1);
    /// cluster.add(track2);
    /// assert_eq!(cluster.d_rel(), 0.0);
    /// ```
    pub fn d_rel(&self) -> f64 {
        self.tracks.iter().map(|t| t.d_rel).sum::<f64>() / self.tracks.len() as f64
    }

    /// Calculates the mean of negative lateral distance for the cluster.
    ///
    /// # Returns
    ///
    /// (`f64`): Mean negative lateral distance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let track1 = Track::new();
    /// let track2 = Track::new();
    /// let mut cluster = Cluster::new();
    /// cluster.add(track1);
    /// cluster.add(track2);
    /// assert_eq!(cluster.y_rel(), 0.0);
    /// ```
    pub fn y_rel(&self) -> f64 {
        self.tracks.iter().map(|t| t.y_rel).sum::<f64>() / self.tracks.len() as f64
    }

    /// Calculates the mean of relative speed for the cluster.
    ///
    /// # Returns
    ///
    /// (`f64`): Mean relative speed.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let track1 = Track::new();
    /// let track2 = Track::new();
    /// let mut cluster = Cluster::new();
    /// cluster.add(track1);
    /// cluster.add(track2);
    /// assert_eq!(cluster.v_rel(), 0.0);
    /// ```
    pub fn v_rel(&self) -> f64 {
        self.tracks.iter().map(|t| t.v_rel).sum::<f64>() / self.tracks.len() as f64
    }

    /// Calculates the mean of relative acceleration for the cluster.
    ///
    /// # Returns
    ///
    /// (`f64`): Mean relative acceleration.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let track1 = Track::new();
    /// let track2 = Track::new();
    /// let mut cluster = Cluster::new();
    /// cluster.add(track1);
    /// cluster.add(track2);
    /// assert_eq!(cluster.a_rel(), 0.0);
    /// ```
    pub fn a_rel(&self) -> f64 {
        self.tracks.iter().map(|t| t.a_rel).sum::<f64>() / self.tracks.len() as f64
    }

    /// Calculates the mean of lead vehicle speed for the cluster.
    ///
    /// # Returns
    ///
    /// (`f64`): Mean lead vehicle speed.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let track1 = Track::new();
    /// let track2 = Track::new();
    /// let mut cluster = Cluster::new();
    /// cluster.add(track1);
    /// cluster.add(track2);
    /// assert_eq!(cluster.v_lead(), 0.0);
    /// ```
    pub fn v_lead(&self) -> f64 {
        self.tracks.iter().map(|t| t.v_lead).sum::<f64>() / self.tracks.len() as f64
    }

    /// Calculates the mean of lead vehicle acceleration for the cluster.
    ///
    /// # Returns
    ///
    /// (`f64`): Mean lead vehicle acceleration.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let track1 = Track::new();
    /// let track2 = Track::new();
    /// let mut cluster = Cluster::new();
    /// cluster.add(track1);
    /// cluster.add(track2);
    /// assert_eq!(cluster.a_lead(), 0.0);
    /// ```
    pub fn a_lead(&self) -> f64 {
        self.tracks.iter().map(|t| t.a_lead).sum::<f64>() / self.tracks.len() as f64
    }

    /// Calculates the mean of the computed distance to the path for the cluster.
    ///
    /// The computed distance to the path is a measure of lateral deviation from the desired path.
    ///
    /// # Returns
    ///
    /// (`f64`): Mean computed distance to the path.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let track1 = Track::new();
    /// let track2 = Track::new();
    /// let mut cluster = Cluster::new();
    /// cluster.add(track1);
    /// cluster.add(track2);
    /// assert_eq!(cluster.d_path(), 0.0);
    /// ```
    pub fn d_path(&self) -> f64 {
        self.tracks.iter().map(|t| t.d_path).sum::<f64>() / self.tracks.len() as f64
    }

    /// Calculates the mean of lateral velocity for the cluster.
    ///
    /// Lateral velocity represents the speed at which the vehicle is moving laterally.
    ///
    /// # Returns
    ///
    /// (`f64`): Mean lateral velocity.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let track1 = Track::new();
    /// let track2 = Track::new();
    /// let mut cluster = Cluster::new();
    /// cluster.add(track1);
    /// cluster.add(track2);
    /// assert_eq!(cluster.v_lat(), 0.0);
    /// ```
    pub fn v_lat(&self) -> f64 {
        self.tracks.iter().map(|t| t.v_lat).sum::<f64>() / self.tracks.len() as f64
    }

    /// Calculates the mean of lead vehicle speed predicted by the extended Kalman filter for the cluster.
    ///
    /// The extended Kalman filter is used to predict the lead vehicle's speed.
    ///
    /// # Returns
    ///
    /// (`f64`): Mean predicted lead vehicle speed.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let track1 = Track::new();
    /// let track2 = Track::new();
    /// let mut cluster = Cluster::new();
    /// cluster.add(track1);
    /// cluster.add(track2);
    /// assert_eq!(cluster.v_lead_k(), 0.0);
    /// ```
    pub fn v_lead_k(&self) -> f64 {
        self.tracks.iter().map(|t| t.v_lead_k).sum::<f64>() / self.tracks.len() as f64
    }

    /// Calculates the mean of lead vehicle acceleration predicted by the extended Kalman filter for the cluster.
    ///
    /// The extended Kalman filter is used to predict the lead vehicle's acceleration.
    ///
    /// # Returns
    ///
    /// (`f64`): Mean predicted lead vehicle acceleration.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let track1 = Track::new();
    /// let track2 = Track::new();
    /// let mut cluster = Cluster::new();
    /// cluster.add(track1);
    /// cluster.add(track2);
    /// assert_eq!(cluster.a_lead_k(), 0.0);
    /// ```
    pub fn a_lead_k(&self) -> f64 {
        self.tracks.iter().map(|t| t.a_lead_k).sum::<f64>() / self.tracks.len() as f64
    }

    /// Checks if any track in the cluster has vision data.
    ///
    /// # Returns
    ///
    /// (`bool`): Whether any track in the cluster has vision data.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let track1 = Track::new();
    /// let track2 = Track::new();
    /// let mut cluster = Cluster::new();
    /// cluster.add(track1);
    /// cluster.add(track2);
    /// assert_eq!(cluster.vision(), false);
    /// ```
    pub fn vision(&self) -> bool {
        self.tracks.iter().any(|t| t.vision)
    }

    /// Returns the maximum vision count among all tracks in the cluster.
    ///
    /// # Returns
    ///
    /// (`i32`): Maximum vision count among all tracks in the cluster.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let track1 = Track::new();
    /// let track2 = Track::new();
    /// let mut cluster = Cluster::new();
    /// cluster.add(track1);
    /// cluster.add(track2);
    /// assert_eq!(cluster.vision_cnt(), 0);
    /// ```
    pub fn vision_cnt(&self) -> i32 {
        self.tracks
            .iter()
            .map(|t| t.vision_cnt)
            .max()
            .unwrap_or(0)
            .try_into()
            .unwrap()
    }

    /// Checks if all tracks in the cluster are stationary.
    ///
    /// # Returns
    ///
    /// (`bool`): Whether all tracks in the cluster are stationary.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let track1 = Track::new();
    /// let track2 = Track::new();
    /// let mut cluster = Cluster::new();
    /// cluster.add(track1);
    /// cluster.add(track2);
    /// assert_eq!(cluster.stationary(), true);
    /// ```
    pub fn stationary(&self) -> bool {
        self.tracks.iter().all(|t| t.stationary)
    }

    /// Checks if all tracks in the cluster are oncoming.
    ///
    /// # Returns
    ///
    /// (`bool`): Whether all tracks in the cluster are oncoming.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let track1 = Track::new();
    /// let track2 = Track::new();
    /// let mut cluster = Cluster::new();
    /// cluster.add(track1);
    /// cluster.add(track2);
    /// assert_eq!(cluster.oncoming(), false);
    /// ```
    pub fn oncoming(&self) -> bool {
        self.tracks.iter().all(|t| t.oncoming)
    }

    /// Converts cluster data to Live20 format for lead vehicle tracking.
    ///
    /// # Arguments
    ///
    /// * `lead` - Lead vehicle object.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Lead};
    /// let cluster = Cluster::new();
    /// let mut lead = Lead::new();
    /// cluster.to_live20(&mut lead);
    /// ```
    pub fn to_live20(&self, lead: &mut Lead) {
        lead.d_rel = self.d_rel() - RDR_TO_LDR;
        lead.y_rel = self.y_rel();
        lead.v_rel = self.v_rel();
        lead.a_rel = self.a_rel();
        lead.v_lead = self.v_lead();
        lead.a_lead = self.a_lead();
        lead.d_path = self.d_path();
        lead.v_lat = self.v_lat();
        lead.v_lead_k = self.v_lead_k();
        lead.a_lead_k = self.a_lead_k();
        lead.status = true;
        lead.fcw = false;
    }

    /// Checks if the cluster represents a potential lead vehicle.
    ///
    /// # Arguments
    ///
    /// * `v_ego` - Ego vehicle speed.
    /// * `enabled` - Flag indicating if the check is enabled.
    ///
    /// # Returns
    ///
    /// (`bool`): Whether the cluster represents a potential lead vehicle.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let cluster = Cluster::new();
    /// let is_lead = cluster.is_potential_lead(20.0, true);
    /// ```
    pub fn is_potential_lead(&self, v_ego: f64, enabled: bool) -> bool {
        // Predict cut-ins by extrapolating lateral speed by a lookahead time
        // Lookahead time depends on cut-in distance. More attentive for close cut-ins
        // Also, above 50 meters the predicted path isn't very reliable

        // The distance at which v_lat matters is higher at higher speed
        let lookahead_dist = 40.0 + v_ego / 1.2; // 40m at 0mph, ~70m at 80mph

        let t_lookahead_v = [1.0, 0.0];
        let t_lookahead_bp = [10.0, lookahead_dist];

        // Average dist
        let d_path = self.d_path();

        if enabled {
            let t_lookahead = interp(
                &[t_lookahead_v[0]],
                &[t_lookahead_v[1]],
                (self.d_rel() - t_lookahead_bp[0]) / (t_lookahead_bp[1] - t_lookahead_bp[0]),
            );
            // Correct d_path for lookahead time, considering only cut-ins and no more than 1m impact
            let lat_corr = (t_lookahead * self.v_lat()).clamp(-1.0, 0.0);
            let d_path = f64::max(d_path + lat_corr, 0.0);

            d_path < 1.5 && !self.stationary() && !self.oncoming()
        } else {
            false
        }
    }

    /// Checks if the cluster represents a potential lead vehicle using an alternate method.
    ///
    /// # Arguments
    ///
    /// * `lead_clusters` - Lead clusters for comparison.
    ///
    /// # Returns
    ///
    /// (`bool`): Whether the cluster represents a potential lead vehicle.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::{Cluster, Track};
    /// let cluster = Cluster::new();
    /// let lead_clusters = vec![Cluster::new()];
    /// let is_lead = cluster.is_potential_lead2(&lead_clusters);
    /// ```
    pub fn is_potential_lead2(&self, lead_clusters: &[Cluster]) -> bool {
        if let Some(lead_cluster) = lead_clusters.first() {
            // Check if the new lead is too close and roughly at the same speed of the first lead
            // It might just be the second axle of the same vehicle
            (self.d_rel() - lead_cluster.d_rel()) < 8.0
                && (self.v_rel() - lead_cluster.v_rel()).abs() < 1.0
        } else {
            false
        }
    }
}

impl Default for Cluster {
    fn default() -> Self {
        Self::new()
    }
}

/// Represents a lead vehicle.
#[derive(Debug)]
pub struct Lead {
    /// Relative longitudinal distance to the lead vehicle.
    pub d_rel: f64,
    /// Negative lateral distance to the lead vehicle.
    pub y_rel: f64,
    /// Relative speed to the lead vehicle.
    pub v_rel: f64,
    /// Relative acceleration to the lead vehicle.
    pub a_rel: f64,
    /// Lead vehicle speed.
    pub v_lead: f64,
    /// Lead vehicle acceleration.
    pub a_lead: f64,
    /// Computed distance to the path.
    pub d_path: f64,
    /// Filtered lateral velocity.
    pub v_lat: f64,
    /// Kalman filter predicted lead vehicle speed.
    pub v_lead_k: f64,
    /// Kalman filter predicted lead vehicle acceleration.
    pub a_lead_k: f64,
    /// Status indicating the presence of a lead vehicle.
    pub status: bool,
    /// Forward Collision Warning (FCW) status.
    pub fcw: bool,
}

impl Lead {
    /// Creates a new `Lead` instance with default values.
    ///
    /// # Returns
    ///
    /// (`Lead`): A new `Lead` instance with default values.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use openpilot::selfdrive::controls::radar_helpers::Lead;
    ///
    /// let lead = Lead::new();
    /// assert_eq!(lead.status, false);
    /// ```
    pub fn new() -> Self {
        Lead {
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
            fcw: false,
        }
    }
}

impl Default for Lead {
    fn default() -> Self {
        Self::new()
    }
}


#[cfg(test)]
mod track_tests {
    use super::*;

    #[test]
    fn test_track_creation() {
        let track = Track::new();
        assert!(track.ekf.is_none());
        assert_eq!(track.stationary, true);
        assert_eq!(track.initted, false);
        assert_eq!(track.d_rel, 0.0);
        assert_eq!(track.y_rel, 0.0);
        assert_eq!(track.v_rel, 0.0);
        assert_eq!(track.d_path, 0.0);
        assert_eq!(track.v_lead, 0.0);
        assert_eq!(track.a_rel, 0.0);
        assert_eq!(track.v_lat, 0.0);
        assert_eq!(track.a_lead, 0.0);
        assert_eq!(track.oncoming, false);
        assert_eq!(track.v_lead_prev, 0.0);
        assert_eq!(track.a_rel_prev, 0.0);
        assert_eq!(track.v_lat_prev, 0.0);
        assert_eq!(track.a_lead_prev, 0.0);
        assert_eq!(track.v_lead_k, 0.0);
        assert_eq!(track.a_lead_k, 0.0);
        assert_eq!(track.cnt, 0);
        assert_eq!(track.vision, false);
        assert_eq!(track.vision_cnt, 0);
    }

    #[test]
    fn test_track_update() {
        let mut track = Track::new();
        track.update(
            10.0, -5.0, 20.0, 15.0, 30.0, 0.1, 0.5, 0.5, 15.0, 20.0, 10.0,
        );
        assert_eq!(track.d_rel, 10.0);
        assert_eq!(track.y_rel, -5.0);
        assert_eq!(track.v_rel, 20.0);
        assert_eq!(track.d_path, 15.0);
        assert_eq!(track.v_lead, 50.0);
        assert_eq!(track.a_rel, 0.0);
        assert_eq!(track.v_lat, 0.0);
        assert_eq!(track.a_lead, 0.0);
        assert_eq!(track.oncoming, false);
        assert_eq!(track.v_lead_prev, 0.0);
        assert_eq!(track.a_rel_prev, 0.0);
        assert_eq!(track.v_lat_prev, 0.0);
        assert_eq!(track.a_lead_prev, 0.0);
        assert_eq!(track.v_lead_k, 50.0);
        assert_eq!(track.a_lead_k, 0.0);
        assert_eq!(track.cnt, 1);
        assert_eq!(track.vision, false);
        assert_eq!(track.vision_cnt, 0);
    }

    #[test]
    fn test_track_mix_vision() {
        let mut track = Track::new();
        track.mix_vision(3.0, 8.0);
        assert_eq!(track.stationary, false);
        assert_eq!(track.vision, true);
        assert_eq!(track.vision_cnt, 1);
    }

    #[test]
    fn test_track_get_key_for_cluster() {
        let track = Track::new();
        let key = track.get_key_for_cluster();
        assert_eq!(key, arr1(&[0.0, 0.0, 0.0]));
    }
}

#[cfg(test)]
mod cluster_tests {
    use super::*;

    #[test]
    fn test_cluster_creation() {
        let cluster = Cluster::new();
        assert_eq!(cluster.tracks.len(), 0);
    }

    #[test]
    fn test_cluster_add() {
        let mut cluster = Cluster::new();
        let track = Track::new();
        cluster.add(track);
        assert_eq!(cluster.tracks.len(), 1);
    }

    #[test]
    fn test_cluster_d_rel() {
        let mut cluster = Cluster::new();
        let track1 = Track::new();
        let track2 = Track::new();
        cluster.add(track1);
        cluster.add(track2);
        assert_eq!(cluster.d_rel(), 0.0);
    }

    #[test]
    fn test_cluster_y_rel() {
        let mut cluster = Cluster::new();
        let track1 = Track::new();
        let track2 = Track::new();
        cluster.add(track1);
        cluster.add(track2);
        assert_eq!(cluster.y_rel(), 0.0);
    }

    #[test]
    fn test_cluster_v_rel() {
        let mut cluster = Cluster::new();
        let track1 = Track::new();
        let track2 = Track::new();
        cluster.add(track1);
        cluster.add(track2);
        assert_eq!(cluster.v_rel(), 0.0);
    }

    #[test]
    fn test_cluster_a_rel() {
        let mut cluster = Cluster::new();
        let track1 = Track::new();
        let track2 = Track::new();
        cluster.add(track1);
        cluster.add(track2);
        assert_eq!(cluster.a_rel(), 0.0);
    }

    #[test]
    fn test_cluster_v_lead() {
        let mut cluster = Cluster::new();
        let track1 = Track::new();
        let track2 = Track::new();
        cluster.add(track1);
        cluster.add(track2);
        assert_eq!(cluster.v_lead(), 0.0);
    }

    #[test]
    fn test_cluster_a_lead() {
        let mut cluster = Cluster::new();
        let track1 = Track::new();
        let track2 = Track::new();
        cluster.add(track1);
        cluster.add(track2);
        assert_eq!(cluster.a_lead(), 0.0);
    }

    #[test]
    fn test_cluster_d_path() {
        let mut cluster = Cluster::new();
        let track1 = Track::new();
        let track2 = Track::new();
        cluster.add(track1);
        cluster.add(track2);
        assert_eq!(cluster.d_path(), 0.0);
    }

    #[test]
    fn test_track_mix_vision() {
        let mut track = Track::new();
        track.mix_vision(3.0, 5.0);
        assert_eq!(track.stationary, false);
        assert_eq!(track.vision, true);
        assert_eq!(track.vision_cnt, 1);
    }

    #[test]
    fn test_track_get_key_for_cluster() {
        let track = Track::new();
        let key = track.get_key_for_cluster();
        assert_eq!(key, arr1(&[0.0, 0.0, 0.0]));
    }

    #[test]
    fn test_cluster_to_live20() {
        // Test Cluster to_live20
        let mut cluster = Cluster::new();
        let track1 = Track::new();
        let track2 = Track::new();
        cluster.add(track1);
        cluster.add(track2);

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

        assert_eq!(lead.d_rel, -2.7);
        assert_eq!(lead.y_rel, 0.0);
        assert_eq!(lead.v_rel, 0.0);
        assert_eq!(lead.a_rel, 0.0);
        assert_eq!(lead.v_lead, 0.0);
        assert_eq!(lead.a_lead, 0.0);
        assert_eq!(lead.d_path, 0.0);
        assert_eq!(lead.v_lat, 0.0);
        assert_eq!(lead.v_lead_k, 0.0);
        assert_eq!(lead.a_lead_k, 0.0);
        assert_eq!(lead.status, true);
        assert_eq!(lead.fcw, false);
    }
}

#[cfg(test)]
mod lead_tests {
    use super::*;

    #[test]
    fn test_lead_creation() {
        let lead = Lead {
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

        assert_eq!(lead.d_rel, 0.0);
        assert_eq!(lead.y_rel, 0.0);
        assert_eq!(lead.v_rel, 0.0);
        assert_eq!(lead.a_rel, 0.0);
        assert_eq!(lead.v_lead, 0.0);
        assert_eq!(lead.a_lead, 0.0);
        assert_eq!(lead.d_path, 0.0);
        assert_eq!(lead.v_lat, 0.0);
        assert_eq!(lead.v_lead_k, 0.0);
        assert_eq!(lead.a_lead_k, 0.0);
        assert_eq!(lead.status, false);
        assert_eq!(lead.fcw, true);
    }
}
