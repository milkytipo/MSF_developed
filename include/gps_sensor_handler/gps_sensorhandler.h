/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (C) 2011-2012 Stephan Weiss, ASL, ETH Zurich, Switzerland
 * You can contact the author at <stephan dot weiss at ieee dot org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef gps_SENSOR_H_
#define gps_SENSOR_H_

#include <msf_core/msf_sensormanagerROS.h>
#include <sensor_msgs/NavSatFix.h>

namespace msf_gps_sensor {

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
class GpsSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:
  Eigen::Matrix<double, 3, 1> z_p_;  ///< Position measurement gps get from globe, longitude, laititude, altitude
  double n_zp_;  ///< Position and attitude measurement noise.
  double delay_;        ///< Delay to be subtracted from the ros-timestamp of
  // the measurement provided by this sensor.

  ros::Subscriber subNavSatFix;

  bool use_fixed_covariance_;  ///< Use fixed covariance set by dynamic reconfigure
  bool provides_absolute_measurements_;  ///<Does this sensor measure relative or
                                         // absolute values

  double gps_measurement_minimum_dt_; ///< Minimum time between two gps measurements in seconds.
                                       // If more GPS measurements are received they are dropped.

  double timestamp_previous_gps_;  ///< Timestamp of previous gps message to subsample messages.

  void ProcessGpsMeasurement(
      const sensor_msgs::NavSatFix & msg);
  void MeasurementCallback(
      const sensor_msgs::NavSatFix & msg);

 public:
  typedef MEASUREMENT_TYPE measurement_t;
  GpsSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace,
                    std::string parameternamespace, bool distortmeas);
  // Used for the init.
  Eigen::Matrix<double, 3, 1> GetGpsMeasurement() {
    return z_p_;
  }
  //setters for configure values
  void SetNoises(double n_zp);
  void SetDelay(double delay);
};
}  // namespace msf_gps_sensor
#include "implementation/gps_sensorhandler.hpp"

#endif  // gps_SENSOR_H_
