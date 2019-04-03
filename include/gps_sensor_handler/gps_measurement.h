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
#ifndef GPS_MEASUREMENT_HPP_
#define GPS_MEASUREMENT_HPP_

#include <msf_core/msf_types.h>
#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <msf_updates/GpsDistorter.h>
#include <sensor_msgs/NavSatFix.h>

namespace msf_updates {
namespace gps_measurement {
enum {
  nMeasurements = 5
};
/**
 * \brief A measurement of global position
 */
typedef msf_core::MSF_Measurement<sensor_msgs::NavSatFix,
    Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> GpsMeasurementBase;

template<
    int StateLIdx = EKFState::StateDefinition_T::L,
    int StatePipIdx = EKFState::StateDefinition_T::p_ip,
    int StateQipIdx = EKFState::StateDefinition_T::q_ip,
    int StateQwgIdx = EKFState::StateDefinition_T::q_wg,
    int StatePwgIdx = EKFState::StateDefinition_T::p_wg
    >
struct GpsMeasurement : public GpsMeasurementBase {
 private:
  typedef GpsMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {
    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    // Get measurements.
    z_p_ = Eigen::Matrix<double, 3, 1>(msg->laititude,
                                       msg->longitude,
                                       msg->altitude);
    z_q_ = Eigen::Quaternion<double>(0,0,0,1);  //not sure yet
    if (distorter_) {
      static double tlast = 0;
      if (tlast != 0) {
        double dt = time - tlast;
        distorter_->Distort(z_p_, z_q_, dt);   //unable to find distorter for GPS now
      }
      tlast = time;
    }

    if (fixed_covariance_) {  // Take fix covariance from reconfigure GUI.
      const double s_zp = n_zp_ * n_zp_;
      R_ =
          (Eigen::Matrix<double, nMeasurements, 1>() << s_zp, s_zp, s_zp)
              .finished().asDiagonal();
    } else {  // Take covariance from sensor.
      R_.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>(
          &msg->position_covariance[0]);  //actually gps' position_covariance has 9 dimensions

      if (msg->header.seq % 100 == 0) {  // Only do this check from time to time.
        if (R_.block<6, 6>(0, 0).determinant() < -0.001)
          MSF_WARN_STREAM_THROTTLE(
              60,
              "The covariance matrix you provided for " "the gps sensor is not positive definite: "<<(R_.block<6, 6>(0, 0)));
      }

      R_.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Zero();

    }
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix<double, 3, 1> z_p_;
  Eigen::Quaternion<double> z_q_;
  double n_zp_
  double n_zq_

  bool measurement_world_sensor_;
  bool fixed_covariance_;
  msf_updates::GpsDistorter::Ptr distorter_;
  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  enum AuxState {
    L = StateLIdx,
    q_ip = StateQipIdx,
    p_ip = StatePipIdx,
    q_wg = StateQwgIdx,
    p_wg = StateQwgIdx
  };

  virtual ~GpsMeasurement() {}
  GpsMeasurement(double n_zp, bool measurement_world_sensor,
                  bool fixed_covariance, bool isabsoluteMeasurement,
                  int sensorID, bool enable_mah_outlier_rejection,
                  double mah_threshold, int fixedstates,
                  msf_updates::GpsDistorter::Ptr distorter =
                      msf_updates::GpsDistorter::Ptr())
      : GpsMeasurementBase(isabsoluteMeasurement, sensorID,
                            enable_mah_outlier_rejection, mah_threshold),
        n_zp_(n_zp),
        measurement_world_sensor_(measurement_world_sensor),
        fixed_covariance_(fixed_covariance),
        distorter_(distorter),
        fixedstates_(fixedstates) {}
  virtual std::string Type() {
    return "gps";
  }

  virtual void CalculateH(
      shared_ptr<EKFState_T> state_in,
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H) {
    const EKFState_T& state = *state_in;  // Get a const ref, so we can read core states.

    H.setZero();

    // Get rotation matrices.
    Eigen::Matrix<double, 3, 3> C_wg = state.Get<StateQwgIdx>()
        .toRotationMatrix();
    Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
        .toRotationMatrix();

    Eigen::Matrix<double, 3, 3> C_pi = state.Get<StateQipIdx>()
        .conjugate().toRotationMatrix();

    // Preprocess for elements in H matrix.
    Eigen::Matrix<double, 3, 1> vecold;

    vecold = (-state.Get<StatePwgIdx>() + state.Get<StateDefinition_T::p>()
        + C_q * state.Get<StatePipIdx>()) * state.Get<StateLIdx>();
    Eigen::Matrix<double, 3, 3> skewold = Skew(vecold);

    Eigen::Matrix<double, 3, 3> ppi_sk = Skew(state.Get<StatePipIdx>());


    // Get indices of states in error vector.
    enum {
      kIdxstartcorr_p = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::p>::value,
      kIdxstartcorr_v = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::v>::value,
      kIdxstartcorr_q = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::q>::value,

      kIdxstartcorr_qwg = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateQwgIdx>::value,
      kIdxstartcorr_qip = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateQipIdx>::value
    };

    // Read the fixed states flags.
    bool scalefix = (fixedstates_ & 1 << StateLIdx);
    bool calibposfix = (fixedstates_ & 1 << StatePipIdx);
    bool calibattfix = (fixedstates_ & 1 << StateQipIdx);
    bool driftwgattfix = (fixedstates_ & 1 << StateQwgIdx);
    bool driftwgposfix = (fixedstates_ & 1 << StatePwgIdx);

    // Set crosscov to zero for fixed states.
    if (scalefix)
      state_in->ClearCrossCov<StateLIdx>();
    if (calibposfix)
      state_in->ClearCrossCov<StatePipIdx>();
    if (calibattfix)
      state_in->ClearCrossCov<StateQipIdx>();
    if (driftwgattfix)
      state_in->ClearCrossCov<StateQwgIdx>();
    if (driftwgposfix)
      state_in->ClearCrossCov<StatePwgIdx>();


    // Construct H matrix.
    // Position:
    H.block<3, 3>(0, kIdxstartcorr_p) = C_wg
        * state.Get<StateLIdx>()(0);  // p

    H.block<3, 3>(0, kIdxstartcorr_q) = -C_wg * C_q * ppi_sk
        * state.Get<StateLIdx>()(0);  // q

    H.block<3, 1>(0, kIdxstartcorr_L) =
        scalefix ?
            Eigen::Matrix<double, 3, 1>::Zero() :
            (C_wg * C_q * state.Get<StatePipIdx>() + C_wg
                    * (-state.Get<StatePwgIdx>()
                        + state.Get<StateDefinition_T::p>())).eval();  // L

    H.block<3, 3>(0, kIdxstartcorr_qwg) =
        driftwgattfix ?
            Eigen::Matrix<double, 3, 3>::Zero() : (-C_wg * skewold).eval();  // q_wg

    H.block<3, 3>(0, kIdxstartcorr_pic) =
        calibposfix ?
            Eigen::Matrix<double, 3, 3>::Zero() :
            (C_wg * C_q * state.Get<StateLIdx>()(0)).eval();  //p_ip


    // TODO (slynen): Check scale commenting
    H.block<3, 3>(0, kIdxstartcorr_pwv) =
        driftwgposfix ?
            Eigen::Matrix<double, 3, 3>::Zero() :
            (-Eigen::Matrix<double, 3, 3>::Identity()
            /* * state.Get<StateLIdx>()(0)*/).eval();  //p_wg

    // Attitude.
    H.block<3, 3>(3, kIdxstartcorr_q) = C_pi;  // q

    H.block<3, 3>(3, kIdxstartcorr_qwg) =
        driftwgattfix ?
            Eigen::Matrix<double, 3, 3>::Zero() :
            (C_pi * C_q.transpose()).eval();  // q_wv

    H.block<3, 3>(3, kIdxstartcorr_qip) =
        calibattfix ?
            Eigen::Matrix<double, 3, 3>::Zero() :
            Eigen::Matrix<double, 3, 3>::Identity().eval();  //q_ic

    // This line breaks the filter if a position sensor in the global frame is
    // available or if we want to set a global yaw rotation.
    H.block<1, 1>(6, kIdxstartcorr_qwg + 2) = Eigen::Matrix<double, 1, 1>::
    Constant(driftwgattfix ? 0.0 : 1.0); // fix vision world yaw drift because unobservable otherwise (see PhD Thesis)

  }

  /**
   * The method called by the msf_core to apply the measurement represented by
   * this object
   */
  virtual void Apply(shared_ptr<EKFState_T> state_nonconst_new,
                     msf_core::MSF_Core<EKFState_T>& core) {

    if (isabsolute_) {  // Does this measurement refer to an absolute measurement,
      // or is is just relative to the last measurement.
      // Get a const ref, so we can read core states
      const EKFState_T& state = *state_nonconst_new;
      // init variables
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
      Eigen::Matrix<double, nMeasurements, 1> r_old;

      CalculateH(state_nonconst_new, H_new);

      // Get rotation matrices.
      Eigen::Matrix<double, 3, 3> C_wg = state.Get<StateQwgIdx>()

          .conjugate().toRotationMatrix();
      Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
          .conjugate().toRotationMatrix();

      // Construct residuals.
      // Position.
      r_old.block<3, 1>(0, 0) = z_p_
          - (C_wg.transpose()
              * (-state.Get<StatePwgIdx>()
                  + state.Get<StateDefinition_T::p>()
                  + C_q.transpose() * state.Get<StatePipIdx>()))
              * state.Get<StateLIdx>();

      // Attitude.
      Eigen::Quaternion<double> q_err;
      q_err = (state.Get<StateQwgIdx>()
          * state.Get<StateDefinition_T::q>()
          * state.Get<StateQipIdx>()).conjugate() * z_q_;
      r_old.block<3, 1>(3, 0) = q_err.vec() / q_err.w() * 2;
      // Vision world yaw drift.
      q_err = state.Get<StateQwgIdx>();

      r_old(6, 0) = -2 * (q_err.w() * q_err.z() + q_err.x() * q_err.y())
          / (1 - 2 * (q_err.y() * q_err.y() + q_err.z() * q_err.z()));

      if (!CheckForNumeric(r_old, "r_old")) {
        MSF_ERROR_STREAM("r_old: "<<r_old);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(H_new, "H_old")) {
        MSF_ERROR_STREAM("H_old: "<<H_new);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(R_, "R_")) {
        MSF_ERROR_STREAM("R_: "<<R_);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }

      // Call update step in base class.
      this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                        R_);
    } else {
      // Init variables: Get previous measurement.
      shared_ptr < msf_core::MSF_MeasurementBase<EKFState_T> > prevmeas_base =
          core.GetPreviousMeasurement(this->time, this->sensorID_);

      if (prevmeas_base->time == msf_core::constants::INVALID_TIME) {
        MSF_WARN_STREAM(
            "The previous measurement is invalid. Could not apply measurement! " "time:"<<this->time<<" sensorID: "<<this->sensorID_);
        return;
      }

      // Make this a gps measurement.
      shared_ptr<GpsMeasurement> prevmeas = dynamic_pointer_cast
          < GpsMeasurement > (prevmeas_base);
      if (!prevmeas) {
        MSF_WARN_STREAM(
            "The dynamic cast of the previous measurement has failed. "
            "Could not apply measurement");
        return;
      }

      // Get state at previous measurement.
      shared_ptr<EKFState_T> state_nonconst_old = core.GetClosestState(
          prevmeas->time);

      if (state_nonconst_old->time == msf_core::constants::INVALID_TIME) {
        MSF_WARN_STREAM(
            "The state at the previous measurement is invalid. Could "
            "not apply measurement");
        return;
      }

      // Get a const ref, so we can read core states.
      const EKFState_T& state_new = *state_nonconst_new;
      const EKFState_T& state_old = *state_nonconst_old;

      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new,
          H_old;
      Eigen::Matrix<double, nMeasurements, 1> r_new, r_old;

      CalculateH(state_nonconst_old, H_old);

      H_old *= -1;

      CalculateH(state_nonconst_new, H_new);

      //TODO (slynen): check that both measurements have the same states fixed!
      Eigen::Matrix<double, 3, 3> C_wg_old, C_wg_new;
      Eigen::Matrix<double, 3, 3> C_q_old, C_q_new;

      C_wg_new = state_new.Get<StateQwgIdx>().conjugate().toRotationMatrix();
      C_q_new = state_new.Get<StateDefinition_T::q>().conjugate()
          .toRotationMatrix();

      C_wg_old = state_old.Get<StateQwgIdx>().conjugate().toRotationMatrix();
      C_q_old = state_old.Get<StateDefinition_T::q>().conjugate()
          .toRotationMatrix();

      // Construct residuals.
      // Position:
      Eigen::Matrix<double, 3, 1> diffprobpos = (C_wg_new.transpose()
          * (-state_new.Get<StatePwgIdx>() + state_new.Get<StateDefinition_T::p>()
              + C_q_new.transpose() * state_new.Get<StatePipIdx>()))
          * state_new.Get<StateLIdx>() - (C_wg_old.transpose()
          * (-state_old.Get<StatePwgIdx>() + state_old.Get<StateDefinition_T::p>()
              + C_q_old.transpose() * state_old.Get<StatePipIdx>()))
              * state_old.Get<StateLIdx>();


      Eigen::Matrix<double, 3, 1> diffmeaspos = z_p_ - prevmeas->z_p_;

      r_new.block<3, 1>(0, 0) = diffmeaspos - diffprobpos;

      // Attitude:
      Eigen::Quaternion<double> diffprobatt = (state_new.Get<StateQwgIdx>()
          * state_new.Get<StateDefinition_T::q>()
          * state_new.Get<StateQipIdx>()).conjugate()
          * (state_old.Get<StateQwgIdx>()
              * state_old.Get<StateDefinition_T::q>()
              * state_old.Get<StateQipIdx>());

      Eigen::Quaternion<double> diffmeasatt = z_q_.conjugate() * prevmeas->z_q_;

      Eigen::Quaternion<double> q_err;
      q_err = diffprobatt.conjugate() * diffmeasatt;

      r_new.block<3, 1>(3, 0) = q_err.vec() / q_err.w() * 2;
      // Vision world yaw drift.
      q_err = state_new.Get<StateQwgIdx>();

      r_new(6, 0) = -2 * (q_err.w() * q_err.z() + q_err.x() * q_err.y())
          / (1 - 2 * (q_err.y() * q_err.y() + q_err.z() * q_err.z()));

      if (!CheckForNumeric(r_old, "r_old")) {
        MSF_ERROR_STREAM("r_old: "<<r_old);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(H_new, "H_old")) {
        MSF_ERROR_STREAM("H_old: "<<H_new);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(R_, "R_")) {
        MSF_ERROR_STREAM("R_: "<<R_);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
      }

      // Call update step in base class.
      this->CalculateAndApplyCorrectionRelative(state_nonconst_old,
                                                state_nonconst_new, core, H_old,
                                                H_new, r_new, R_);

    }
  }
};

}  // namespace msf_gps_sensor
}  // namespace msf_updates
#endif  // GPS_MEASUREMENT_HPP_
