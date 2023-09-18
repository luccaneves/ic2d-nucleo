#ifndef FORECAST_HARDWARE_HPP
#define FORECAST_HARDWARE_HPP

#ifdef TARGET_STM32F4

#include <hw/DigitalEncoder/DigitalEncoderAB.h>
#include <hw/Motor/EsconMotor.h>
#include <mbed.h>

#include <utility/math.hpp>

#include "../../Status.hpp"

#include "../../IHardware.hpp"

#include <utility/filters/AnalogFilter.hpp>

namespace forecast {
class Hardware : public IHardware {
public:
  /**
   * @brief   Hardware constructor
   */
  Hardware(App &app) : IHardware(app) {
    DEBUG_INFO("Hardware constructor called\n");

    logs["t"] = &current_time;
    logs["dt"] = &dt;

    logs["Control Signal"] = &tauM;
    //logs["dtauM"] = &dtauM;
    //logs["ddtauM"] = &ddtauM;

    //logs["tauE"] = &tauE;
    //logs["dtauE"] = &dtauE;
    //logs["ddtauE"] = &ddtauE;

    //logs["thetaE"] = &tauE;
    //logs["dthetaE"] = &dtauE;
    //logs["ddthetaE"] = &ddtauE;

    logs["F2"] = &tauS;
    logs["dF2"] = &dtauS;
    logs["ddF2"] = &ddtauS;

    logs["F1"] = &tauSensor;
    logs["dF1"] = &dtauSensor;
    logs["ddF1"] = &ddtauSensor;

    logs["x1"] = &thetaM;
    logs["dx1"] = &dthetaM;
    logs["ddx1"] = &ddthetaM;

    logs["x2"] = &thetaE;
    logs["dx2"] = &dthetaE;
    logs["ddx2"] = &ddthetaE;

    logs["pA"] = &pressureSensorA;
    logs["pB"] = &pressureSensorB;
    logs["pS"] = &pressureSensorS;
    logs["pT"] = &pressureSensorT;

    //logs["thetaEnvMotor"] = &thetaEnvMotor;
    //logs["dthetaEnvMotor"] = &dthetaEnvMotor;
    //logs["ddthetaEnvMotor"] = &ddthetaEnvMotor;
  }

  // ~Hardware() {
  //     delete encoder;
  //     delete motor;
  //     delete strain_gauge;
  // }

  /**
   * @brief   Initialization of the Hardware
   */
  Status init();

  virtual inline size_t get_motor_num() const override { return 2; }

  virtual inline size_t get_tau_sensor_num() const override { return 2; }

  virtual inline size_t get_theta_sensor_num() const override { return 3; }

  virtual inline void set_tau_m(size_t index, float torque) override {
    if (index == 0) {
      control_motor->setTorque(torque);
      return;
    }
    if (index == 1) {
      env_motor->setTorque(torque);
      return;
    }

    app.send_error("unknown motor index in set_tau_m");
  }

  /**
   * @brief   Return the hw time t of the last update.
   *
   * @return  t
   */
  virtual inline float get_t() const override { return t; }
  /**
   * @brief   Return the hw dt used in the last update.
   *
   * @return  dt
   */
  virtual inline float get_dt() const override { return dt; }

  /**
   * @brief   Set the start time of the experiment.
   *
   * @param  time The start time of the experiment
   */
  inline void set_start_time(float time) override { start_t = time; }

 /**
   * @brief   Set the duration of the experiment.
   *
   * @param  time The duartion of the experiment
   */
  inline void set_duration_time(float time) override { duration_t = time; }

  /**
   * @brief   Return the start time of the experiment.
   *
   * @return  start_t
   */
  virtual inline float get_start_time() const override { return start_t; }

  /**
   * @brief   Return the hw time t from the start of the experiment.
   *
   * @return  curr_t
   */
  virtual inline float get_current_time() const override { return t - start_t; }

  /**
   * @brief   Return the hw time t from the duration of the experiment.
   *
   * @return  duration_t
   */
  virtual inline float get_duration_time() const override { return duration_t; }

  /**std::make_unique<control::Control>()
   * @brief   Return the torque applied by the motor (current feedback)
   *
   * @return  motorTorqueFeedback
   */
  virtual inline float get_tau_m(size_t motor_idx) const override {
    switch (motor_idx) {
    case 0:
      return tauM;
    case 1:
      return tauE;
    default:
      app.send_error("invalid motor inde in get_tau_m");
      return 0.f;
    }
  }
  virtual inline float get_d_tau_m(size_t motor_idx) const override {
    switch (motor_idx) {
    case 0:
      return dtauM;
    case 1:
      return dtauE;
    default:
      app.send_error("invalid motor inde in get_d_tau_m");
      return 0.f;
    }
  }
  virtual inline float get_dd_tau_m(size_t motor_idx) const override {
    switch (motor_idx) {
    case 0:
      return ddtauM;
    case 1:
      return ddtauE;
    default:
      app.send_error("invalid motor inde in get_dd_tau_m");
      return 0.f;
    }
  }

  /**
   * @brief   Return the torque measured by the spring
   *
   * @return  tau_s
   */
  virtual inline float get_tau_s(size_t sensor_idx) const {
    switch (sensor_idx) {
    case 0: // spring
      return tauS;
    case 1: // sensor
      return tauSensor;
    default:
      app.send_error("invalid sensor index in get_tau_s");
      return 0.f;
    }
  }
  virtual inline float get_d_tau_s(size_t sensor_idx) const {
    switch (sensor_idx) {
    case 0:
      return dtauS;
    case 1:
      return dtauSensor;
    default:
      app.send_error("invalid sensor index in get_d_tau_s");
      return 0.f;
    }
  }
  virtual inline float get_dd_tau_s(size_t sensor_idx) const {
    switch (sensor_idx) {
    case 0:
      return ddtauS;
    case 1:
      return ddtauSensor;
    default:
      app.send_error("invalid sensor index in get_dd_tau_s");
      return 0.f;
    }
  }

  /**
   * @brief   Return the angle radius measured by the encoder of the motor.
   *
   * @return  thetaM
   */
  virtual inline float get_theta(size_t sensor_idx) const {
    switch (sensor_idx) {
    case 0: // motor
      return thetaM;
    case 1: // environment
      return thetaE;
    case 2:
      return thetaEnvMotor;
    default:
      app.send_error("invalid sensor index in get_theta");
      return 0.f;
    }
  }
  virtual inline float get_d_theta(size_t sensor_idx) const {
    switch (sensor_idx) {
    case 0: // motor
      return dthetaM;
    case 1: // environment
      return dthetaE;
    case 2:
      return dthetaEnvMotor;
    default:
      app.send_error("invalid sensor index in get_theta");
      return 0.f;
    }
  }
  virtual inline float get_dd_theta(size_t sensor_idx) const {
    switch (sensor_idx) {
    case 0: // motor
      return ddthetaM;
    case 1: // environment
      return ddthetaE;
    case 2:
      return thetaEnvMotor;
    default:
      app.send_error("invalid sensor index in get_theta");
      return 0.f;
    }
  }

  /**
   * @brief   Return the pressure measured by the sensors
   *
   * @return  pressure
   */
  virtual inline float get_pressure(size_t sensor_idx) const {
    switch (sensor_idx) {
    case 0: // pressure A
      return pressureSensorA;
    case 1: // Pressure B
      return pressureSensorB;
    case 2: // pressure S
      return pressureSensorS;
    case 3: // Pressure T
      return pressureSensorT;
    default:
      app.send_error("invalid sensor index in get_pressure");
      return 0.f;
    }
  }

  virtual inline float get_output() const override { return output; }
  virtual inline float get_d_output() const override { return doutput; }
  virtual inline float get_dd_output() const override { return ddoutput; }

  virtual inline void safety_on() {
    control_motor->setEnable(false);
    env_motor->setEnable(false);
  }

  virtual inline void safety_off() {
    control_motor->setEnable(true);
    env_motor->setEnable(true);
  }

  /**
   * @brief   Update the Hardware by reading the value from the physical hw
   *
   * @param   Torque that has to be given to the motor for actuating the
   * control Torque that has to be given to the environment for the simulation
   *
   * @param   dt is the delta time in seconds to use for the
   * calculations for controls
   */
  // void update(float controlTorque, float envTorque, float dt);
  virtual void update(float dt) override;

  virtual void home() override;

protected:
  bool motorEncoderInit(); /// < Initialize the motor encoder

  bool envEncoderInit(); /// < Initialize the environment encoder

  bool envMotorEncoderInit(); /// < Initialize the environment encoder

  bool motorControlInit(); ///< Initialize the motor

  bool motorEnvironmentInit(); ///< Initialize the motor

  bool torqueSensorInit(); ///< Initialize the torque sensor 1

  bool torqueSensor2Init(); ///< Initialize the torque sensor 2

  bool pressureSensorAInit(); ///< Initialize the pressure sensor a
  
  bool pressureSensorBInit(); ///< Initialize the pressure sensor b

  bool pressureSensorSInit(); ///< Initialize the pressure sensor s
  
  bool pressureSensorTInit(); ///< Initialize the pressure sensor t


  DigitalEncoderAB *encoder_motor = nullptr;     ///< Motor encoder
  DigitalEncoderAB *encoder_env = nullptr;       ///< Environment encoder
  DigitalEncoderAB *encoder_env_motor = nullptr; ///< Environment motor encoder

  EsconMotor *control_motor = nullptr; ///< Motor used for the control
  EsconMotor *env_motor = nullptr;     ///< Motor used for the env. simulation

  AnalogInput *torque_sensor = nullptr; ///< Torque sensor
  AnalogInput *load_cell2_sensor = nullptr; ///< Torque sensor 
  AnalogInput *pressure_sensor_a = nullptr; ///< Pressure sensor a
  AnalogInput *pressure_sensor_b = nullptr; ///< Pressure sensor b
  AnalogInput *pressure_sensor_s = nullptr; ///< Pressure sensor s
  AnalogInput *pressure_sensor_t = nullptr; ///< Pressure sensor t

  utility::AnalogFilter* lowPassTauSensor;
  utility::AnalogFilter* lowPassLoacCell2;

  float t, dt, current_time, duration_t;
  float start_t;

  float tauM;
  float dtauM;
  float ddtauM;
  float tauSensorOffset;

  float tauE;
  float dtauE;
  float ddtauE;

  float tauS;
  float dtauS;
  float ddtauS;
  float tauSOffset;

  float tauSensor;
  float dtauSensor;
  float ddtauSensor;

  float thetaM;
  float dthetaM;
  float ddthetaM;

  float thetaE;
  float dthetaE;
  float ddthetaE;

  float thetaEnvMotor;
  float dthetaEnvMotor;
  float ddthetaEnvMotor;

  float prev_tauM;
  float prev_dtauM;

  float prev_tauE;
  float prev_dtauE;

  float prev_tauS;
  float prev_dtauS;

  float prev_tauSensor;
  float prev_dtauSensor;

  float prev_thetaM;
  float prev_dthetaM;

  float prev_thetaE;
  float prev_dthetaE;

  float prev_thetaEnvMotor;
  float prev_dthetaEnvMotor;

  float output;
  float doutput;
  float ddoutput;

  float pressureSensorA;
  float pressureSensorB;
  float pressureSensorT;
  float pressureSensorS;
};
} // namespace forecast

#endif // TARGET_STM32F4
#endif // FORECAST_HARDWARE_HPP