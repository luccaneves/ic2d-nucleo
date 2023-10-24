#ifdef TARGET_STM32F4

#include <forecast/platforms/workbench/Hardware.hpp>
#include <utility/math.hpp>
// hardware configurations
#include <forecast/platforms/workbench/config/AB_encoder_environment.h>
#include <forecast/platforms/workbench/config/AB_encoder_motor.h>
#include <forecast/platforms/workbench/config/AB_encoder_motor_environment.h>
#include <forecast/platforms/workbench/config/analog_torque_sensor.h>
#include <forecast/platforms/workbench/config/escon_motor.h>
#include <forecast/platforms/workbench/config/escon_motor_env.h>
#include <forecast/platforms/workbench/config/spring.h>

forecast::Status forecast::Hardware::init() {
  wait(2.0);

  if (not motorEncoderInit())
    return Status::MOTOR_ENCODER_INIT_ERR;

  if (not envEncoderInit())
    return Status::ENV_ENCODER_INIT_ERR;

  if (not envMotorEncoderInit())
    return Status::ENV_MOTOR_ENCODER_INIT_ERR;

  if (not motorControlInit())
    return Status::CONTROL_MOTOR_INIT_ERR;
  control_motor->setTorque(0.f);

  if (not motorEnvironmentInit())
    return Status::ENV_MOTOR_INIT_ERR;
  env_motor->setTorque(0.f);

  if (not torqueSensorInit())
    return Status::TORQUE_SENSOR_INIT_ERR;

  if (not torqueSensor2Init())
    return Status::TORQUE_SENSOR_2_INIT_ERR;

  if (not pressureSensorAInit())
    return Status::PRESSURE_SENSOR_A_INIT_ERR;

  if (not pressureSensorBInit())
    return Status::PRESSURE_SENSOR_B_INIT_ERR;

  if (not pressureSensorSInit())
    return Status::PRESSURE_SENSOR_S_INIT_ERR;

  if (not pressureSensorTInit())
    return Status::PRESSURE_SENSOR_T_INIT_ERR;

  //load_cell2_sensor = new AnalogInput(PC_1);
  
  lowPassTauSensor = utility::AnalogFilter::getLowPassFilterHz(10.0f);
  lowPassTauSensor->clean();

  lowPassLoacCell2 = utility::AnalogFilter::getLowPassFilterHz(2.0f);
  lowPassLoacCell2->clean();

  return Status::NO_ERROR;
}

bool forecast::Hardware::motorEncoderInit() {
  encoder_motor =
      new DigitalEncoderAB(motorEncoder::CPR, motorEncoder::GEAR_RATIO);

  /* Set the encoder timer */
  encoder_motor->setTIM1(); // read position
  encoder_motor->setTIM3(); // read velocity

  return true;
}

bool forecast::Hardware::envEncoderInit() {
  encoder_env = new DigitalEncoderAB(envEncoder::CPR, envEncoder::GEAR_RATIO);

  /* Set the encoder timer */
  encoder_env->setTIM8(); // read position
  encoder_env->setTIM4(); // read velocity

  return true;
}

bool forecast::Hardware::envMotorEncoderInit() {
  encoder_env_motor =
      new DigitalEncoderAB(envMotorEncoder::CPR, envMotorEncoder::GEAR_RATIO);

  /* Set the encoder timer */
  encoder_env_motor->setTIM2(); // read position

  return true;
}

bool forecast::Hardware::motorControlInit() {
  MotorConfiguration conf;
  conf.enable = motorControl::MOTOR_ENABLE_PIN;
  conf.currFeedback = motorControl::MOTOR_CURRENT_FEEDBACK_PIN;
  conf.analog = motorControl::MOTOR_ANALOG_PIN;
  conf.offset_bias = motorControl::offset_bias;
  conf.amp_scale = motorControl::amp_scale; 

  /* Control motor */
  control_motor = new EsconMotor(conf, motorControl::KT, motorControl::JM, motorControl::MAX_CURR);

  return control_motor != nullptr;
}

bool forecast::Hardware::motorEnvironmentInit() {
  MotorConfiguration conf;
  conf.enable = envMotor::MOTOR_ENABLE_PIN;
  conf.currFeedback = envMotor::MOTOR_CURRENT_FEEDBACK_PIN;
  conf.analog = envMotor::MOTOR_ANALOG_PIN;
  conf.offset_bias = envMotor::offset_bias;
  conf.amp_scale = envMotor::amp_scale; 

  /* Environment motor */
  env_motor = new EsconMotor(conf, envMotor::KT, envMotor::JM, envMotor::MAX_CURR);

  return env_motor != nullptr;
}

bool forecast::Hardware::torqueSensorInit() {
  torque_sensor =
      new AnalogInput(TORQUE_SENSOR_PIN, ADC_PCLK2, ADC_Right, ADC_15s, ADC_12b,
                      ADC_Continuous, ADC_Dma, DMA_BUFFER_SIZE);

  /* Enable the ADC - In continous mode the ADC start is done automatically */
  //auto enabled = torque_sensor->enable();
  //tauSensorOffset = torque_sensor->read_average_float() * 3.3f;

  return true;
  }

  bool forecast::Hardware::torqueSensor2Init() {
  load_cell2_sensor =
      new AnalogInput(TORQUE_SENSOR_2_PIN, ADC_PCLK2, ADC_Right, ADC_15s, ADC_12b,
                      ADC_Continuous, ADC_Dma, DMA_BUFFER_SIZE);

  /* Enable the ADC - In continous mode the ADC start is done automatically */
  //auto enabled = load_cell2_sensor->enable();
  //tauSOffset = load_cell2_sensor->read_average_float() * 3.3f;

  return true;
}


bool forecast::Hardware::pressureSensorAInit() {
  pressure_sensor_a =
      new AnalogInput(PRESSURE_SENSOR_A_PIN, ADC_PCLK2, ADC_Right, ADC_15s, ADC_12b,
                      ADC_Continuous, ADC_Dma, DMA_BUFFER_SIZE);

  /* Enable the ADC - In continous mode the ADC start is done automatically */
  //auto enabled = pressure_sensor_a->enable();

  return true;
}

bool forecast::Hardware::pressureSensorBInit() {
  pressure_sensor_b =
      new AnalogInput(PRESSURE_SENSOR_B_PIN, ADC_PCLK2, ADC_Right, ADC_15s, ADC_12b,
                      ADC_Continuous, ADC_Dma, DMA_BUFFER_SIZE);

  /* Enable the ADC - In continous mode the ADC start is done automatically */
  //auto enabled = pressure_sensor_b->enable();

  return true;
}

bool forecast::Hardware::pressureSensorSInit() {
  pressure_sensor_s =
      new AnalogInput(PRESSURE_SENSOR_S_PIN, ADC_PCLK2, ADC_Right, ADC_15s, ADC_12b,
                      ADC_Continuous, ADC_Dma, DMA_BUFFER_SIZE);

  /* Enable the ADC - In continous mode the ADC start is done automatically */
 // auto enabled = pressure_sensor_s->enable();

  return true;
}

bool forecast::Hardware::pressureSensorTInit() {
  pressure_sensor_t =
      new AnalogInput(PRESSURE_SENSOR_T_PIN, ADC_PCLK2, ADC_Right, ADC_15s, ADC_12b,
                      ADC_Continuous, ADC_Dma, DMA_BUFFER_SIZE);

  /* Enable the ADC - In continous mode the ADC start is done automatically */
  auto enabled = pressure_sensor_t->enable();

  return true;
}


void forecast::Hardware::update(float dt) {
  /* Time update */
  this->dt = dt;
  t = us_ticker_read() / 1e6;
  current_time = get_current_time();

  /* Motor encoder update */
  thetaM = -encoder_motor->getAngleRad()*0.001; // mm -> m
  // thetaM += 1;
  // dthetaM = (thetaM - prev_thetaM) / dt;
  dthetaM = -encoder_motor->getVelocityRad(dt)*0.001;
  ddthetaM = (dthetaM - prev_dthetaM) / dt;
  prev_thetaM = thetaM;
  prev_dthetaM = dthetaM;

  /* Environment encoder update */
  thetaE = encoder_env->getAngleRad();
  // dthetaE = (thetaE - prev_thetaE) / dt;
  dthetaE = encoder_env->getVelocityRad(dt);
  ddthetaE = (dthetaE - prev_dthetaE) / dt;
  prev_thetaE = thetaE;
  prev_dthetaE = dthetaE;

  /* Environment motor encoder update */
  thetaEnvMotor = encoder_env_motor->getAngleRad();
  dthetaEnvMotor = (thetaEnvMotor - prev_thetaEnvMotor) / dt;
  ddthetaEnvMotor = (dthetaEnvMotor - prev_dthetaEnvMotor) / dt;
  prev_thetaEnvMotor = thetaEnvMotor;
  prev_dthetaEnvMotor = dthetaEnvMotor;

  /* Control motor update  (from Escon feedback) */
  tauM = control_motor->getTorqueFeedback();
  dtauM = (tauM - prev_tauM) / dt;
  ddtauM = (dtauM - prev_dtauM) / dt;
  prev_tauM = tauM;
  prev_dtauM = dtauM;

  /* Environment motor update  (from Escon feedback) *//*
  tauE = env_motor->getTorqueFeedback();
  dtauE = (tauE - prev_tauE) / dt;
  ddtauE = (dtauE - prev_dtauE) / dt;
  prev_tauE = tauE;
  prev_dtauE = dtauE;*/

  // /* Spring torque update */
  /*
  tauS = KS * (thetaM - thetaE);
  dtauS = (tauS - prev_tauS) / dt;
  ddtauS = (dtauS - prev_dtauS) / dt;
  prev_tauS = tauS;
  prev_dtauS = dtauS; 
  */


  float amplitude_voltage(1);
  float center_voltage(LOADCELL_250_OFFSET);
  //float center_voltage(LOADCELL_5K_OFFSET);
  

  // Read Load Cell 1 - Board voltage from USB: 3.324 V
  float signed_voltage = torque_sensor->read_average_float() * 3.324f - center_voltage;
  if (signed_voltage >= 0.00f) {
    amplitude_voltage = 3.324 - center_voltage;
    tauSensor = signed_voltage/amplitude_voltage * LOADCELL_250_RANGE;
  } else{
    amplitude_voltage = center_voltage - 0.00;
    tauSensor = signed_voltage/amplitude_voltage * LOADCELL_250_RANGE;
  }

  float tauSensor_filt = lowPassTauSensor->process(tauSensor, dt);
  //tauSensor = tauSensor_filt - 181.0f; // Bias in Newton, hydraulic tests 2023-07-19
  //tauSensor = tauSensor_filt + 95.0f; // Bias in Newton, hydraulic tests 2023-09-11
  tauSensor = tauSensor_filt; // Bias in Newton, linmot tests 2023-09-26

  dtauSensor = (tauSensor - prev_tauSensor) / dt;
  ddtauSensor = (dtauSensor - prev_dtauSensor) / dt;
  prev_tauSensor = tauSensor;
  prev_dtauSensor = dtauSensor;

  // Chambers pressure reading
  pressureSensorA = pressure_sensor_a->read_average_float() * PRESSURE_RANGE;
  pressureSensorB = pressure_sensor_b->read_average_float() * PRESSURE_RANGE;
  pressureSensorS = pressure_sensor_s->read_average_float() * PRESSURE_RANGE;
  pressureSensorT = pressure_sensor_t->read_average_float() * PRESSURE_RANGE;


  // Read Load Cell 2
  //float lc2_signed_voltage = 0;
  float lc2_signed_voltage = load_cell2_sensor->read_average_float() * 3.324f - 1.749;
  if (lc2_signed_voltage >= 0.00f) {
    amplitude_voltage = 3.324 -center_voltage;
    tauS = lc2_signed_voltage/amplitude_voltage * LOADCELL_5K_RANGE;
  } else{
    amplitude_voltage = center_voltage - 0.00;
    tauS = lc2_signed_voltage/amplitude_voltage * LOADCELL_5K_RANGE;
  }
  //float tauS_filt = lowPassLoacCell2->process(-tauS, dt);

  tauS = (-tauS) - 88.0f; // Bias in Newton, hydraulic tests 2023-09-11
}

void forecast::Hardware::home() 
{
  // resetting the encoders to make the board think it's in the starting
  // position
  encoder_motor->reset(0);
  encoder_env->reset(0);
  encoder_env_motor->reset(0);

  // resetting actuators output:
  control_motor->setTorque(0);
  env_motor->setTorque(0);

  lowPassTauSensor->clean();
  tauSensor = 0;

  //lowPassTauSensor->clean();
  //lowPassLoacCell2->clean();

  //tauSensor = 0;
  //tauS = 0;
  wait_ms(500);
}


#endif // TARGET_STM32F4