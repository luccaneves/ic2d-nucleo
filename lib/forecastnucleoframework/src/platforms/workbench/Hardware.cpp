#ifdef TARGET_STM32F4

#include <forecast/platforms/workbench/Hardware.hpp>
#include <utility/math.hpp>

#include <eigen.h>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/QR>

// hardware configurations
#include <forecast/platforms/workbench/config/AB_encoder_environment.h>
#include <forecast/platforms/workbench/config/AB_encoder_motor.h>
#include <forecast/platforms/workbench/config/AB_encoder_motor_environment.h>
#include <forecast/platforms/workbench/config/analog_torque_sensor.h>
#include <forecast/platforms/workbench/config/escon_motor.h>
#include <forecast/platforms/workbench/config/escon_motor_env.h>
#include <forecast/platforms/workbench/config/spring.h>

using namespace Eigen;


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

  //if (not torqueSensor2Init())
    //return Status::TORQUE_SENSOR_2_INIT_ERR;

    if (not pressureSensorAInit())
      return Status::PRESSURE_SENSOR_A_INIT_ERR;

    if (not pressureSensorBInit())
      return Status::PRESSURE_SENSOR_B_INIT_ERR;

    if (not pressureSensorSInit())
      return Status::PRESSURE_SENSOR_S_INIT_ERR;

    if (not pressureSensorTInit())
      return Status::PRESSURE_SENSOR_T_INIT_ERR;

  //load_cell2_sensor = new AnalogInput(PC_1);

  //auto enabled = torque_sensor->enable();
  
  lowPassTauSensor = utility::AnalogFilter::getLowPassFilterHz(40.0f);
  lowPassTauSensor->clean();

  lowPassLoacCell2 = utility::AnalogFilter::getLowPassFilterHz(40.0f);
  lowPassLoacCell2->clean();

  lowPassDX1 = utility::AnalogFilter::getLowPassFilterHz(40.0f);
  lowPassDX1->clean();

  lowPassDX1_E = utility::AnalogFilter::getLowPassFilterHz(40.0f);
  lowPassDX1_E->clean();

  lowPassDDX1 = utility::AnalogFilter::getLowPassFilterHz(40.0f);
  lowPassDDX1->clean();
  
  lowPassDDDX1 = utility::AnalogFilter::getLowPassFilterHz(40.0f);
  lowPassDDDX1->clean();

  lowPassDDX1_E = utility::AnalogFilter::getLowPassFilterHz(40.0f);
  lowPassDDX1_E->clean();

  lowPassDF1 = utility::AnalogFilter::getLowPassFilterHz(40.0f);
  lowPassDF1->clean();

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

int counter = 0;
#define FINITE_DIF_SAMPLING_COUNTER 0

void polyfit(	
  const std::vector<double> &t,
		const std::vector<double> &v,
		std::vector<double> &coeff,
		int order

	     )
{
	// Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial, for exame k = 3 for cubic polynomial
	Eigen::MatrixXd T(t.size(), order + 1);
	Eigen::VectorXd V = Eigen::VectorXd::Map(&v.front(), v.size());
	Eigen::VectorXd result;

	// check to make sure inputs are correct
	assert(t.size() == v.size());
	assert(t.size() >= order + 1);
	// Populate the matrix
	for(size_t i = 0 ; i < t.size(); ++i)
	{
		for(size_t j = 0; j < order + 1; ++j)
		{
			T(i, j) = pow(t.at(i), j);
		}
	}
	
	// Solve for linear least square fit
	result  = T.householderQr().solve(V);
	coeff.resize(order+1);
	for (int k = 0; k < order+1; k++)
	{
		coeff[k] = result[k];
	}

}

void forecast::Hardware::UpdateEncoder() {
  thetaM = encoder_motor->getAngleRad()*0.001; // mm -> m
}

void forecast::Hardware::update(float dt) {
  /* Time update */
  this->dt = dt;
  t = us_ticker_read() / 1e6;
  current_time = get_current_time();

  /* Motor encoder update */
  float theta_m_nofilt = 0;
  theta_m_nofilt = encoder_motor->getAngleRad()*0.001; // mm -> m
  //lowPassDDDX1 = utility::AnalogFilter::getLowPassFilterHz(20.0f);
  thetaM = lowPassDDDX1->process(theta_m_nofilt,dt);
  // thetaM += 1;
  // dthetaM = (thetaM - prev_thetaM) / dt;
  //float dthetaM_NoFilt = -encoder_motor->getVelocityRad(dt)*0.001;
  //dthetaM = lowPassDX1->process(dthetaM_NoFilt, dt);
  //float ddthetaM_NoFilt = (dthetaM - prev_dthetaM) / dt;
  //ddthetaM = lowPassDDX1->process(ddthetaM_NoFilt, dt);
  //prev_thetaM = thetaM;
  //prev_dthetaM = dthetaM;

  /* Environment encoder update */
  thetaE = -encoder_env->getAngleRad()*2;
  // dthetaE = (thetaE - prev_thetaE) / dt;
  //dthetaE = encoder_env->getVelocityRad(dt);
  //ddthetaE = (dthetaE - prev_dthetaE) / dt;
  //prev_thetaE = thetaE;
  //prev_dthetaE = dthetaE;

  /* Environment motor encoder update */
  thetaEnvMotor = encoder_env_motor->getAngleRad();
  dthetaEnvMotor = (thetaEnvMotor - prev_thetaEnvMotor) / dt;
  ddthetaEnvMotor = (dthetaEnvMotor - prev_dthetaEnvMotor) / dt;
  prev_thetaEnvMotor = thetaEnvMotor;
  prev_dthetaEnvMotor = dthetaEnvMotor;


  //Diferencas finitas usando mais pontos para calculo das derivadas
  if(counter == FINITE_DIF_SAMPLING_COUNTER){
    float dthetaM_NoFilt = (2.45*thetaM - 6*prev1_thetaM + 7.5*prev2_thetaM - 6.666666*prev3_thetaM + 3.75*prev4_thetaM - 1.2*prev5_thetaM + 0.1666666*prev6_thetaM)/((FINITE_DIF_SAMPLING_COUNTER + 1)*dt);
    
    dthetaM_NoFilt = (thetaM - prev1_thetaM)/dt;

    dthetaM = lowPassDX1->process(dthetaM_NoFilt, (FINITE_DIF_SAMPLING_COUNTER + 1)*dt);
    //dthetaM = dthetaM_NoFilt;
    
    float ddthetaM_NoFilt = (2.45*dthetaM - 6*prev1_dthetaM + 7.5*prev2_dthetaM - 6.666666*prev3_dthetaM + 3.75*prev4_dthetaM - 1.2*prev5_dthetaM + 0.1666666*prev6_dthetaM)/dt;
    
    //double ddthetaM_NoFilt = (double)(4.511111*((double)thetaM) - 17.4*((double)prev1_thetaM) + 29.25*((double)prev2_thetaM) -28.22222*((double)prev3_thetaM) + 16.5*((double)prev4_thetaM)  -5.4*((double)prev5_thetaM) + 0.761111*((double)prev6_thetaM))/((double) ((FINITE_DIF_SAMPLING_COUNTER + 1)*(FINITE_DIF_SAMPLING_COUNTER + 1)*((double)dt)*((double)dt)));
    
    float dthetaE_NoFilt = (2.45*thetaE - 6*prev1_thetaE + 7.5*prev2_thetaE - 6.66*prev3_thetaE + 3.75*prev4_thetaE - 1.2*prev5_thetaE + 0.16*prev6_thetaE)/((FINITE_DIF_SAMPLING_COUNTER + 1)*dt);
    
    //float ddthetaM_NoFilt = (2.45*dthetaM - 6*prev1_dthetaM + 7.5*prev2_dthetaM - 6.66*prev3_dthetaM + 3.75*prev4_dthetaM - 1.2*prev5_dthetaM + 0.16*prev6_dthetaM)/dt;
    
    //double ddthetaE_NoFilt = (double)(2*((double)thetaE) - 5*((double)prev1_thetaE) + 4*((double)prev2_thetaE) - 1*((double)prev3_thetaE) + 0*((double)prev4_thetaE))/((double) ((FINITE_DIF_SAMPLING_COUNTER + 1)*(FINITE_DIF_SAMPLING_COUNTER + 1)*((double)dt)*((double)dt)));
    
    //Parte do polyfit
    
    /*std::vector<double> time_vec = {t - 6*(FINITE_DIF_SAMPLING_COUNTER + 1)*dt,
    t - 5*(FINITE_DIF_SAMPLING_COUNTER + 1)*dt,
    t - 4*(FINITE_DIF_SAMPLING_COUNTER + 1)*dt,
    t - 3*(FINITE_DIF_SAMPLING_COUNTER + 1)*dt,
     t - 2*(FINITE_DIF_SAMPLING_COUNTER + 1)*dt, 
     t - (FINITE_DIF_SAMPLING_COUNTER + 1)*dt, 
     t};*/
	  // velocity value
	  /*std::vector<double> theta_vec = {prev6_thetaM, prev5_thetaM, prev4_thetaM, prev3_thetaM, prev2_thetaM, prev1_thetaM, thetaM};

    std::vector<double> coeff;

    polyfit(time_vec, theta_vec, coeff, 2);

    double dddthetaM_polyfit = 0;

    double ddthetaM_polyfit = 2*coeff[2];

    double dthetaM_polyfit = 2*coeff[2]*t + coeff[1];*/
    
    //dddthetaM = lowPassDDDX1->process(dddthetaM_polyfit, (FINITE_DIF_SAMPLING_COUNTER + 1)*dt);

    //dthetaM = lowPassDX1->process(dthetaM_NoFilt, (FINITE_DIF_SAMPLING_COUNTER + 1)*dt);

    dthetaE = lowPassDX1_E->process(dthetaE_NoFilt, (FINITE_DIF_SAMPLING_COUNTER + 1)*dt);

    
    //double ddthetaM_NoFilt = (double)(-1)*(4.51*((double)thetaM) - 17.4*((double)prev1_thetaM) + 29.25*((double)prev2_thetaM) - 28.2*((double)prev3_thetaM) + 16.5*((double)prev4_thetaM) - 5.4*((double)prev5_thetaM) + 0.76*((double)prev6_thetaM))/((double) (((double)dt)*((double)dt)));
    
    //double ddthetaM_NoFilt = (double)(-1)*(812*((double)thetaM) - 3132*((double)prev1_thetaM) + 5265*((double)prev2_thetaM) - 5080*((double)prev3_thetaM) + 2970*((double)prev4_thetaM) - 972*((double)prev5_thetaM) + 137*((double)prev6_thetaM))/((double) ((FINITE_DIF_SAMPLING_COUNTER + 1)*(FINITE_DIF_SAMPLING_COUNTER + 1)*180*((double)dt)*((double)dt)));
    

    //double ddthetaM_NoFilt = (double)(-1)*(4*((double)thetaM) - 17*((double)prev1_thetaM) + 29*((double)prev2_thetaM) - 28*((double)prev3_thetaM) + 16*((double)prev4_thetaM) - 5*((double)prev5_thetaM) + 1*((double)prev6_thetaM))/((double) (((double)dt)*((double)dt)));
    
    //float ddthetaM_NoFilt = (dthetaM - prev_dthetaM) / dt;
    //ddthetaE = lowPassDDX1_E->process((float)ddthetaE_NoFilt, (FINITE_DIF_SAMPLING_COUNTER + 1)*dt);

    ddthetaM = lowPassDDX1->process((float)ddthetaM_NoFilt, (FINITE_DIF_SAMPLING_COUNTER + 1)*dt);
    //ddthetaM = ddthetaM_NoFilt;

    prev6_thetaM = prev5_thetaM;
    prev5_thetaM = prev4_thetaM;
    prev4_thetaM = prev3_thetaM;
    prev3_thetaM = prev2_thetaM;
    prev2_thetaM = prev1_thetaM;
    prev1_thetaM = thetaM;

    prev6_dthetaM = prev5_dthetaM;
    prev5_dthetaM = prev4_dthetaM;
    prev4_dthetaM = prev3_dthetaM;
    prev3_dthetaM = prev2_dthetaM;
    prev2_dthetaM = prev1_dthetaM;
    prev1_dthetaM = dthetaM;


    prev6_thetaE = prev5_thetaE;
    prev5_thetaE = prev4_thetaE;
    prev4_thetaE = prev3_thetaE;
    prev3_thetaE = prev2_thetaE;
    prev2_thetaE = prev1_thetaE;
    prev1_thetaE = thetaE;

    prev6_dthetaE = prev5_dthetaE;
    prev5_dthetaE = prev4_dthetaE;
    prev4_dthetaE = prev3_dthetaE;
    prev3_dthetaE = prev2_dthetaE;
    prev2_dthetaE = prev1_dthetaE;
    prev1_dthetaE = dthetaE;

    prev_thetaM = thetaM;
    prev_dthetaM = dthetaM;

    prev_thetaE = thetaE;
    prev_dthetaE = dthetaE;

    counter = 0;
  }
  else{
    counter++;
  }

  

  /* Control motor update  (from Escon feedback) */

  //TODO: Voltar
  float tauM_nofilt = control_motor->getTorqueFeedback();
  tauM = lowPassDDX1_E->process(tauM_nofilt,dt);
  
  //TODO: ATENÇÃO, VOLTAR
  //tauM = control_motor->getCurrentFeedback();
  
  dtauM = (tauM - prev_tauM) / dt;
  ddtauM = (dtauM - prev_dtauM) / dt;
  prev_tauM = tauM;
  prev_dtauM = dtauM;

  prev6_tauM = prev5_tauM;
  prev5_tauM = prev4_tauM;
  prev4_tauM = prev3_tauM;
  prev3_tauM = prev2_tauM;
  prev2_tauM = prev1_tauM;
  prev1_tauM = tauM;


  //Diferencas finitas usando mais pontos para calculo das derivadas
  /*dtauM = (2.28*tauM - 5*prev1_tauM + 5*prev2_tauM - 3.33*prev3_tauM + 1.25*prev4_tauM - 0.2*prev5_tauM)/dt;
  ddtauM = (2.28*dtauM - 5*prev1_dtauM + 5*prev2_dtauM - 3.33*prev3_dtauM + 1.25*prev4_dtauM - 0.2*prev5_dtauM)/dt;

  prev5_tauM = prev4_tauM;
  prev4_tauM = prev3_tauM;
  prev3_tauM = prev2_tauM;
  prev2_tauM = prev1_tauM;
  prev1_tauM = tauM;


  prev5_dtauM = prev4_dtauM;
  prev4_dtauM = prev3_dtauM;
  prev3_dtauM = prev2_dtauM;
  prev2_dtauM = prev1_dtauM;
  prev1_dtauM = dtauM;

  prev_tauM = tauM;
  prev_dtauM = dtauM;*/





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
  float tau_sensors_nofilt = 0;

  // Read Load Cell 1 - Board voltage from USB: 3.324 V
  float signed_voltage = torque_sensor->read_average_float() * 3.324f - center_voltage;
  if (signed_voltage >= 0.00f) {
    amplitude_voltage = 3.324 - center_voltage;
    tau_sensors_nofilt = signed_voltage/amplitude_voltage * LOADCELL_250_RANGE*1.25f;
  } else{
    amplitude_voltage = center_voltage - 0.00;
    tau_sensors_nofilt = signed_voltage/amplitude_voltage * LOADCELL_250_RANGE*1.25f;
  }

  //float tauSensor_filt = lowPassTauSensor->process(tauSensor, dt);
  //tauSensor = tauSensor_filt - 181.0f; // Bias in Newton, hydraulic tests 2023-07-19
  //tauSensor = tauSensor_filt + 95.0f; // Bias in Newton, hydraulic tests 2023-09-11
  
  tau_sensors_nofilt = tau_sensors_nofilt; 
  
  tauSensor = lowPassTauSensor->process(tau_sensors_nofilt, dt);
  //tauSensor = tau_sensors_nofilt;

  //tauSensor = tau_sensors_nofilt; //TODO: Voltar
  //float dtauSensor_NoFilt = (tauSensor - prev_tauSensor) / dt;
  //dtauSensor = lowPassDF1->process(dtauSensor_NoFilt, dt);
  ddtauSensor = (dtauSensor - prev_dtauSensor) / dt;

  //float dtauSensor_NoFilt = (tauSensor - prev_tauSensor) / dt;
  //dtauSensor = lowPassDF1->process(dtauSensor_NoFilt, dt);
  float dtauSensor_NoFilt  = (2.45*tauSensor - 6*prev1_tauSensor + 7.5*prev2_tauSensor - 6.66*prev3_tauSensor + 3.75*prev4_tauSensor - 1.2*prev5_tauSensor + 0.16*prev6_tauSensor)/dt;
  dtauSensor_NoFilt = (tauSensor - prev1_tauSensor)/dt;
  dtauSensor = lowPassDF1->process(dtauSensor_NoFilt, dt);
  //dtauSensor = dtauSensor_NoFilt;

  //ddtauSensor = (2.28*dtauSensor - 5*prev1_dtauSensor + 5*prev2_dtauSensor - 3.33*prev3_dtauSensor + 1.25*prev4_dtauSensor - 0.2*prev5_dtauSensor)/dt;

  prev6_tauSensor = prev5_tauSensor;
  prev5_tauSensor = prev4_tauSensor;
  prev4_tauSensor = prev3_tauSensor;
  prev3_tauSensor = prev2_tauSensor;
  prev2_tauSensor = prev1_tauSensor;
  prev1_tauSensor = tauSensor;


  prev5_dtauSensor = prev4_dtauSensor;
  prev4_dtauSensor = prev3_dtauSensor;
  prev3_dtauSensor = prev2_dtauSensor;
  prev2_dtauSensor = prev1_dtauSensor;
  prev1_dtauSensor = dtauSensor;

  prev_tauSensor = tauSensor;
  prev_dtauSensor = dtauSensor;

  

  // Chambers pressure reading
  pressureSensorA = pressure_sensor_a->read_average_float() * PRESSURE_RANGE;
  pressureSensorB = pressure_sensor_b->read_average_float() * PRESSURE_RANGE;
  pressureSensorS = pressure_sensor_s->read_average_float() * PRESSURE_RANGE;
  pressureSensorT = pressure_sensor_t->read_average_float() * PRESSURE_RANGE;


  // Read Load Cell 2 
  /*
  float lc2_signed_voltage = 0;
  float lc2_signed_voltage = load_cell2_sensor->read_average_float() * 3.324f - 1.749;
  if (lc2_signed_voltage >= 0.00f) {
    amplitude_voltage = 3.324 -center_voltage;
    tauS = lc2_signed_voltage/amplitude_voltage * LOADCELL_5K_RANGE;
  } else{
    amplitude_voltage = center_voltage - 0.00;
   tauS = lc2_signed_voltage/amplitude_voltage * LOADCELL_5K_RANGE;
  }
  float tauS_filt = lowPassLoacCell2->process(-tauS, dt);
  tauS = tauS_filt; // Bias in Newton, hydraulic tests 2023-09-11

}*/
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
  //tauSensor = 0;

  //lowPassTauSensor->clean();
  lowPassLoacCell2->clean();

  lowPassDF1->clean();
  lowPassDX1->clean();
  lowPassDDX1->clean();
  lowPassDDDX1->clean();

  lowPassDX1_E->clean();
  lowPassDDX1_E->clean();

  prev6_dthetaM = 0;
  prev5_dthetaM = 0;
  prev4_dthetaM = 0;
  prev3_dthetaM = 0;
  prev2_dthetaM = 0;
  prev1_dthetaM = 0;

  prev6_thetaM = 0;
  prev5_thetaM = 0;
  prev4_thetaM = 0;
  prev3_thetaM = 0;
  prev2_thetaM = 0;
  prev1_thetaM = 0;

  prev6_dthetaE = 0;
  prev5_dthetaE = 0;
  prev4_dthetaE = 0;
  prev3_dthetaE = 0;
  prev2_dthetaE = 0;
  prev1_dthetaE = 0;

  prev6_thetaE = 0;
  prev5_thetaE = 0;
  prev4_thetaE = 0;
  prev3_thetaE = 0;
  prev2_thetaE = 0;
  prev1_thetaE = 0;

  prev6_tauSensor = 0;
  prev5_tauSensor = 0;
  prev4_tauSensor = 0;
  prev3_tauSensor = 0;
  prev2_tauSensor = 0;
  prev1_tauSensor = 0;

  //tauSensor = 0;
  //tauS = 0;
  wait_ms(500);
}


#endif // TARGET_STM32F4