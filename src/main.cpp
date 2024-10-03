/**
 * @file main.cpp
 * @author AltairLab
 * @brief Main file with App and Hardware classes instances.
 * @version 0.1
 * @date 2023-06-01
 *
 * @copyright Copyright (c) 2023
 * @details App and Hardware classes instances. Refgens and controllers are added here.
 */

#include <mbed.h>
#include <forecast/App.hpp>
#include <forecast/platforms/workbench/Hardware.hpp>

/** Controllers Headers */
#include <forecast/controllers/PositionPID.hpp>
#include <forecast/controllers/VelocityPID.hpp>
#include <forecast/controllers/ForcePID.hpp>
#include <forecast/controllers/ForcePID_DOB.hpp>
#include <forecast/controllers/ForcePID_DOB_Hyd.hpp>
#include <forecast/controllers/ForcePID_DOB_Hyd_Lin.hpp>
#include <forecast/controllers/NoController.hpp>
#include <forecast/controllers/EnvRenderingControl.hpp>
#include <forecast/controllers/AdmittanceControl.hpp>
#include <forecast/controllers/ImpedanceControl.hpp>
#include <forecast/controllers/Bypass.hpp>
#include <forecast/controllers/ForcePID_VC.hpp>
#include <forecast/controllers/ForcePID_VC_LinMot.hpp>
#include <forecast/controllers/FeedbackLin.hpp>
#include <forecast/controllers/Imp_Dob_LinMot_4000.hpp>
#include <forecast/controllers/SlidingMode.hpp>
#include <forecast/controllers/ImpedanceHyd.hpp>
#include <forecast/controllers/Impedance_Admitance_Switch.hpp>
#include <forecast/controllers/Adaptative.hpp>

#include <forecast/controllers/CompliantHelio.hpp>
#include <forecast/controllers/CompliantHelioFL.hpp>
#include <forecast/controllers/CompliantHelioAdapt.hpp>
#include <forecast/controllers/ImpAdapt.hpp>
#include <forecast/controllers/ImpSlide.hpp>


/** Refgen Headers */
#include <forecast/reference_generators/ConstantRefGen.hpp>
#include <forecast/reference_generators/SmoothStep.hpp>
#include <forecast/reference_generators/Sinusoid.hpp>
#include <forecast/reference_generators/Ramp.hpp>
#include <forecast/reference_generators/Sweep.hpp>


#include <debug.hpp>
#include <signal.h>

using namespace forecast;

extern "C" void abort_handler(int signal_number)
{
    DEBUG_INFO("SIGNAL HANDLER CALLED\n");
}

int main()
{
    signal(SIGABRT, &abort_handler);
    App app;
    DEBUG_INFO("App constructed\n");
    Hardware hw(app);
    hw.init();
    app.set_hw(&hw);

    DEBUG_INFO("hw constructed\n");

    app.get_ref_gen_factory().add("constant", make_constant_ref_gen_builder());
    app.get_ref_gen_factory().add("Smooth Step", make_smoothstep_ref_gen_builder());
    app.get_ref_gen_factory().add("Ramp", make_ramp_ref_gen_builder());
    app.get_ref_gen_factory().add("Sinusoid", make_sinusoid_ref_gen_builder());
    app.get_ref_gen_factory().add("Sweep", make_sweep_ref_gen_builder());

    app.get_controller_factory().add("PositionPID", make_Position_PID_builder());
    //app.get_controller_factory().add("VelocityPID", make_Velocity_PID_builder());
    app.get_controller_factory().add("ForcePID", make_Force_PID_builder());
    //app.get_controller_factory().add("Environment", make_EnvRenderingController_damped());
    app.get_controller_factory().add("NoController", make_no_controller_builder());
    app.get_controller_factory().add("Admittance", make_admittance_control_builder());
    app.get_controller_factory().add("Impedance", make_impedance_control_builder());
    app.get_controller_factory().add("Bypass", make_Bypass_builder());
    //app.get_controller_factory().add("ForcePIDVC", make_Force_PID_VC_builder());
    app.get_controller_factory().add("FeedbackLin", make_feedback_lin_builder());
    app.get_controller_factory().add("DOB_5000Hz_LinMot", make_Force_PID_DOB_builder());
    //app.get_controller_factory().add("Linmot_VC", make_ForcePID_VC_LinMot_builder());
    app.get_controller_factory().add("DOB HYD (5000Hz)", make_Force_PID_DOB_hyd_builder());
    app.get_controller_factory().add("DOB HYD LIN (5000Hz)", make_Force_PID_DOB_hyd_lin_builder());
    app.get_controller_factory().add("Imp_linmot_dob_5000", make_Imp_Dob_LinMot_4000_builder());
    app.get_controller_factory().add("SlidingMode", make_SlidingMode_builder());
    app.get_controller_factory().add("Imp_Hyd", make_ImpedanceHyd_builder()); 
    app.get_controller_factory().add("Adapt", make_Adaptative_builder()); 
    app.get_controller_factory().add("IMP_ADM_SWITCH", make_impedance_admitance_control_builder()); 
    app.get_controller_factory().add("ImpAdapt", make_ImpAdapt_builder()); 
    app.get_controller_factory().add("ImpSLide", make_ImpedanceSlide_builder()); 


    app.get_controller_factory().add("HelioCompliantSlide", make_CompliantHelio_builder()); 
    app.get_controller_factory().add("HelioCompliantFL", make_CompliantHelioFL_builder()); 
    app.get_controller_factory().add("HelioCompliantAdapt", make_CompliantHelioAdapt_builder()); 

    //app.get_operator_factory().add("Sum", make_sum_op_builder());

    DEBUG_INFO("finished with app\n");

    app.run();

    return 0;
}