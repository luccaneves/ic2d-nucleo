#ifndef POSITION_PID_H
#define POSITION_PID_H

#include <utility/filters/AnalogFilter.hpp>
#include "../Controller.hpp"

namespace forecast
{
    /**
     * @brief PositionPID control class
     **/

    class PositionPID : public Controller
    {
    public:
        /**
         * @brief Construct a new Position P I D object. This constructor initialize,
         * the controller.
         *
         * @param kp
         * @param ki
         * @param kd
         */
        PositionPID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f,float dob_gain = 0);

        virtual float process(const IHardware *hw, std::vector<float> ref) override;

    protected:
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;

        float theta = 0.0f;
        float dtheta = 0.0f;

        float err = 0.0f;
        float derr = 0.0f;
        float ierr = 0.0f;

        float dob_gain = 0;

        float errPast = 0.0f;

        float out;
        float reference = 0.0;

        float last_erro_1 = 0;
        float last_erro_2 = 0;
        float last_erro_3 = 0;
        float last_erro_4 = 0;
        float last_erro_5 = 0;
        float last_erro_6 = 0;

        double prev1_inv_model_exit = 0;
        double prev2_inv_model_exit = 0;
        double prev3_inv_model_exit = 0;
        double prev4_inv_model_exit = 0;
        double prev5_inv_model_exit = 0;


        double prev1_filter_exit = 0;
        double prev2_filter_exit = 0;
        double prev3_filter_exit = 0;
        double prev4_filter_exit = 0;
        double prev5_filter_exit = 0;

        double controller_prev1_tauSensor;
        double controller_prev2_tauSensor;
        double controller_prev3_tauSensor;
        double controller_prev4_tauSensor;
        double controller_prev5_tauSensor;
        double controller_prev6_tauSensor;



        double controller_prev1_tauM;
        double controller_prev2_tauM;
        double controller_prev3_tauM;
        double controller_prev4_tauM;
        double controller_prev5_tauM;
        double controller_prev6_tauM;


        utility::AnalogFilter *lowPassExit;
        utility::AnalogFilter *lowPass;

    };

    inline ControllerFactory::Builder make_Position_PID_builder()
    {
        auto fn = [](std::vector<float> params) -> Controller *
        {
            if (params.size() < 1)
                return nullptr;

            return new PositionPID(params[0], params[1], params[2], params[3]);
        };

        return {fn, {"Kp", "Ki", "Kd","dob_gain"}, {"Reference"}};
    }

} // namespace forecast

#endif // POSITION_PID_H