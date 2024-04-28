#include <forecast/reference_generators/ConstantRefGen.hpp>

forecast::ConstantRefGen::ConstantRefGen(std::vector<float> v) : values(std::move(v)) {
    // ntd;
}

std::vector<float> forecast::ConstantRefGen::process(const IHardware* hw) {
float time = hw->get_current_time();
float duration = hw->get_duration_time();


/*if(time > (duration - 4)){
    return {-1};
}
else*/ 
if (time > 10.0f && time < duration - 10.0f){
        return values;
    } 
else {
        return {0.0};
    }
}
