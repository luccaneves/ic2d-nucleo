#include <forecast/controllers/Bypass.hpp>

using namespace forecast;

Bypass::Bypass() {
  logs.push_back(&reference);
}

float Bypass::process(const IHardware *hw, std::vector<float> ref) {
  reference = ref[0]-0.27; // offset G761 de corrente 0.27, E024 0.17
  *(hw->var1) = ref[0]; //Forca desejada
  return reference;
  

}