#include <nuclear>

#include "/home/steve/NUSimulator/module/simulation/CameraSimulator/src/CameraSimulator.h"
int main(int argc, char** argv) {

    NUClear::PowerPlant::Configuration config;
    config.threadCount = 4;

    NUClear::PowerPlant plant(config, argc, const_cast<const char**>(argv));
    std::cout << "Installing " << "simulation::CameraSimulator" << std::endl;
    plant.install<module::simulation::CameraSimulator>();

    plant.start();
    return 0;
}
