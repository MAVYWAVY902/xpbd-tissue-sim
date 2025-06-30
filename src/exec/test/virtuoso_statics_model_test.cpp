#include "simobject/VirtuosoArm.hpp"
#include "config/simobject/VirtuosoArmConfig.hpp"

#include "common/types.hpp"

#include <memory>

int main()
{
    // create VirtuosoArm
    Config::VirtuosoArmConfig config("arm1", 
        Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), false, false,
        1.56e-3, 1.14e-3, 1.5383e-2, 5e-3, 1.04e-3, 0.82e-3,
        0, 10e-3, 0, 20e-3,
        Sim::VirtuosoArm::ToolType::SPATULA);

    std::unique_ptr<Sim::VirtuosoArm> arm = config.createObject(nullptr);

    const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames = arm->outerTubeFrames();
    const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = arm->innerTubeFrames();

    std::cout << "=== NO FORCE ===" << std::endl;
    std::cout << "Inner tube tip position: " << arm->tipPosition()[0] << ", " << arm->tipPosition()[1] << ", " << arm->tipPosition()[2] << std::endl;
    std::cout << "Outer tube tip position: " << ot_frames.back().origin()[0] << ", " << ot_frames.back().origin()[1] << ", " << ot_frames.back().origin()[2] << std::endl;

    arm->setTipForce(Vec3r(0,20,10));

    
    // const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames2 = arm->outerTubeFrames();
    std::cout << "\n=== TIP FORCE = (" << arm->tipForce()[0] << ", " << arm->tipForce()[1] << ", " << arm->tipForce()[2] << ") ===" << std::endl;

    std::cout << "Inner tube tip position: " << arm->tipPosition()[0] << ", " << arm->tipPosition()[1] << ", " << arm->tipPosition()[2] << std::endl;
    std::cout << "Outer tube tip position: " << ot_frames.back().origin()[0] << ", " << ot_frames.back().origin()[1] << ", " << ot_frames.back().origin()[2] << std::endl;
}