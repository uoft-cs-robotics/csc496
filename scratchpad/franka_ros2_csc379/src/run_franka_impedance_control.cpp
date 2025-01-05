
#include "franka_ros2_csc379/franka_impedance_control.hpp"


int main(int argc, char* argv[])
{
    FrankaImpedanceControl fic = FrankaImpedanceControl();
    
    // Task: After check impedance works properly by pushing robot.
    // Create a joint trajectory here and move robot using SetCommandJointPositions()

    fic.join(); // waits for .control thread to finish so program doesn't exit. 
    return 0;
}
