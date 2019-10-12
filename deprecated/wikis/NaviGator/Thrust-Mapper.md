This program receives a wrench from the wrench arbiter and solves for the optimal thrust vector configurations. Thruster center of gravity offsets are located in the the launch file.

Thrusters are added to the solver by creating new Thruster objects and adding them to the Mapper class on instantiation.

This mapper is a simple Ax = b linear least squares solver. The A matrix is built on every iteration and remapping, allowing us to eventually detect thruster failure and adapt the mapper on the fly. For right now though we are solving for 4 thrusters offset at +-45 degree angles to the vehicle.

Essentially all that is happening is a net force is being distributed to all four thrusters. There are many ways to do this so we find the solution with the smallest "sum of the deviations from the slope line squared - least squares". 

**The Numpy Linear Algebra library is used to solve the least squares problem**

In the code:

    A = Matrix of thruster locations and angles to the boat in relation to the x axis contained in a 3x4 matrix
    b = Desired net x and y forces and desired rotation around the z axis contained in a 3x1 vector
    x = The optimal thrust force for each thruster that we solve for contained in a 4x1 vector

**Thruster configurations are in the gnc.launch file. They contain the (x,y,theta) offset for each thruster.**

The naming conventions below are used throughout the entire system

    BL = Back Left
    BR = Back Right
    FL = Front Left
    FR = Front Right

### Bibliography

    [1] Christiaan De Wit
        "Optimal Thrust Allocation Methods for Dynamic Positioning of Ships"
        see: http://repository.tudelft.nl/assets/uuid:4c9685ac-3f76-41c0-bae5-a2a96f4d757e/DP_Report_FINAL.pdf