#ifndef VEHICLE_PARAMETERS_HPP
#define VEHICLE_PARAMETERS_HPP

// Structure to store the vehicle parameters

struct VehicleParameters{
    // Vehicle parameters
    struct Inertia{
        double mass; // Mass of the vehicle (kg)
        double Izz; // Moment of inertia about the z-axis (kg*m^2)
    } inertia;
    // Vehicle kinematics
    struct Kinematics{
        double l_f; // Distance from the center of mass to the front axle (m)
        double l_r; // Distance from the center of mass to the rear axle (m)
        double drag_coefficient; // Drag coefficient of the vehicle 
        double rolling_resistance_coefficient; // Rolling resistance coefficient of the vehicle 
    } kinematics;
    // Tire parameters
    struct Tire{
        double c_f; // Front tire cornering stiffness (N/rad)
        double c_r; // Cornering stiffness of the tire (N/rad)
    } tire;
};

#endif