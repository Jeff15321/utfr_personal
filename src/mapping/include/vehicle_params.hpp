#ifndef VEHICLE_PARAMETERS_HPP
#define VEHICLE_PARAMETERS_HPP

struct VehicleParameters{
    struct Inertia{
        double mass; // Mass of the vehicle (kg)
        double Izz; //
    } inertia;
    struct Kinematics{
        double l_f;
        double l_r;
        double drag_coefficient;
        double rolling_resistance_coefficient;
        double cornering_stiffness;
        double down_force_coefficient;
    } kinematics;
    struct Tire{
        double c_r;
        double c_f;
    } tire;
};

#endif