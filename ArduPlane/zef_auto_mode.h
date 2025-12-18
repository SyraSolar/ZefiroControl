#include <math.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include "Plane.h"

class ZefAutoMode {
private:

public:
    AP_SerialManager serial_manager;
    Location desired_position;
    double desired_alt;
    bool must_reset_loc;
    double return_errors[3];
    AP_InertialNav zef_inertial_nav = AP_InertialNav(AP::ahrs());
    
    //initialization
    /*ZefAutoMode() {
        //
    }*/
};

extern ZefAutoMode zefiroAutoMode;