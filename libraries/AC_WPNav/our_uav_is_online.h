#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude control library
#include <AP_Terrain/AP_Terrain.h>
#include <AC_Avoidance/AC_Avoid.h>                 // Stop at fence library

#define YARICAP            3
#define PERIYOT            6


class Stm
{
public:
static const struct AP_Param::GroupInfo var_info[];	


void cember(float *ax,float *az);
protected:
AP_Int16 _yaricap_m;
AP_Int16 _periyot;
private:
float hx;
float hz;
};



