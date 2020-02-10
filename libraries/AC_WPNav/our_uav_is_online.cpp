#include <AP_HAL/AP_HAL.h>
#include "our_uav_is_online.h"


const AP_Param::GroupInfo Stm::var_info[] = {
    // index 0 was used for the old orientation matrix

    // @Param: YARICAP
    // @DisplayName: Drone'un dikey cember hareketini yapacagi captir
    // @Description: Metre cinsindendir.
    // @Units: m
    // @Range: 1 10
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STM_YARICAP",       0, Stm, _yaricap_m, YARICAP),

    // @Param: PERIYOT
    // @DisplayName: HAREKET PERIYODU
    // @Description: Saniye cinsindendir.
    // @Units: s
    // @Range: 0 30
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STM_PERIYOT",      1, Stm, _periyot, PERIYOT),

    AP_GROUPEND
};
uint32_t pastr 	= AP_HAL::millis();
uint32_t resr 	= AP_HAL::millis();
uint32_t dtr 	= 25 ;
uint32_t tr      = 0;
void Stm::cember(float *ax,float *az){
if ( AP_HAL::millis() - pastr >= dtr ){
		hx 	= sqrt((2*M_PI*YARICAP)/PERIYOT)*cos((2*M_PI*tr)/PERIYOT);
		hz 	= sqrt((2*M_PI*YARICAP)/PERIYOT)*cos((2*M_PI*tr)/PERIYOT);
		pastr 	= AP_HAL::millis();
		tr 	= tr+dtr ;
		az	= &hz;
		ax	= &hx;
		}

if( AP_HAL::millis() - resr >= 4000 ){
		resr 	= AP_HAL::millis();
		tr 	= 0;
}
}
