#include "Copter.h"
#include <math.h>

#if MODE_STM_ENABLED == ENABLED




uint32_t past 	= AP_HAL::millis();
uint32_t res 	= AP_HAL::millis();
uint32_t dt 	= 50  ;
uint32_t t      = 0   ;
uint32_t pt     = 0;



int yc		= 3   ;
int per		= 6   ;
float hz,hx,posx,posz ;
double a,b	      ;
float aci,baci	      ;
double our_angle,dg_angle  ;

Vector3f hiz	      ;
Vector3f des_pos      ;
Vector3f bes_pos      ;
Vector3f two_meters   ;	     
Vector3f sifir        ;	     
Vector3f positron     ;

double te,pte	      ;
int32_t height_above_ground_cm ;
bool relative_angle= true;
float yaw = 359       ;
int sayac = 1         ;
Location loc          ;





bool ModeStm::init(bool ignore_checks){
sayac=1;
res 	= AP_HAL::millis();
return true;
}





void ModeStm::run()
{

  gcs().send_text(MAV_SEVERITY_WARNING, "x:%falt:%dsayac:%dposx:%fposz:%f",inertial_nav.get_position().x,copter.flightmode->get_alt_above_ground_cm(),sayac,posx*100,posz*100);
  // hal.console->printf("aci:%f",atan(a/b));
  switch(sayac)
{	
     case 1:
		two_meter();
		if(two_meter_check()){

			sayac=2;
		past=AP_HAL::millis();}
		break;
		
     case 2:
		five_meters_with_yaw();
		if(AP_HAL::millis()-past>=10*1000){
			sayac=3;
		past=AP_HAL::millis();}
		break;
     case 3:
		cross_with_yaw();
		if(AP_HAL::millis()-past>=15*1000)
			sayac=4;
		break;
     case 4:
		two_meter();
		if(two_meter_check()){
			sayac=5;
		past=AP_HAL::millis();}
		break;

	
     case 5:
		//circle_with_velocity();
		//circle_with_position();
		circle_with_position_time();
		if(AP_HAL::millis()-past>=14*1000){
			if(two_meter_check_ns()){
			sayac=6;
		}}
	break;
     case 6:
		hover();
		break;
}

}



void ModeStm::circle_with_velocity()
{
    te=double(t)/1000;
	hx 	= sqrt((2*M_PI*yc)/per)*cos((2*M_PI*te)/per);
	hz 	= (sqrt((2*M_PI*yc)/per))*sin((2*M_PI*te)/per);
	//gcs().send_text(MAV_SEVERITY_WARNING, "x teki hiz %f z deki hiz %f zaman:%f ",hx,hz,te);
//	hal.console->printf("x teki hiz %f z deki hiz %f t=%u zaman:%f millis=%d aci=%f \n",hx,hz,t,te,AP_HAL::millis(),pos_control->get_pitch());
	t	+= 2.5 ;
	past	= AP_HAL::millis() ;
	hiz	= Vector3f(hx * 100.0f, 0 , hz * 100.0f);
	//motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

	//copter.mode_guided.set_velocity(hiz, false, 0, true, 60*100, false);
    	pos_control->set_desired_velocity(hiz);
	//set_desired_velocity_with_accel_limits(hiz);
	auto_yaw.set_rate(60*100);
	//pos_control->set_desired_velocity_z(hiz.z);
	pos_control->update_vel_controller_xyz();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), auto_yaw.rate_cds());
	pos_control->update_z_controller();
}



//iptal 
void ModeStm::circle_with_position()
{
	a= inertial_nav.get_position().x;
	b= 5-inertial_nav.get_position().z;
	
	if(a>0){
		our_angle=angle(a,b)+0.02;
		}
	else if(a<0){
		our_angle=angle(a,b)+M_PI+0.02;}

	dg_angle=(our_angle*180)/M_PI;
	if(dg_angle>=34 and dg_angle<=46)
	our_angle+=0.2;
	gcs().send_text(MAV_SEVERITY_WARNING, "ANGLE:%f",dg_angle);
	posx = yc*sin(our_angle);
	posz = 5 - yc*cos(our_angle);
	//posx = yc*sin(atan2(b,a)-0.174);
	//posz = 5 - yc*cos(atan2(a,b)-0.174);
	positron=Vector3f(posx*100.0f,0,posz*100.0f);
	pos_control->set_pos_target(positron);
	pos_control->update_vel_controller_xyz();
	auto_yaw.set_rate(60*100);
	attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
	pos_control->update_z_controller();
}
void ModeStm::circle_with_position_time(){
	
	if (AP_HAL::millis()-res>dt){
	res 	= AP_HAL::millis();
	pt+=dt;
	pte=double(pt)/1000;
	posx = yc*sin((2*M_PI*pte)/20);
	posz = 5 - yc*cos((2*M_PI*pte)/20);
		}

	positron=Vector3f(posx*100.0f,0,posz*100.0f);
	pos_control->set_pos_target(positron);
	pos_control->update_vel_controller_xyz();
	auto_yaw.set_rate(60*100);
	attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
	pos_control->update_z_controller();

}

//iptal

void ModeStm::two_meter(){
	two_meters = Vector3f(0,0,2*100);
	auto_yaw.set_rate(0*100);
	copter.pos_control->set_pos_target(two_meters);
	pos_control->update_vel_controller_xyz();
	attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());	
	pos_control->update_z_controller();
}

float ModeStm::angle(float ap,float bp){
	float dot= -5*bp;
	float magn_vec=sqrt(sq(ap)+sq(bp));
	float magn= 5*magn_vec;
	return acos(dot/magn);
	
//vectors a,b   0,-5
			}

bool ModeStm::two_meter_check(){

	height_above_ground_cm = copter.flightmode->get_alt_above_ground_cm();

	if(height_above_ground_cm >190 && height_above_ground_cm<210){
		return true;
	}
	else
		return false;
}



bool ModeStm::two_meter_check_ns(){

	height_above_ground_cm = copter.flightmode->get_alt_above_ground_cm();
	//inertial_nav.get_altitude();
	
	if(height_above_ground_cm<215){
		return true;
	}
	else
		return false;
}



void ModeStm::cross_with_yaw(){
	des_pos = Vector3f(0*100.0f,0*100.0f,7*100.0f);
       
        pos_control->set_pos_target(des_pos);
	auto_yaw.set_rate(60*100);
		pos_control->update_vel_controller_xyz();
	//auto_yaw.set_fixed_yaw(yaw, 0.0f, 0, relative_angle);
      
        //attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
	attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
        pos_control->update_z_controller();

}



void ModeStm::five_meters_with_yaw(){
	bes_pos= Vector3f(5*100.0f,0,2*100.0f);

	pos_control->set_pos_target(bes_pos);
	auto_yaw.set_rate(60*100);
	//auto_yaw.set_fixed_yaw(yaw, 0.0f, 0, relative_angle);
       	pos_control->update_vel_controller_xyz();
        //attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
	attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
        pos_control->update_z_controller();
}



void ModeStm::hover(){

	sifir = Vector3f(0.0f,0.0f,0.0f);
	
	auto_yaw.set_rate(0);

	set_desired_velocity_with_accel_limits(sifir);

	pos_control->update_vel_controller_xyz();

	attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), auto_yaw.rate_cds());

	pos_control->update_z_controller();
}



void ModeStm::set_desired_velocity_with_accel_limits(const Vector3f& vel_des)
{

    Vector3f curr_vel_des = pos_control->get_desired_velocity();

    Vector3f vel_delta = vel_des - curr_vel_des;


    float vel_delta_xy = safe_sqrt(sq(vel_delta.x)+sq(vel_delta.y));
    float vel_delta_xy_max = G_Dt * pos_control->get_max_accel_xy();
    float ratio_xy = 1.0f;
    if (!is_zero(vel_delta_xy) && (vel_delta_xy > vel_delta_xy_max)) {
        ratio_xy = vel_delta_xy_max / vel_delta_xy;
    }
    curr_vel_des.x += (vel_delta.x * ratio_xy);
    curr_vel_des.y += (vel_delta.y * ratio_xy);

    float vel_delta_z_max = G_Dt * pos_control->get_max_accel_z();
    curr_vel_des.z += constrain_float(vel_delta.z, -vel_delta_z_max, vel_delta_z_max);


    pos_control->set_desired_velocity(curr_vel_des);
}
#endif
