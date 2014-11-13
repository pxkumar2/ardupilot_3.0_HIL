// ==========================================================================================================
/* Function File for abc controller for use in simulink */

//#include "abc_controller.h"
#include <limits.h>
#include "abc_controller_type.h"
/*************************************************************************
 * FUNCTION ABC_Controller
 * /**************************************************************************
 * Hawker Beechcraft Company               D. Kimball   March 11, 2010
 *
 * This function contains the WSU ABC controller with ANN feature, exerting
 * control of elevator and throttle servos when engaged.
 * Called regardless of whether ABC_Engage is true or false.
 * Neural networks are not activated until Use_ANN = true.
 **************************************************************************/

/*void ABC_Controller(double input[17], double output[5], double bona_in[8], double save_data[13]) // For WSU
// void ABC_Controller(void) // For BCC
{*/
    // Variables added that are used as global variables in HBC C code  JTO 10/28/10
   	float Vdot_DES;
    float Vdot_ADD;
    float Qdot_DES;
    float Qdot_ADD;
    float ABC_elev_pos;
    float ABC_pla_pos;   
					    
    //const float aterm		= -0.0000534;// constant term in shp vs. pla quadratic, updated 11/19/2008 DFK
   // const float bterm		= 0.01387;   // constant term in shp vs. pla quadratic, updated 11/19/2008 DFK
   //						= 0.9 * 0.01387;   // reduce by 10% for sensitivity study 3/16/2010 DFK
   // float cterm			= 0.09582;   // constant term in shp vs. pla quadratic, updated 11/19/2008 DFK
    float rho_term			= -0.0000142;

	float Vcommand;
	float GammaCmd;
    float Vtrue;
	float Vtrue_dot;
	float Qbar;
    float Pitch;
    float PitchRate;
    float Qdot;
    float Qhat;
    float Gamma;
    float Alpha_ABC;
    float AlphaDot_ABC;
    float AlphaDot_hat_ABC;
    float V_error;
    float Vdot_command;
//  float Gamma_offset;
    float tau_gamma;
    float GammaCmd_mdl;
    float gamma_error;
    float gamma_error_rate;
    float pid_Kd;
    float pid_Kp;
    float pid_Ki;
	float Qdot_error;
//	float elevator;
   float ABC_flap_pos;

    static float Vtrue_Prev;   
    static float PitchRate_Prev;
    static float Qdot_Prev;
    static float Alpha_Prev_ABC;
    static float Vdot_ADD_Prev;
    static float GammaCmd_mdl_Prev;
    static float gamma_error_Prev;
    static float pid_Ki_Prev;
	static float Qdot_ADD_Prev;

	static float elev_offset;
    static float PLA_offset;
	static float left_elevator_jam;

    float Qdot_command;
    float rho_est;    
    float thrust_est;
    float Tcp_est;
	float CL_est;
    float CD_est;
    float Cmde0;
    float drag_est;
    float thrust_demand;
    float CM_est;
    float Elevator_new;
    float shp_demand;
    float Throttle_new;
//	float Qdot_learning_gain;
    float Qdot_ADD_delta;

// Varables for Predictive Inverse Controller
    float CL_est_Pred;
    float CD_est_Pred;
    float Cmde0_Pred;
    float drag_est_Pred;
    float thrust_demand_Pred;
    float CM_est_Pred;
    float Elevator_new_Pred;
    float CLALPHAzero;
    const float CLADOTHAT		= 1.7050;
    const float CLQHAT			= 5.7357;
    const float CLDE			= 0.3867;
    float CLthrust;
    float clRHS;
    float Q_DES;
    float QDES_prev;
    float Alpha_ABC_Pred;


	/*Const Since not using Dynamic Inversion*/
	/*for Cessna_Roskam*/
	/*const float CL0_est		= 0.307;
    const float CLa_est		= 4.41;
    const float CD0_est		= 0.027;
    const float CDk_est		= 1/(PI*7.443*0.8);
    const float CLi_est		= 0.0;
    const float CM0_est		= 0.04;
    const float CMa_est		= -0.613;
    const float CMde_est	= -1.122;
    const float CMq_est		= -12.4;
    const float CMalphadot_est = -7.27;
	const float dT = 0.0;      //0.894 for bonanza*/

    const float CL0_est		= 0.32;
    const float CLa_est		= 4.581;
    const float CD0_est		= 0.027;
    const float CDk_est		= 1.0/(PI*9.36*0.9);
    const float CLi_est		= 0.0;
    const float CM0_est		= 0.0327;
    const float CMa_est		= -1.7237;
    const float CMde_est	= -1.0;               //-0.0234;
    const float CMq_est		= -11.18;
    const float CMalphadot_est = -5.57;

	const float dT = 0.0;  
    float shp_est;
	const float shpmax_est = 0.04;    
    const float shp_limit    = shpmax_est;
	const float eta_prop_est = 1.0;
	const float minelev		 = -15.0;	// set to reduced values for safety
    const float maxelev		 = 15.0;    // set to reduced values for safety

	WSU_STATE state = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    //WSU_AIRPLANE airplane = {0.02,4.9,174,-20,10};   for cessna 172
	WSU_AIRPLANE airplane = {0.02,.481,2.165,minelev, maxelev};
	/*airplane.delta_t	= ;					
    airplane.wing_mac	= ;					
    airplane.wing_area	= ;                       
    airplane.elevator_neg_limit = ;
    airplane.elevator_pos_limit = ;*/

	const float dtor		= M_PI/180.0;	 // degrees to radians
    const float rtod		= 180.0/M_PI;	 // radians to degrees
    const float cas_to_tas  = 1.0;		 // change airspeed
    const float kt_to_ft	= 1.6878;	 // nuatical mile to feet
    const float PhiT        = 0.0* dtor; //was -2.0
	const float T1_u        = 1.5;        //=5; // ref WSU MATLAB file bona.asv of 2/22/2010  
	const float minthrottle	= 5.0;		 // leave tolerance between software stop and hardware stop
    const float maxthrottle	= 96.0;		 // leave tolerance between software stop and hardware stop

    const float Kp_gamma    = 1.3613 ;//;        
    const float Ki_gamma	= 0.0;		// from WSU MATLAB file bona.asv of 2/22/2010
    const float Kd_gamma	= 2.3335;  // from WSU MATLAB file bona.asv of 2/22/2010

    const float Iyy_body	= 0.02274;	// moment of inertia about y-axis,  slug-ft^3

    int ABC_CommandedMode = 2;					// mode
    bool Pred_Engage;
	bool Use_ANN ;   
	bool ABC_Engage;   
    bool  DynInversionMode;						// 0 = fixed aero coefficient; 1 = real-time updated  4/1/2010
	


void abc_controller(float input[16],float output[5])
	{
 //*****************BEGIN ABC_CONTROLLER*******************************************  
 // ===============================================================================

    // Assign values to variables not included in HBC code          JTO 10/28/10
    state.kcas_command		= input[0];
    state.gamma_command		= input[1];
    state.kcas				= input[2];
    state.gamma_air			= input[3];
    state.pitch				= input[4];
    state.pitch_rate		= input[5];
    state.altitude	     	= input[6];
    state.elevator  		= input[7];
    state.pla    			= input[8];
    state.gear_pos			= input[10];
	Pred_Engage             = input[12];
	Use_ANN                 = input[13];
    ABC_Engage				= input[14];
    ABC_CommandedMode		= input[15]; //Added PJM 07/26/2012
    DynInversionMode		= input[16];    
    state.flap_pos_constant = 0.0;
    state.rpm				= 2500.0; 
    state.weight			= 2.09; 
    state.mass				= 2.09/32.3;
	airplane.delta_t        = G_Dt;        //Use real time for derivative 
    /*if((!ABC_Engage)||(!ElevatorEngage)||(!ThrottleEngage)||(GS411.GS_CommandedMode >= 2))
        !ABC_Engage = TRUE;
    else
        !ABC_Engage = FALSE;*/
	// ==========================================================================================================   

//  Process inputs into ABC controller
    GammaCmd       = dtor * state.gamma_command;					 // Gamma_cmd (radians)
    Vcommand       = kt_to_ft * cas_to_tas * state.kcas_command;     // Speed commmand in true airspeed (fps);
    Vtrue          = kt_to_ft * cas_to_tas * state.kcas;		     // true airspeed (unfiltered) (fps)
    Qbar           = 0.003386 * state.kcas * state.kcas;		     // dynamic pressure (psf)
	Pitch          = dtor * state.pitch;							 // pitch attitude (radians)
    PitchRate      = dtor * state.pitch_rate;						 // pitch rate (rad/sec)
	Gamma		   = dtor * state.gamma_air;					     // flight path angle relative to air
	Qhat		   = (PitchRate * airplane.wing_mac)/(2.0 * Vtrue);    // non-dimensional pitch rate (rad)
    Alpha_ABC      = Pitch - Gamma;								     // derived alpha (valid when wings level) (rad)
	
	Vtrue_dot      = Derivative(Vtrue_Prev, Vtrue, airplane.delta_t);	
	Vtrue_Prev     = Vtrue;
    
    Qdot		   = Derivative(PitchRate_Prev, PitchRate, airplane.delta_t); // pitch accel (rad/sec^2)
    PitchRate_Prev = PitchRate;			        				              // previous pass saved for Derivative

    AlphaDot_ABC   = Derivative(Alpha_Prev_ABC, Alpha_ABC, airplane.delta_t); // (rad/sec)
    Alpha_Prev_ABC = Alpha_ABC;
    AlphaDot_hat_ABC =(AlphaDot_ABC * airplane.wing_mac /(2.0 * Vtrue));        // non-dimensional alphadot (rad)
    
    
    rho_est = 0.0023769 * (1.0 + (2.0 + rho_term * state.altitude) * (rho_term * state.altitude)); // (slug-ft^3)
    //shpmax_est = 285. * (rho_est / 0.0023769)*(-0.04625 + 0.0003875 * state.rpm);                  // est SHP
    //eta_prop_est = 0.55 + 0.0012 * Vtrue;                                                          // approximation from desktop simulation
    //eta_prop_est = min(eta_prop_est, 0.87);// for bonanza
    
    if(DynInversionMode)   // advanced controller...update aero terms real-time
    {
       /* shp_est = shpmax_est * (cterm + state.pla * (bterm + aterm * state.pla));                  // engine shaft horsepower
        thrust_est = eta_prop_est * 550. * shp_est / Vtrue;	                                      // pounds
        Tcp_est = thrust_est / (Qbar * airplane.wing_area);
        CL0_est = 0.320 + 0.0911 * Tcp_est + 0.0163 * state.gear_pos + 0.01356 * ABC_flap_pos;
        CLa_est = 4.8652 + 2.6610 * Tcp_est - 0.2275 * state.gear_pos + 0.00845 * ABC_flap_pos;
        CD0_est = 0.0204 + 0.098 * Tcp_est + 0.0208 * state.gear_pos + 0.00156 * ABC_flap_pos;
        CDk_est = 0.078 - 0.075 * Tcp_est;
        CLi_est = 0.070;
        CM0_est = 0.01174 + 0.1259 * Tcp_est - 0.0008 * state.gear_pos - 0.00179 * ABC_flap_pos;
        CMa_est = -0.351 - 0.4211 * Tcp_est + 0.0222 * state.gear_pos - 0.00267 * ABC_flap_pos;
        CMq_est = -13.24 - 8.818 * Tcp_est;
        CMalphadot_est = -3.968 - 14.83 * Tcp_est;
        CMde_est = -1.187 - 2.086 * Tcp_est + 0.0005 * state.gear_pos - 0.00125 * ABC_flap_pos;
    }
    else   // basic controller: fixed aero coefficients per WSU s-function controller of 2/22/2010
    {
        CD0_est = //0.0249;
        CLi_est = 0.0729;
        CDk_est = 0.0780;
        CLa_est = 4.9866;
        CL0_est = 0.3242;
        CM0_est = 0.0175;
        CMa_est = -0.3702;
        CMde_est = -1.2822;
        CMq_est = -13.6424;
        CMalphadot_est = -4.6447;*/
    }
    
//  ABC controller: velocity loop
    if(!ABC_Engage)
    {
        Vdot_DES = 0.0;
    }
    else
    {
        Vdot_DES = (Vcommand - Vtrue) / T1_u;   // Ref MATLAB file bon (ft/sec^2)
    }
    if((!ABC_Engage) || (!Use_ANN))
    {
        Vdot_ADD = 0.0;		                    // null as ANN is not activated
    }
    else
    {
        V_error = Vdot_DES - Vtrue_dot;    // (fps)        
//		if(((state.pla[0] <= maxthrottle) && (state.pla[0] > minthrottle))
//			|| ((state.pla[0] < minthrottle) && (V_error > 0))
//			|| ((state.pla[0] > maxthrottle) && (V_error < 0)))
//		if(((state.pla[0] < maxthrottle) && (state.pla[0] > minthrottle))
//		3/30/2010 DFK to avoid small residual allowing integrator windup:

		/**************************************************************************************************************************
        /*if(((state.pla < (maxthrottle - 0.25)) && (state.pla > (minthrottle + 0.25)))
        || ((state.pla <= minthrottle) && (V_error > 0.0))
        || ((state.pla >= maxthrottle) && (V_error < 0.0)))*/
	    if(((state.pla < (maxthrottle + PLA_offset - 2))
        && (state.pla  > (minthrottle + PLA_offset + 2)))
        || ((state.pla <= (minthrottle)) && (V_error > 0))
        || ((state.pla >= (maxthrottle)) && (V_error < 0)))

        {
            Vdot_ADD = Vdot_ADD_Prev + 0.01 * V_error;  // (ft/sec^2)
        }
        else
        {
            Vdot_ADD = Vdot_ADD_Prev;		// (ft/sec^2)
        }
    }
    
    Vdot_command = Vdot_DES + Vdot_ADD;		// (ft/sec^2)
    Vdot_ADD_Prev = Vdot_ADD;
    
//  ABC controller: gamma loop
    if(!ABC_Engage)   // initialize model follower and PID filter dynamics for engagement
    {
        GammaCmd_mdl_Prev = GammaCmd;
        gamma_error_Prev = GammaCmd - Gamma; //0.-Gamma; //Added PJM 07/26/2012
        pid_Ki_Prev = 0.0;
        Qdot_DES = 0.0;    // ThetaDDcmd in bon.mdl;
        left_elevator_jam = state.elevator;	// save this position for reference
    }
    else
    {
      //  Implement first-order Model Follower filter on command:
      //  tau_gamma = Kd_gamma / Kp_gamma;	                      // equivalent tau for gamma model follower filter
        tau_gamma = 0.5 * Kd_gamma / Kp_gamma;	                  // equivalent tau for gamma model follower filter
        GammaCmd_mdl = lag_filter(GammaCmd, tau_gamma, GammaCmd_mdl_Prev);
        GammaCmd_mdl_Prev = GammaCmd_mdl;        
        gamma_error = GammaCmd_mdl - Gamma;		                  // (radians)
        
      // Implement PID controller
      // Derivative path:
        gamma_error_rate = (gamma_error - gamma_error_Prev) / airplane.delta_t; // (rad/sec)
		pid_Kd = Kd_gamma * gamma_error_rate;
        gamma_error_Prev = gamma_error;
      // Proportional path:
        pid_Kp = Kp_gamma * gamma_error;
      // Integral path:
        pid_Ki = pid_Ki_Prev + Ki_gamma * gamma_error * airplane.delta_t;
        pid_Ki_Prev = pid_Ki;
        
        Qdot_DES = pid_Kd + pid_Kp + pid_Ki;                      // ThetaDDcmd in bon.mdl (rad/sec^2)
    }
    
    if((!ABC_Engage) || (!Use_ANN))
    {
        Qdot_ADD = 0.0;
    }
    else
    {
        Qdot_error = Qdot_DES - Qdot_Prev;
        
       //3/30/2010 DFK to avoid small residual allowing integrator windup:
        if(((state.elevator <= (maxelev - 0.25)) && (state.elevator >= (minelev + 0.25)))
        || ((state.elevator > maxelev) && (Qdot_error > 0.0))
        || ((state.elevator < minelev) && (Qdot_error < 0.0)))
        {
  /*          
//*		Allow Qdot_learning_gain to be adjusted in flight using StandoffAlt dial-in  5/4/2010 DFK
            if(GS_StandoffAlt <= 0.5)
                Qdot_learning_gain = 0.05;
            else if (GS_StandoffAlt <= 1.5)
                Qdot_learning_gain = 0.10;
            else if (GS_StandoffAlt <= 2.5)
                Qdot_learning_gain = 0.15;		// baseline
            else if (GS_StandoffAlt <= 3.5)
                Qdot_learning_gain = 0.20;
            else if (GS_StandoffAlt <= 4.5)
                Qdot_learning_gain = 0.25;
            else if (GS_StandoffAlt <= 5.5)
                Qdot_learning_gain = 0.30;
            else
                Qdot_learning_gain = 0.15;		// baseline*/
            
            Qdot_ADD = (Qdot_ADD_Prev + 0.35* Qdot_error);
        //	Qdot_ADD = (Qdot_ADD_Prev + Qdot_learning_gain * Qdot_error);  // 5/4/2010 DFK
            
            if(ABC_CommandedMode == 3)
            {
              /*Apply e modification (error weighted weight damping) from Dr. Steck 5/6/2010  							
               Qdot_ADD = Qdot_ADD - 0.05 * abs(Qdot_error) * Qdot_ADD;    // 5/7/2010 DFK
               Qdot_ADD = Qdot_ADD - 0.05 * abs(Qdot_error) * Qdot_ADD;    // 5/7/2010 DFK
               Reprogram to preclude a numerical error with above (in Eclipse compilation)*/
                if(Qdot_error >= 0.)
                    Qdot_ADD_delta = - 0.05 * Qdot_error * Qdot_ADD;    // 5/10/2010 DFK
                else
                    Qdot_ADD_delta = 0.05 * Qdot_error * Qdot_ADD;		// 5/10/2010 DFK
                
            }
            else
            {
                Qdot_ADD_delta = 0.0;
            }
            Qdot_ADD += Qdot_ADD_delta;					                  // 5/10/2010 DFK
        }
    }
    
    Qdot_command = Qdot_DES + Qdot_ADD;     
    Qdot_ADD_Prev = Qdot_ADD;
    Qdot_Prev = Qdot;                                                      // save last pass for ANN
    // compute current offsets using predictive controller   8/31/2012 Venkat
    if((ABC_CommandedMode == 6)||(ABC_CommandedMode == 7)||(ABC_CommandedMode == 8))  
        
// Modifications for Predictive Inverse controller  2/9/2012 - Jim Steck
    {
        Q_DES = QDES_prev + Qdot_DES*airplane.delta_t;
        QDES_prev = Q_DES;
        CL_est_Pred = CL0_est + CLa_est * Alpha_ABC;													// Checked ???? - BSS
        CD_est_Pred = CD0_est + CDk_est * (CL_est_Pred - CLi_est) * (CL_est_Pred - CLi_est);			// Checked ????
        Cmde0_Pred = CM0_est + CMa_est * Alpha_ABC + CMalphadot_est * AlphaDot_hat_ABC + CMq_est * Qhat;// ???? - BSS
        drag_est_Pred = CD_est_Pred * Qbar * airplane.wing_area;										// Checked 3/4/2011 - BSS
        thrust_demand_Pred = (drag_est_Pred + state.weight * sinf(Gamma) + state.mass * Vdot_command) / cosf(Alpha_ABC+ PhiT); // Checked ???? - BSS
        CM_est_Pred = (Qdot_command * Iyy_body + dT * thrust_demand_Pred)/(Qbar * airplane.wing_area * airplane.wing_mac)- Cmde0_Pred; // Checked ???? - BSS
        Elevator_new_Pred = (CM_est_Pred / CMde_est) ;		  									// (radians) // Checked ???? - BSS        
        CLALPHAzero=CL0_est+CLADOTHAT*AlphaDot_hat_ABC+CLQHAT*Qhat+CLDE*Elevator_new_Pred;
        CLthrust=thrust_demand_Pred/(Qbar * airplane.wing_area)*sinf(Alpha_ABC+ PhiT);
        clRHS=state.mass*Vtrue/(Qbar * airplane.wing_area)*Q_DES+state.weight/(Qbar * airplane.wing_area)*cosf(Gamma)-CLALPHAzero-CLthrust;
        Alpha_ABC_Pred =clRHS/CLa_est;
        Alpha_ABC = Alpha_ABC_Pred; //  use predicted value of alpha Alpha_ABC_Pred below instead of current value Alpha_ABC
    }
//   End modifications for Predictive Inverse controller 2/9/2012 - Jim Steck

//  Inverse Controller.  Equations per WSU file inverse_linear.m  2/22/2010:  	
	CL_est = CL0_est + CLa_est * Alpha_ABC;
	CD_est = CD0_est + CDk_est * (CL_est - CLi_est) * (CL_est - CLi_est);
	Cmde0 = CM0_est + CMa_est * Alpha_ABC + CMalphadot_est * AlphaDot_hat_ABC + CMq_est * Qhat;
	drag_est = CD_est * Qbar * airplane.wing_area;

	// Convert thrust demand into PLA for saturation limits to apply
	thrust_demand = (drag_est + state.weight * sinf(Gamma) + state.mass * Vdot_command) / cosf(Alpha_ABC + PhiT);    //added PhiT term
	shp_demand = thrust_demand * Vtrue / (eta_prop_est * 550.0);
	//shp_limit = (cterm - bterm*bterm /(4*aterm))* shpmax_est - 1.0;       // avoid trap with sqrt call below 3/17/2010 DFK
	shp_demand = min(shp_demand, shp_limit);
	Throttle_new = shp_demand/shp_limit *100.0; //(-bterm + sqrt(bterm * bterm - 4. * aterm * (cterm - shp_demand / shpmax_est))) / (2.*aterm);
	ABC_pla_pos = max(minthrottle, min(maxthrottle, Throttle_new + PLA_offset));

	// Convert Saturated throttle back into thrust value for trim solution

	// Added 
	/**********************************************
	Throttle_new = ABC_pla_pos;*/
	Throttle_new = ABC_pla_pos - PLA_offset;
	shp_demand = Throttle_new*shp_limit;
		//(((Throttle_new * (2.*aterm) + bterm)*(Throttle_new * (2.*aterm) + bterm))*(shpmax_est) - bterm*bterm * shpmax_est + 4*aterm*cterm*shpmax_est)/(4*aterm);
	thrust_demand = shp_demand * (eta_prop_est * 550.0) / Vtrue;

	// Calculate Elevator Deflection
	CM_est = (Qdot_command * Iyy_body + dT* thrust_demand)/(Qbar * airplane.wing_area * airplane.wing_mac)- Cmde0;
	Elevator_new = (CM_est / CMde_est) / dtor;  // (degrees)
    // compute current offsets to ensure transient-free engagement
    if(!ABC_Engage)	 
    {
        elev_offset = state.elevator - Elevator_new;   // (deg)
        PLA_offset = state.pla - Throttle_new;		   // (%)
    }
    
    ABC_elev_pos = max(airplane.elevator_neg_limit,
            min(airplane.elevator_pos_limit, Elevator_new + elev_offset));  // (deg)

    // simulate a jam of "left" elevator surface  3/19/2010 DFK
    if((ABC_CommandedMode == 4)||(ABC_CommandedMode == 7))  
    {
       /*Adjust ABC_elev_pos to reflect only right elevator responding to control
         while left surface remains jammed at fixed position...*/
        ABC_elev_pos = 0.5 * (left_elevator_jam + ABC_elev_pos);
    }
    else
    {
        left_elevator_jam = state.elevator;	// save this position for reference
    }
    
    ABC_pla_pos = max(minthrottle, min(maxthrottle, Throttle_new + PLA_offset));  //(%)
    // simulate a linear reduction in thrust output  3/19/2010 DFK
    if((ABC_CommandedMode == 5)||(ABC_CommandedMode == 8))  
    {
        ABC_pla_pos = 0.67 * ABC_pla_pos;     //Adjust ABC_pla_pos to reduce engine output:
    }
    
   
//  Need to add proper output, not included in HBC code     JTO 10/28/10
    output[0] = ABC_pla_pos;
    output[1] = ABC_elev_pos;
    output[2] = Qdot_ADD;
    output[3] = Vdot_ADD;
    output[4] = CL0_est;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//END ABC_CONTROLLER
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Routine for calculating derivative of the input */

float Derivative(float Prev_value, float Current_value, float Delta_t)
{
    //float value_hat;
    /* u_hat = (u(t) - u(t-1))/delta_t */
    //value_hat = (Current_value - Prev_value)/ Delta_t ;
    /* Return derivative */
    //return value_hat;
	return ((Current_value - Prev_value)/ Delta_t);
}

/* Routine for calculating filtered values */
float lag_filter(float X, float tau, float Y_last)
{
    float ka, kb, Y;
    // Because global variables cannot be used, the time step must be included  JTO 10/28/10
    ka = 0.02 / (tau + 0.02);
    kb = tau / (tau + 0.02);
    Y = ka * X + kb * Y_last;
    
    /* ka = airplane.delta_t / (tau + airplane.delta_t);
     * kb = tau / (tau + airplane.delta_t);
     * Y = ka * X + kb * Y_last; */
    
    return Y;
}
