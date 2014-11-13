/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Uno, Platform=avr, Package=arduino
*/

#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
#define __AVR_ATmega328p__
#define __AVR_ATmega328P__
#define ARDUINO 105
#define ARDUINO_MAIN
#define __AVR__
#define __avr__
#define F_CPU 16000000L
#define __cplusplus
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__

#define __builtin_va_list
#define __builtin_va_start
#define __builtin_va_end
#define __DOXYGEN__
#define __attribute__(x)
#define NOINLINE __attribute__((noinline))
#define prog_void
#define PGM_VOID_P int
            
typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {;}

//
//
static void ahrs_update();
static void update_speed_height(void);
static void update_mount(void);
static void update_compass(void);
static void compass_accumulate(void);
static void barometer_accumulate(void);
static void update_logging1(void);
static void update_logging2(void);
static void obc_fs_check(void);
static void update_aux(void);
static void one_second_loop();
static void log_perf_info();
static void compass_save();
static void airspeed_ratio_update(void);
static void update_GPS_50Hz(void);
static void update_GPS_10Hz(void);
static void handle_auto_mode(void);
static void update_flight_mode(void);
static void update_navigation();
static void update_alt();
static void update_abc_controller();
static void debug_print();
static float get_speed_scaler(void);
static bool stick_mixing_enabled(void);
static void stabilize_roll(float speed_scaler);
static void stabilize_pitch(float speed_scaler);
static void stick_mix_channel(RC_Channel *channel, int16_t &servo_out);
static void stabilize_stick_mixing_direct();
static void stabilize_stick_mixing_fbw();
static void stabilize_yaw(float speed_scaler);
static void stabilize_training(float speed_scaler);
static void stabilize_acro(float speed_scaler);
static void stabilize();
static void calc_throttle();
static void calc_nav_yaw_coordinated(float speed_scaler);
static void calc_nav_yaw_course(void);
static void calc_nav_yaw_ground(void);
static void calc_nav_pitch();
static void calc_nav_roll();
static void throttle_slew_limit(int16_t last_throttle);
static bool auto_takeoff_check(void);
static bool is_flying(void);
static bool suppress_throttle(void);
static void channel_output_mixer(uint8_t mixing_type, int16_t &chan1_out, int16_t &chan2_out);
static void flaperon_update(int8_t flap_percent);
static void set_servos(void);
static void demo_servos(uint8_t i);
static NOINLINE void send_heartbeat(mavlink_channel_t chan);
static NOINLINE void send_attitude(mavlink_channel_t chan);
static NOINLINE void send_fence_status(mavlink_channel_t chan);
static NOINLINE void send_extended_status1(mavlink_channel_t chan);
static void NOINLINE send_location(mavlink_channel_t chan);
static void NOINLINE send_nav_controller_output(mavlink_channel_t chan);
static void NOINLINE send_gps_raw(mavlink_channel_t chan);
static void NOINLINE send_system_time(mavlink_channel_t chan);
void NOINLINE send_servo_out(mavlink_channel_t chan);
static void NOINLINE send_radio_in(mavlink_channel_t chan);
static void NOINLINE send_radio_out(mavlink_channel_t chan);
static void NOINLINE send_vfr_hud(mavlink_channel_t chan);
static void NOINLINE send_raw_imu1(mavlink_channel_t chan);
static void NOINLINE send_raw_imu2(mavlink_channel_t chan);
static void NOINLINE send_raw_imu3(mavlink_channel_t chan);
static void NOINLINE send_ahrs(mavlink_channel_t chan);
static void NOINLINE send_simstate(mavlink_channel_t chan);
static void NOINLINE send_hwstatus(mavlink_channel_t chan);
static void NOINLINE send_wind(mavlink_channel_t chan);
static void NOINLINE send_rangefinder(mavlink_channel_t chan);
static void NOINLINE send_current_waypoint(mavlink_channel_t chan);
static void NOINLINE send_statustext(mavlink_channel_t chan);
static bool telemetry_delayed(mavlink_channel_t chan);
static void mavlink_delay_cb();
static void gcs_send_message(enum ap_message id);
static void gcs_data_stream_send(void);
static void gcs_update(void);
static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str);
static void gcs_send_airspeed_calibration(const Vector3f &vg);
static void gcs_retry_deferred(void);
static bool print_log_menu(void);
static void do_erase_logs(void);
static void Log_Write_Attitude(void);
static void Log_Write_Performance();
static void Log_Write_Camera();
static void Log_Write_Startup(uint8_t type);
static void Log_Write_Control_Tuning();
static void Log_Write_TECS_Tuning(void);
static void Log_Write_Nav_Tuning();
static void Log_Write_Mode(uint8_t mode);
static void Log_Write_Sonar();
static void Log_Write_Current();
static void Log_Arm_Disarm();
static void Log_Write_Compass();
static void Log_Write_GPS(uint8_t instance);
static void Log_Write_IMU();
static void Log_Write_RC(void);
static void Log_Write_Baro(void);
static void Log_Write_Airspeed(void);
static void Log_Write_ABC();
static void Log_Read(uint16_t log_num, int16_t start_page, int16_t end_page);
static void start_logging();
static void Log_Write_Startup(uint8_t type);
static void Log_Write_Current();
static void Log_Write_Nav_Tuning();
static void Log_Write_TECS_Tuning();
static void Log_Write_Performance();
static void Log_Write_Attitude();
static void Log_Write_Control_Tuning();
static void Log_Write_Camera();
static void Log_Write_Mode(uint8_t mode);
static void Log_Write_Compass();
static void Log_Write_GPS(uint8_t instance);
static void Log_Write_IMU();
static void Log_Write_RC();
static void Log_Write_Airspeed(void);
static void Log_Write_Baro(void);
static void load_parameters(void);
void abc_controller(float input[16],float output[5]);
float Derivative(float Prev_value, float Current_value, float Delta_t);
float lag_filter(float X, float tau, float Y_last);
void add_altitude_data(unsigned long xl, long y);
static int32_t read_alt_to_hold();
static void set_next_WP(const struct Location& loc);
static void set_guided_WP(void);
static void init_home();
static void update_home();
static void do_RTL(void);
static bool verify_takeoff();
static bool verify_land();
static bool verify_nav_wp();
static bool verify_loiter_unlim();
static bool verify_loiter_time();
static bool verify_loiter_turns();
static bool verify_RTL();
static bool verify_wait_delay();
static bool verify_change_alt();
static bool verify_within_distance();
static void do_loiter_at_location();
static void do_take_picture();
static void exit_mission_callback();
static void update_commands(void);
static void delay(uint32_t ms);
static void mavlink_delay(uint32_t ms);
static uint32_t millis();
static uint32_t micros();
static void read_control_switch();
static uint8_t readSwitch(void);
static void reset_control_switch();
static void autotune_start(void);
static void autotune_restore(void);
static void failsafe_short_on_event(enum failsafe_state fstype);
static void failsafe_long_on_event(enum failsafe_state fstype);
static void failsafe_short_off_event();
void low_battery_event(void);
static void update_events(void);
void failsafe_check(void);
static Vector2l get_fence_point_with_index(unsigned i);
static void set_fence_point_with_index(Vector2l &point, unsigned i);
static void geofence_load(void);
static bool geofence_present(void);
static void geofence_update_pwm_enabled_state();
static bool geofence_set_enabled(bool enable, GeofenceEnableReason r);
static bool geofence_enabled(void);
static bool geofence_check_minalt(void);
static bool geofence_check_maxalt(void);
static void geofence_check(bool altitude_check_only);
static bool geofence_stickmixing(void);
static void geofence_send_status(mavlink_channel_t chan);
bool geofence_breached(void);
static void geofence_check(bool altitude_check_only);
static bool geofence_stickmixing(void);
static bool geofence_enabled(void);
static bool geofence_present(void);
static bool geofence_set_enabled(bool enable, GeofenceEnableReason r);
bool geofence_breached(void);
static void set_nav_controller(void);
static void loiter_angle_reset(void);
static void loiter_angle_update(void);
static void navigate();
static void calc_airspeed_errors();
static void calc_gndspeed_undershoot();
static void calc_altitude_error();
static void update_loiter();
static void update_cruise();
static void update_fbwb_speed_height(void);
static void setup_glide_slope(void);
static float relative_altitude(void);
static int32_t relative_altitude_abs_cm(void);
static void set_control_channels(void);
static void init_rc_in();
static void init_rc_out();
static void rudder_arm_check();
static void read_radio();
static void control_failsafe(uint16_t pwm);
static void trim_control_surfaces();
static void trim_radio();
static bool rc_failsafe_active(void);
static void init_barometer(void);
static void init_sonar(void);
static void read_sonars(void);
static void read_airspeed(void);
static void zero_airspeed(void);
static void read_battery(void);
void read_receiver_rssi(void);
static int32_t adjusted_altitude_cm(void);
static void report_radio();
static void report_ins();
static void report_compass();
static void report_flight_modes();
static void print_radio_values();
static void print_switch(uint8_t p, uint8_t m);
static void print_done();
static void print_blanks(int16_t num);
static void print_divider(void);
static int8_t radio_input_switch(void);
static void zero_eeprom(void);
static void print_enabled(bool b);
static void print_accel_offsets_and_scaling(void);
static void print_gyro_offsets(void);
static void init_ardupilot();
static void startup_ground(void);
static enum FlightMode get_previous_mode();
static void set_mode(enum FlightMode mode);
static void exit_mode(enum FlightMode mode);
static void check_long_failsafe();
static void check_short_failsafe();
static void startup_INS_ground(bool do_accel_init);
static void update_notify();
static void resetPerfData(void);
static uint32_t map_baudrate(int8_t rate, uint32_t default_baud);
static void check_usb_mux(void);
static void print_comma(void);
static void servo_write(uint8_t ch, uint16_t pwm);
static bool should_log(uint32_t mask);
static void print_hit_enter();

#include "C:\Program Files (x86)\Arduino\hardware\arduino\cores\arduino\arduino.h"
#include "C:\Program Files (x86)\Arduino\hardware\arduino\variants\standard\pins_arduino.h" 
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\ArduPlane.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\APM_Config.h"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\Attitude.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\GCS_Mavlink.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\Log.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\Parameters.h"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\Parameters.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\abc_controller.h"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\abc_controller.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\abc_controller_type.h"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\climb_rate.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\commands.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\commands_logic.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\commands_process.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\compat.h"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\compat.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\config.h"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\control_modes.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\defines.h"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\events.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\failsafe.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\geofence.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\navigation.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\radio.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\sensors.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\setup.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\system.pde"
#include "C:\Users\p588w525\Documents\GitHub\ardupilot_baseline\ardupilot_3.0_HIL\ArduPlane\test.pde"
#endif
