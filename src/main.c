/**
 * TEAM MOTOREX - INTEGRATED VEHICLE TELEMETRY OS
 * CORE INITIALIZATION & GRAPHICS CONTROLLER FOR SAE BAJA 2026
 * LVGL 9.6 COMPATIBLE - NATIVE 1024x600 RASPBERRY PI LIVE SOCKETCAN BUILD
 */

#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdbool.h>

#ifdef _MSC_VER
#include <Windows.h>
#else
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#endif

#include "lvgl/lvgl.h"
#include <SDL.h>
#include "hal/hal.h"

/************************************************
 * VEHICLE PARAMETERS
 ************************************************/
#define GEAR_RATIO 5.7f
#define WHEEL_CIRCUMFERENCE 1.95f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/************************************************
 * STEERING PROTOCOL PARAMETERS (must match the
 * Teensy SBW controller firmware exactly)
 ************************************************/
#define STEER_ENC_OFFSET     0.873f   // encoded = (wheelAngleRad + STEER_ENC_OFFSET) / STEER_ENC_SCALE
#define STEER_ENC_SCALE      0.001f
#define STEER_RATIO          3.5f     // steeringWheelAngle = wheelAngle * STEER_RATIO
#define STEER_ANGLE_MIN      (-60.0f) // Target angle input limits (degrees)
#define STEER_ANGLE_MAX      (60.0f)
#define STEER_CAN_CMD_ID      0x103   // Target angle command: Linux -> Teensy (uint16 encoded, 2 bytes, LE)
#define STEER_CAN_STATUS_ID   0x063   // AS5600 encoder status: Teensy -> Linux (byte0: 0=No Error,1=Error)
#define STEER_CAN_FEEDBACK_ID 0x203   // Steering angle feedback: Teensy -> Linux (uint16 encoded, 2 bytes, LE)

/* ---- CAN MODIFICATION: Autonomous mode selection CAN ID (Linux -> Jetson Nano) ---- */
#define AUTONOMOUS_MODE_CAN_ID 0x400  // Mode select command: Linux -> Jetson Nano (uint16, 2 bytes, LE)

/************************************************
 * MOTOR CONTROLLER (EZkontrol MCU->METER) CAN IDs
 * Reference: EZkontrol MCU to Meter CAN Protocols V1.1
 ************************************************/
#define MC_CAN_ID_MSG1        0x180117EFUL  // Bus Voltage / Bus Current / Phase Current / Speed
#define MC_CAN_ID_MSG2        0x180217EFUL  // Controller Temp / Motor Temp / Accel% / Gear / Brake / Errors

/************************************************
 * EXTERNAL IMAGE ASSET DECLARATIONS
 ************************************************/
#include "motorex_bat.c" 
#include "steering_wheel.c" 
#include "car_chassis.c" 
#include "throttle_pedal.c" 
#include "brake_rotor.c"

/************************************************
 * TYPE DEFINITIONS & OPERATIONAL STATES
 ************************************************/
typedef enum {
    PAGE_HOME = 0,
    PAGE_VEHICLE,
    PAGE_SENSORS,
    PAGE_STEERING,
    PAGE_MODE, 
    PAGE_ERRORS,
    PAGE_SETTINGS
} runtime_page_t;

static runtime_page_t current_page = PAGE_HOME;

// Global Layout Master Viewport Containers
static lv_obj_t *main_screen;
static lv_obj_t *header_container;
static lv_obj_t *page_container;
static lv_obj_t *footer_container;

// Header Display Widgets
static lv_obj_t *header_time_label;
static lv_obj_t *dot_auto = NULL;
static lv_obj_t *dot_lat = NULL;
static lv_obj_t *dot_long = NULL;
static lv_obj_t *dot_camera = NULL;
static lv_obj_t *dot_radar = NULL;
static lv_obj_t *dot_imu = NULL;

// Home Page Boot Sequencer References
static lv_obj_t *loading_bar;
static lv_obj_t *status_label;
static lv_obj_t *lbl_boot_pct;

// Vehicle Page Live Telemetry Indicators
static lv_obj_t *lbl_steer_angle = NULL;
static lv_obj_t *lbl_throttle_cmd = NULL;   // now: Throttle Percentage
static lv_obj_t *lbl_brake_cmd = NULL;      // now: Battery Voltage
static lv_obj_t *lbl_velocity = NULL;
static lv_obj_t *lbl_g_force = NULL;        // now: Motor RPM
static lv_obj_t *lbl_brake_pressure = NULL; // now: Gear
static lv_obj_t *lbl_wheel_fl = NULL;
static lv_obj_t *lbl_wheel_fr = NULL;
static lv_obj_t *lbl_wheel_rl = NULL;
static lv_obj_t *lbl_wheel_rr = NULL;

// Steering Page Live Telemetry Indicators
static lv_obj_t *lbl_st_curr_angle = NULL;
static lv_obj_t *lbl_st_wheel_angle = NULL;
static lv_obj_t *lbl_st_motor_curr = NULL;
static lv_obj_t *lbl_st_torque = NULL;
static lv_obj_t *lbl_st_rate = NULL;        // now (on Mode/Throttle tab): Motor Temperature
static lv_obj_t *lbl_st_encoder = NULL;
static lv_obj_t *steering_wheel_arc = NULL;
static lv_obj_t *img_steering_wheel = NULL; 
static lv_obj_t *ta_target_angle = NULL;    // Steering Page: "Enter Target Angle" input box
static lv_obj_t *btn_target_ok = NULL;      // Steering Page: OK button (sends command over CAN)
static lv_obj_t *lbl_sensor_status = NULL;  // Steering Page: Steering Sensor Status (Error/No Error)
static lv_obj_t *keypad_slot = NULL;        // Steering Page: reserved area under the status card
static lv_obj_t *target_keypad = NULL;      // Steering Page: custom numeric keypad (built inside keypad_slot)

// SUB-NAV MODE OPTION REGISTERS
// Tab order: 0=MODE OPTIONS, 1=THROTTLE, 2=BRAKE
static int current_sub_mode_tab = 0;       
static int selected_mission_preset_idx = -1; // -1 = none active

// Active state for each autonomous mode: 0=STANDBY, 1=AEB, 2=LKA, 3=ENDURANCE
static bool mode_active[4] = {true, false, false, false};
/* ---- CAN MODIFICATION: tracks the mode value (0-3) last transmitted over CAN ID 0x400.
 * Defaults to 0 (Standby) to match the default active mode above; used so we only
 * send a CAN frame when the mode actually changes. ---- */
static int can_active_mode = 0;
static int pending_confirm_mode_idx = -1; // which mode is pending confirmation
static lv_obj_t * mode_overlay = NULL; // tracks active dialog overlay

// Live Sub-Nav Diagnostic Graphical Pointers (Throttle)
static lv_obj_t *throttle_live_arc = NULL; 
static lv_obj_t *lbl_throttle_txt_metrics = NULL;
static lv_obj_t *lbl_controller_temp = NULL; // Mode/Throttle tab: Controller Temperature value

// Live Sub-Nav Diagnostic Graphical Pointers (Brake)
static lv_obj_t *brake_live_arc = NULL;
static lv_obj_t *lbl_brake_cmd_pct = NULL;
static lv_obj_t *brake_cmd_bar = NULL;
static lv_obj_t *lbl_brake_current = NULL;
static lv_obj_t *lbl_brake_pressure_metrics = NULL;
static lv_obj_t *lbl_brake_actuator = NULL;
static lv_obj_t *brake_act_arc = NULL;

// Dynamic Error Page Pointers
static lv_obj_t *fault_card_obj = NULL;
static lv_obj_t *lbl_err_title = NULL;
static lv_obj_t *lbl_err_desc = NULL;

// Global Operational Telemetry Variable (Thread Safe State)
typedef struct {
    int boot_progress;
    int vehicle_speed_kmh;
    int motor_rpm;
    int wheel_rpm;           
    float steer_angle_deg;
    float throttle_pct;        // derived SOLELY from vehicle_speed_kmh (single source of truth)
    float brake_pct;
    float battery_voltage;     // decoded Bus Voltage (0x180117EF, bytes 0-1)
    int controller_temp;       // decoded Controller Temperature (0x180217EF, byte 0)
    int motor_temp;            // decoded Motor Temperature (0x180217EF, byte 1)
    int gear_raw;              // decoded Gear (0x180217EF, byte 3 bits 2-0)
    int steering_status; 
    int ind_auto;
    int ind_lat;
    int ind_long;
} vehicle_telemetry_t;

static volatile vehicle_telemetry_t live_car_data = {
    .boot_progress = 20,
    .vehicle_speed_kmh = 0,
    .motor_rpm = 0,
    .wheel_rpm = 0,          
    .steer_angle_deg = 0.0f,
    .throttle_pct = 0.0f,
    .brake_pct = 0.0f,
    .battery_voltage = 0.0f,
    .controller_temp = 0,
    .motor_temp = 0,
    .gear_raw = 0,
    .steering_status = 0,
    .ind_auto = 0,
    .ind_lat = 0,
    .ind_long = 0
};

// Internal baseline variables
static int system_clock_seconds = 0;
#ifndef _MSC_VER
static pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

/************************************************
 * CRITICAL FUNCTION PROTOYPES FORWARD-DECLARATIONS
 ************************************************/
static void clear_page_telemetry_pointers(void);
static void nav_button_callback(lv_event_t * e);
static lv_obj_t* create_header_status_pill(lv_obj_t * parent, const char * name, lv_color_t dot_color, int x_offset, lv_align_t align);
static void create_nav_button(const char * icon, const char * name, runtime_page_t page_id);
static void build_footer_menu(void);
static void draw_home_page(void);
static void draw_vehicle_page(void);
static void draw_sensors_page(void);
static void draw_steering_page(void);
static void draw_mode_page(void); 
static void draw_errors_page(void);
static void draw_settings_page(void);
static void sub_mode_tab_click_callback(lv_event_t * e);
static void preset_grid_click_callback(lv_event_t * e);
static void mode_confirm_yes_cb(lv_event_t * e);
static void mode_confirm_no_cb(lv_event_t * e);
static void application_background_loop(lv_timer_t *timer);
static void motorex_ui_init(void);
static void pulse_widget(lv_obj_t * obj);
static void target_ta_clicked_cb(lv_event_t * e);
static void target_ok_cb(lv_event_t * e);
static void target_inc_cb(lv_event_t * e);
static void target_dec_cb(lv_event_t * e);
static void build_target_keypad(void);
static void keypad_digit_cb(lv_event_t * e);
static void keypad_backspace_cb(lv_event_t * e);
static void keypad_sign_cb(lv_event_t * e);
static void keypad_confirm_cb(lv_event_t * e);
static void confirm_and_send_target(void);
static void can_send_target_angle(float target_deg);
/* ---- CAN MODIFICATION: forward declaration for autonomous mode CAN transmit helper ---- */
static void can_send_mode(uint16_t mode);
static const char * gear_to_string(int gear_raw);
#ifndef _MSC_VER
static void* background_socketcan_thread(void* arg);
static void can_tx_init(void);
#endif

/************************************************
 * GEAR RAW VALUE -> DISPLAY STRING
 * Per EZkontrol protocol byte3 bits[2:0]:
 * 0=NO,1=R,2=N,3=D1,4=D2,5=D3,6=S,7=P
 ************************************************/
static const char * gear_to_string(int gear_raw)
{
    switch(gear_raw) {
        case 1: return "R";
        case 2: return "N";
        case 3: return "D1";
        case 4: return "D2";
        case 5: return "D3";
        case 6: return "S";
        case 7: return "P";
        default: return "N";
    }
}

/************************************************
 * SAFE TELEMETRY POINTER RESET DECK
 ************************************************/
static void clear_page_telemetry_pointers(void)
{
    lbl_steer_angle = NULL; lbl_throttle_cmd = NULL; lbl_brake_cmd = NULL;
    lbl_velocity = NULL; lbl_g_force = NULL; lbl_brake_pressure = NULL;
    lbl_wheel_fl = NULL; lbl_wheel_fr = NULL; lbl_wheel_rl = NULL; lbl_wheel_rr = NULL;
    lbl_st_curr_angle = NULL; lbl_st_wheel_angle = NULL; lbl_st_motor_curr = NULL;
    lbl_st_torque = NULL; lbl_st_rate = NULL; lbl_st_encoder = NULL;
    steering_wheel_arc = NULL; img_steering_wheel = NULL;
    ta_target_angle = NULL; btn_target_ok = NULL; lbl_sensor_status = NULL;
    // keypad_slot / target_keypad live inside page_container, so lv_obj_clean()
    // (called just before this function) already destroyed them — just null the pointers.
    keypad_slot = NULL; target_keypad = NULL;
    
    throttle_live_arc = NULL; lbl_throttle_txt_metrics = NULL; lbl_controller_temp = NULL;
    
    brake_live_arc = NULL; lbl_brake_cmd_pct = NULL; brake_cmd_bar = NULL;
    lbl_brake_current = NULL; lbl_brake_pressure_metrics = NULL; lbl_brake_actuator = NULL;
    brake_act_arc = NULL;

    fault_card_obj = NULL; lbl_err_title = NULL; lbl_err_desc = NULL;
}

/************************************************
 * NAVIGATION STATE CONTROLLER & VIEW SWAPPER
 ************************************************/
static void nav_button_callback(lv_event_t * e)
{
    lv_obj_t * btn = lv_event_get_target(e);
    runtime_page_t target_page = (runtime_page_t)(intptr_t)lv_obj_get_user_data(btn);
    
    if(current_page == target_page) return;
    current_page = target_page;

    lv_obj_clean(page_container);
    clear_page_telemetry_pointers(); 
    
    switch(current_page) {
        case PAGE_HOME:     draw_home_page();     break;
        case PAGE_VEHICLE:  draw_vehicle_page();  break;
        case PAGE_SENSORS:  draw_sensors_page();  break;
        case PAGE_STEERING: draw_steering_page(); break;
        case PAGE_MODE:     draw_mode_page();     break; 
        case PAGE_ERRORS:   draw_errors_page();   break;
        case PAGE_SETTINGS: draw_settings_page(); break;
    }
}

/************************************************
 * SENSOR STATUS PILL BUILDER (SCALED FOR 1024)
 ************************************************/
static lv_obj_t* create_header_status_pill(lv_obj_t * parent, const char * name, lv_color_t dot_color, int x_offset, lv_align_t align)
{
    lv_obj_t * pill = lv_obj_create(parent);
    lv_obj_set_size(pill, 95, 35);
    lv_obj_set_style_bg_opa(pill, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(pill, 0, 0);
    lv_obj_set_style_pad_all(pill, 0, 0);
    lv_obj_align(pill, align, x_offset, 0);
    lv_obj_clear_flag(pill, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * dot = lv_obj_create(pill);
    lv_obj_set_size(dot, 8, 8);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dot, dot_color, 0);
    lv_obj_set_style_border_width(dot, 0, 0);
    lv_obj_align(dot, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t * lbl = lv_label_create(pill);
    lv_label_set_text(lbl, name);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 15, 0);
    
    return dot;
}

/************************************************
 * SIMPLE OPACITY "PULSE" FEEDBACK ANIMATION
 * Used for the target-angle increment/decrement
 * boxes and the OK button so taps feel responsive.
 ************************************************/
static void pulse_anim_opa_cb(void * obj, int32_t v)
{
    lv_obj_set_style_opa((lv_obj_t*)obj, (lv_opa_t)v, 0);
}

static void pulse_widget(lv_obj_t * obj)
{
    if (obj == NULL) return;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_exec_cb(&a, pulse_anim_opa_cb);
    lv_anim_set_values(&a, LV_OPA_50, LV_OPA_COVER);
    lv_anim_set_time(&a, 120);
    lv_anim_set_playback_time(&a, 120);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

/************************************************
 * NAVIGATION BUTTON BUILDER (SCALED FOR 1024)
 ************************************************/
static void create_nav_button(const char * icon, const char * name, runtime_page_t page_id)
{
    lv_obj_t * btn = lv_button_create(footer_container);
    lv_obj_set_size(btn, 132, 75); 
    lv_obj_set_user_data(btn, (void*)(intptr_t)page_id);
    
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x121212), 0);
    lv_obj_set_style_border_width(btn, 1, 0);
    lv_obj_set_style_border_color(btn, lv_color_hex(0x222222), 0);
    lv_obj_set_style_radius(btn, 10, 0); 

    if(page_id == current_page) {
        lv_obj_set_style_border_color(btn, lv_color_hex(0xFF1E27), 0);
        lv_obj_set_style_border_width(btn, 2, 0);
    }

    lv_obj_t * icon_lbl = lv_label_create(btn);
    lv_label_set_text(icon_lbl, icon);
    lv_obj_set_style_text_font(icon_lbl, &lv_font_montserrat_18, 0);
    lv_obj_align(icon_lbl, LV_ALIGN_TOP_MID, 0, 8);

    lv_obj_t * text_lbl = lv_label_create(btn);
    lv_label_set_text(text_lbl, name);
    lv_obj_set_style_text_font(text_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(text_lbl, LV_ALIGN_BOTTOM_MID, 0, -8);
    
    if(page_id == current_page) {
        lv_obj_set_style_text_color(icon_lbl, lv_color_hex(0xFF1E27), 0);
        lv_obj_set_style_text_color(text_lbl, lv_color_hex(0xFF1E27), 0); 
    } else {
        lv_obj_set_style_text_color(icon_lbl, lv_color_hex(0x666666), 0);
        lv_obj_set_style_text_color(text_lbl, lv_color_hex(0x666666), 0); 
    }

    lv_obj_add_event_cb(btn, nav_button_callback, LV_EVENT_CLICKED, NULL);
}

static void build_footer_menu(void)
{
    lv_obj_clean(footer_container);
    create_nav_button(LV_SYMBOL_HOME,     "Home",     PAGE_HOME);
    create_nav_button(LV_SYMBOL_DRIVE,    "Vehicle",  PAGE_VEHICLE);
    create_nav_button(LV_SYMBOL_LIST,     "Sensors",  PAGE_SENSORS);
    create_nav_button(LV_SYMBOL_REFRESH,  "Steering", PAGE_STEERING); 
    create_nav_button(LV_SYMBOL_SHUFFLE,  "Mode",     PAGE_MODE); 
    create_nav_button(LV_SYMBOL_WARNING,  "Errors",   PAGE_ERRORS);
    create_nav_button(LV_SYMBOL_SETTINGS, "Settings", PAGE_SETTINGS);
}

/************************************************
 * MASTER STRUCTURAL VIEWPORT CONTROLLER
 ************************************************/
static void motorex_ui_init(void) 
{
    main_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(main_screen, lv_color_hex(0x0A0A0A), 0); 
    lv_scr_load(main_screen);

    lv_obj_set_style_border_width(main_screen, 3, 0);
    lv_obj_set_style_border_color(main_screen, lv_color_hex(0xFF1E27), 0); 
    lv_obj_set_style_pad_all(main_screen, 2, 0);

    lv_obj_set_layout(main_screen, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(main_screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(main_screen, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(main_screen, 0, 0);

    header_container = lv_obj_create(main_screen);
    lv_obj_set_size(header_container, lv_pct(100), 55);
    lv_obj_set_style_bg_color(header_container, lv_color_hex(0x0E0E0E), 0); 
    lv_obj_set_style_border_side(header_container, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_width(header_container, 1, 0);
    lv_obj_set_style_border_color(header_container, lv_color_hex(0x222222), 0);
    lv_obj_set_style_radius(header_container, 0, 0);
    lv_obj_clear_flag(header_container, LV_OBJ_FLAG_SCROLLABLE);
    
    lv_obj_t *logo_lbl = lv_label_create(header_container);
    lv_label_set_text(logo_lbl, "TEAM MOTOREX");
    lv_obj_set_style_text_font(logo_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(logo_lbl, lv_color_white(), 0);
    lv_obj_align(logo_lbl, LV_ALIGN_LEFT_MID, 15, 0);

    header_time_label = lv_label_create(header_container);
    lv_label_set_text(header_time_label, "Loading...");
    lv_obj_set_style_text_font(header_time_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(header_time_label, lv_color_hex(0xDDDDDD), 0);
    lv_obj_align(header_time_label, LV_ALIGN_CENTER, 0, 0);

    dot_imu = create_header_status_pill(header_container, "IMU", lv_color_hex(0x444444), -10, LV_ALIGN_RIGHT_MID);
    dot_radar = create_header_status_pill(header_container, "Radar", lv_color_hex(0x444444), -115, LV_ALIGN_RIGHT_MID); 
    dot_camera = create_header_status_pill(header_container, "Camera", lv_color_hex(0x444444), -220, LV_ALIGN_RIGHT_MID);

    dot_auto = create_header_status_pill(header_container, "AUTO", lv_color_hex(0x444444), 180, LV_ALIGN_LEFT_MID);
    dot_lat  = create_header_status_pill(header_container, "LAT",  lv_color_hex(0x444444), 260, LV_ALIGN_LEFT_MID);
    dot_long = create_header_status_pill(header_container, "LONG", lv_color_hex(0x444444), 340, LV_ALIGN_LEFT_MID);

    page_container = lv_obj_create(main_screen);
    lv_obj_set_size(page_container, lv_pct(100), 440); 
    lv_obj_set_style_bg_color(page_container, lv_color_hex(0x0A0A0A), 0); 
    lv_obj_set_style_border_width(page_container, 0, 0);
    lv_obj_set_style_radius(page_container, 0, 0);
    lv_obj_clear_flag(page_container, LV_OBJ_FLAG_SCROLLABLE);

    footer_container = lv_obj_create(main_screen);
    lv_obj_set_size(footer_container, lv_pct(100), 90); 
    lv_obj_set_style_bg_color(footer_container, lv_color_hex(0x0A0A0A), 0); 
    lv_obj_set_style_border_width(footer_container, 0, 0);
    lv_obj_set_style_radius(footer_container, 0, 0);
    lv_obj_set_layout(footer_container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(footer_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(footer_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(footer_container, LV_OBJ_FLAG_SCROLLABLE);
}

/************************************************
 * VIEW PANELS: PAGE 1 - HOME NATIVE SCALE
 ************************************************/
static void draw_home_page(void) 
{
    build_footer_menu(); 
    
    lv_obj_set_layout(page_container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(page_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(page_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(page_container, 10, 0); 

    lv_obj_t * logo_img = lv_image_create(page_container);
    lv_image_set_src(logo_img, &motorex_bat);
    lv_obj_set_size(logo_img, 240, 110); 
    lv_image_set_scale(logo_img, 85);    
    
    lv_obj_t *main_title = lv_label_create(page_container);
    lv_label_set_text(main_title, "TEAM MOTOREX - aBAJA 2026");
    lv_obj_set_style_text_font(main_title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(main_title, lv_color_white(), 0);
    
    status_label = lv_label_create(page_container);
    lv_label_set_text(status_label, live_car_data.boot_progress < 100 ? "Vehicle Initializing..." : "READY TO DRIVE");
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(status_label, live_car_data.boot_progress < 100 ? lv_color_hex(0xAAAAAA) : lv_color_hex(0x00FF66), 0);

    lv_obj_t * bar_row = lv_obj_create(page_container);
    lv_obj_set_size(bar_row, 500, 35);
    lv_obj_set_style_bg_opa(bar_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(bar_row, 0, 0);
    lv_obj_set_layout(bar_row, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(bar_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(bar_row, LV_OBJ_FLAG_SCROLLABLE);

    loading_bar = lv_bar_create(bar_row);
    lv_obj_set_size(loading_bar, 400, 12); 
    lv_bar_set_value(loading_bar, live_car_data.boot_progress, LV_ANIM_OFF); 
    lv_obj_set_style_bg_color(loading_bar, lv_color_hex(0x222222), LV_PART_MAIN);
    lv_obj_set_style_bg_color(loading_bar, lv_color_hex(0xFF1E27), LV_PART_INDICATOR); 
    lv_obj_set_style_radius(loading_bar, 6, 0);

    lbl_boot_pct = lv_label_create(bar_row);
    char buf[48];
    snprintf(buf, sizeof(buf), " %d%%", live_car_data.boot_progress);
    lv_label_set_text(lbl_boot_pct, buf);
    lv_obj_set_style_text_font(lbl_boot_pct, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(lbl_boot_pct, lv_color_hex(0xAAAAAA), 0);

    lv_obj_t *checklist_label = lv_label_create(page_container);
    lv_label_set_text(checklist_label, 
        LV_SYMBOL_OK " Steering    " LV_SYMBOL_OK " Brake    "
        LV_SYMBOL_OK " Motor    " LV_SYMBOL_OK " Battery    "
        LV_SYMBOL_OK " Camera    " LV_SYMBOL_OK " Radar    "
        LV_SYMBOL_OK " GPS"
    );
    lv_obj_set_style_text_font(checklist_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(checklist_label, lv_color_hex(0x00FF66), 0);
}

/************************************************
 * VIEW PANELS: PAGE 2 - CORE TELEMETRY NATIVE
 * Tiles now show:
 *   Left col:  Steering Angle | Throttle Percentage | Battery Voltage
 *   Right col: Vehicle Speed  | Motor RPM            | Gear
 ************************************************/
static void draw_vehicle_page(void)
{
    build_footer_menu(); 
    
    lv_obj_set_layout(page_container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(page_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(page_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(page_container, 5, 0);

    // COLUMN 1: LEFT SIDE DATA MATRIX
    lv_obj_t * col_left = lv_obj_create(page_container);
    lv_obj_set_size(col_left, 290, 410);
    lv_obj_set_style_bg_opa(col_left, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(col_left, 0, 0);
    lv_obj_set_layout(col_left, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(col_left, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(col_left, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(col_left, LV_OBJ_FLAG_SCROLLABLE);

    // Card 1: Steering
    lv_obj_t * card_steer = lv_obj_create(col_left);
    lv_obj_set_size(card_steer, 280, 120);
    lv_obj_set_style_bg_color(card_steer, lv_color_hex(0x141414), 0);
    lv_obj_set_style_border_color(card_steer, lv_color_hex(0x252525), 0);
    lv_obj_set_style_radius(card_steer, 12, 0);
    lv_obj_set_layout(card_steer, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(card_steer, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(card_steer, 10, 0);
    lv_obj_clear_flag(card_steer, LV_OBJ_FLAG_SCROLLABLE);
    
    lv_obj_t * lbl_s_title = lv_label_create(card_steer);
    lv_label_set_text(lbl_s_title, "Steering Angle");
    lv_obj_set_style_text_font(lbl_s_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_s_title, lv_color_hex(0x888888), 0);

    lbl_steer_angle = lv_label_create(card_steer);
    lv_obj_set_style_text_font(lbl_steer_angle, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(lbl_steer_angle, lv_color_white(), 0);

    // Card 2: Throttle Percentage (was "Throttle Position")
    lv_obj_t * card_throttle = lv_obj_create(col_left);
    lv_obj_set_size(card_throttle, 280, 120);
    lv_obj_set_style_bg_color(card_throttle, lv_color_hex(0x141414), 0);
    lv_obj_set_style_border_color(card_throttle, lv_color_hex(0x252525), 0);
    lv_obj_set_style_radius(card_throttle, 12, 0);
    lv_obj_set_layout(card_throttle, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(card_throttle, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(card_throttle, 10, 0);
    lv_obj_clear_flag(card_throttle, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * lbl_t_title = lv_label_create(card_throttle);
    lv_label_set_text(lbl_t_title, "Throttle Percentage");
    lv_obj_set_style_text_font(lbl_t_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_t_title, lv_color_hex(0x888888), 0);

    lbl_throttle_cmd = lv_label_create(card_throttle);
    lv_obj_set_style_text_font(lbl_throttle_cmd, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(lbl_throttle_cmd, lv_color_hex(0x00FF66), 0); 

    // Card 3: Battery Voltage (was "Brake Displacement")
    lv_obj_t * card_brake = lv_obj_create(col_left);
    lv_obj_set_size(card_brake, 280, 120);
    lv_obj_set_style_bg_color(card_brake, lv_color_hex(0x141414), 0);
    lv_obj_set_style_border_color(card_brake, lv_color_hex(0x252525), 0);
    lv_obj_set_style_radius(card_brake, 12, 0);
    lv_obj_set_layout(card_brake, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(card_brake, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(card_brake, 10, 0);
    lv_obj_clear_flag(card_brake, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * lbl_b_title = lv_label_create(card_brake);
    lv_label_set_text(lbl_b_title, "Battery Voltage");
    lv_obj_set_style_text_font(lbl_b_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_b_title, lv_color_hex(0x888888), 0);

    lbl_brake_cmd = lv_label_create(card_brake);
    lv_obj_set_style_text_font(lbl_brake_cmd, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(lbl_brake_cmd, lv_color_hex(0xFF3344), 0); 

    // COLUMN 2: CENTER VEHICLE CHASSIS DECK
    lv_obj_t * col_center = lv_obj_create(page_container);
    lv_obj_set_size(col_center, 380, 410);
    lv_obj_set_style_bg_color(col_center, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_color(col_center, lv_color_hex(0x222222), 0);
    lv_obj_set_style_radius(col_center, 16, 0); 
    lv_obj_clear_flag(col_center, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * img_car = lv_image_create(col_center);
    lv_image_set_src(img_car, &car_chassis);
    lv_obj_set_size(img_car, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_image_set_scale(img_car, 65); 
    lv_obj_align(img_car, LV_ALIGN_CENTER, 0, 10);

    lbl_wheel_fl = lv_label_create(col_center);
    lv_obj_set_style_text_font(lbl_wheel_fl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_wheel_fl, lv_color_hex(0x00F5FF), 0);
    lv_obj_align(lbl_wheel_fl, LV_ALIGN_CENTER, -120, -70);

    lbl_wheel_fr = lv_label_create(col_center);
    lv_obj_set_style_text_font(lbl_wheel_fr, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_wheel_fr, lv_color_hex(0x00F5FF), 0);
    lv_obj_align(lbl_wheel_fr, LV_ALIGN_CENTER, 120, -70);

    lbl_wheel_rl = lv_label_create(col_center);
    lv_obj_set_style_text_font(lbl_wheel_rl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_wheel_rl, lv_color_hex(0x00F5FF), 0);
    lv_obj_align(lbl_wheel_rl, LV_ALIGN_CENTER, -120, 70);

    lbl_wheel_rr = lv_label_create(col_center);
    lv_obj_set_style_text_font(lbl_wheel_rr, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_wheel_rr, lv_color_hex(0x00F5FF), 0);
    lv_obj_align(lbl_wheel_rr, LV_ALIGN_CENTER, 120, 70);

    // COLUMN 3: RIGHT SIDE METRICS STACKS
    lv_obj_t * col_right = lv_obj_create(page_container);
    lv_obj_set_size(col_right, 290, 410);
    lv_obj_set_style_bg_opa(col_right, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(col_right, 0, 0);
    lv_obj_set_layout(col_right, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(col_right, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(col_right, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(col_right, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * card_vel = lv_obj_create(col_right);
    lv_obj_set_size(card_vel, 280, 120);
    lv_obj_set_style_bg_color(card_vel, lv_color_hex(0x141414), 0);
    lv_obj_set_style_border_color(card_vel, lv_color_hex(0x252525), 0);
    lv_obj_set_style_radius(card_vel, 12, 0);
    lv_obj_set_layout(card_vel, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(card_vel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(card_vel, 10, 0);
    lv_obj_clear_flag(card_vel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * lbl_v_title = lv_label_create(card_vel);
    lv_label_set_text(lbl_v_title, "Vehicle Speed");
    lv_obj_set_style_text_font(lbl_v_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_v_title, lv_color_hex(0x777777), 0);

    lbl_velocity = lv_label_create(card_vel);
    lv_obj_set_style_text_font(lbl_velocity, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(lbl_velocity, lv_color_white(), 0);

    // Card: Motor RPM (was "Vehicle Dynamics")
    lv_obj_t * card_imu = lv_obj_create(col_right);
    lv_obj_set_size(card_imu, 280, 120);
    lv_obj_set_style_bg_color(card_imu, lv_color_hex(0x141414), 0);
    lv_obj_set_style_border_color(card_imu, lv_color_hex(0x252525), 0);
    lv_obj_set_style_radius(card_imu, 12, 0);
    lv_obj_set_layout(card_imu, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(card_imu, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(card_imu, 10, 0);
    lv_obj_clear_flag(card_imu, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * lbl_g_title = lv_label_create(card_imu);
    lv_label_set_text(lbl_g_title, "Motor RPM");
    lv_obj_set_style_text_font(lbl_g_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_g_title, lv_color_hex(0x777777), 0);

    lbl_g_force = lv_label_create(card_imu);
    lv_obj_set_style_text_font(lbl_g_force, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_g_force, lv_color_white(), 0);

    // Card: Gear (was "Brake Status")
    lv_obj_t * card_press = lv_obj_create(col_right);
    lv_obj_set_size(card_press, 280, 120);
    lv_obj_set_style_bg_color(card_press, lv_color_hex(0x141414), 0);
    lv_obj_set_style_border_color(card_press, lv_color_hex(0x252525), 0);
    lv_obj_set_style_radius(card_press, 12, 0);
    lv_obj_set_layout(card_press, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(card_press, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(card_press, 10, 0);
    lv_obj_clear_flag(card_press, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * lbl_p_title = lv_label_create(card_press);
    lv_label_set_text(lbl_p_title, "Gear");
    lv_obj_set_style_text_font(lbl_p_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_p_title, lv_color_hex(0x777777), 0);

    lbl_brake_pressure = lv_label_create(card_press);
    lv_obj_set_style_text_font(lbl_brake_pressure, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(lbl_brake_pressure, lv_color_white(), 0);
}

/************************************************
 * VIEW PANELS: PAGE 3 - SENSORS HEALTH MODULE
 ************************************************/
static void draw_sensors_page(void)
{
    build_footer_menu(); 
    lv_obj_set_layout(page_container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(page_container, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(page_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(page_container, 15, 0);

    const char * sensors[]  = {"Front Camera", "Radar", "IMU", "Steering Sensor", "Wheel Speed Sensor", "Brake Pressure Sensor"};
    const bool sensor_connected[] = { true,           true,            true,          true,             true,             true       };

    for(int i = 0; i < 6; i++) {
        lv_obj_t * card = lv_obj_create(page_container);
        lv_obj_set_size(card, 300, 110);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x141414), 0);
        lv_obj_set_style_border_color(card, lv_color_hex(0x2A2A2A), 0);
        lv_obj_set_style_radius(card, 12, 0);

        lv_obj_t * title = lv_label_create(card);
        lv_label_set_text(title, sensors[i]);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(title, lv_color_white(), 0);
        lv_obj_align(title, LV_ALIGN_TOP_LEFT, 12, 10);

        lv_obj_t * value = lv_label_create(card);
        lv_label_set_text(value, sensor_connected[i] ? "STATUS: FUNCTIONAL" : "STATUS: NOT CONNECTED");
        lv_obj_set_style_text_font(value, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(value, sensor_connected[i] ? lv_color_hex(0x888888) : lv_color_hex(0xFF6666), 0);
        lv_obj_align(value, LV_ALIGN_TOP_LEFT, 12, 45); 

        lv_obj_t * dot = lv_obj_create(card);
        lv_obj_set_size(dot, 10, 10);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, sensor_connected[i] ? lv_color_hex(0x00FF66) : lv_color_hex(0xFF3333), 0); 
        lv_obj_align(dot, LV_ALIGN_TOP_RIGHT, -12, 14);
    }
}

/************************************************
 * VIEW PANELS: PAGE 4 - NATIVE RUNTIME STEERING
 ************************************************/
static void draw_steering_page(void)
{
    build_footer_menu(); 
    lv_obj_set_layout(page_container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(page_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(page_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(page_container, 5, 0);

    // COLUMN 1: LEFT HAND ANGLE STACKS
    lv_obj_t * col_left = lv_obj_create(page_container);
    lv_obj_set_size(col_left, 290, 410);
    lv_obj_set_style_bg_opa(col_left, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(col_left, 0, 0);
    lv_obj_set_layout(col_left, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(col_left, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(col_left, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(col_left, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * card_curr = lv_obj_create(col_left);
    lv_obj_set_size(card_curr, 280, 120);
    lv_obj_set_style_bg_color(card_curr, lv_color_hex(0x141414), 0);
    lv_obj_set_style_border_color(card_curr, lv_color_hex(0x252525), 0);
    lv_obj_set_style_radius(card_curr, 12, 0);
    
    lv_obj_t * lbl_c_title = lv_label_create(card_curr);
    lv_label_set_text(lbl_c_title, "Current Angle");
    lv_obj_set_style_text_font(lbl_c_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_c_title, lv_color_hex(0x888888), 0);
    lv_obj_align(lbl_c_title, LV_ALIGN_TOP_LEFT, 12, 10);

    lbl_st_curr_angle = lv_label_create(card_curr);
    lv_obj_set_style_text_font(lbl_st_curr_angle, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(lbl_st_curr_angle, lv_color_hex(0xFF3344), 0); 
    lv_obj_align(lbl_st_curr_angle, LV_ALIGN_BOTTOM_LEFT, 12, -10);

    // ---- TARGET ANGLE INPUT CARD ----
    // "Enter Target Angle" editable box + OK button (sends via CAN)
    lv_obj_t * card_target = lv_obj_create(col_left);
    lv_obj_set_size(card_target, 280, 120);
    lv_obj_set_style_bg_color(card_target, lv_color_hex(0x141414), 0);
    lv_obj_set_style_border_color(card_target, lv_color_hex(0x252525), 0);
    lv_obj_set_style_radius(card_target, 12, 0);
    lv_obj_clear_flag(card_target, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * lbl_t_title = lv_label_create(card_target);
    lv_label_set_text(lbl_t_title, "Enter Target Angle");
    lv_obj_set_style_text_font(lbl_t_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_t_title, lv_color_hex(0x888888), 0);
    // Nudged slightly higher (was y=10) so it reads more clearly above the input row.
    lv_obj_align(lbl_t_title, LV_ALIGN_TOP_LEFT, 12, -4);

    lv_obj_t * target_row = lv_obj_create(card_target);
    lv_obj_set_size(target_row, 256, 56);
    lv_obj_set_style_bg_opa(target_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(target_row, 0, 0);
    lv_obj_set_style_pad_all(target_row, 0, 0);
    lv_obj_set_style_pad_column(target_row, 8, 0);
    lv_obj_align(target_row, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_set_layout(target_row, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(target_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(target_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(target_row, LV_OBJ_FLAG_SCROLLABLE);

    ta_target_angle = lv_textarea_create(target_row);
    lv_obj_set_size(ta_target_angle, 182, 48);
    lv_textarea_set_one_line(ta_target_angle, true);
    lv_textarea_set_max_length(ta_target_angle, 6);
    lv_textarea_set_accepted_chars(ta_target_angle, "-0123456789");
    // No placeholder example text - box starts populated with an actual "0",
    // not a placeholder hint, per spec.
    lv_textarea_set_text(ta_target_angle, "0");
    lv_obj_set_style_text_font(ta_target_angle, &lv_font_montserrat_18, 0);
    lv_obj_set_style_bg_color(ta_target_angle, lv_color_hex(0x1E1E1E), 0);
    lv_obj_set_style_border_color(ta_target_angle, lv_color_hex(0x333333), 0);
    lv_obj_set_style_text_color(ta_target_angle, lv_color_white(), 0);
    lv_obj_set_style_radius(ta_target_angle, 8, 0);
    // Custom keypad handles all input - tapping the box just opens/keeps it open.
    lv_obj_add_event_cb(ta_target_angle, target_ta_clicked_cb, LV_EVENT_CLICKED, NULL);

    btn_target_ok = lv_button_create(target_row);
    lv_obj_set_size(btn_target_ok, 56, 48);
    lv_obj_set_style_bg_color(btn_target_ok, lv_color_hex(0xFF1E27), 0);
    lv_obj_set_style_bg_color(btn_target_ok, lv_color_hex(0xCC1820), LV_STATE_PRESSED);
    lv_obj_set_style_radius(btn_target_ok, 8, 0);
    lv_obj_t * ok_lbl = lv_label_create(btn_target_ok);
    lv_label_set_text(ok_lbl, "OK");
    lv_obj_set_style_text_font(ok_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(ok_lbl, lv_color_white(), 0);
    lv_obj_center(ok_lbl);
    lv_obj_add_event_cb(btn_target_ok, target_ok_cb, LV_EVENT_CLICKED, NULL);

    // ---- INCREMENT / DECREMENT CARD ----
    // One unified card (same size/style as the two above) split into a
    // -5° / +5° zone with a thin divider, so all three left-hand cards match.
    lv_obj_t * card_incdec = lv_obj_create(col_left);
    lv_obj_set_size(card_incdec, 280, 120);
    lv_obj_set_style_bg_color(card_incdec, lv_color_hex(0x141414), 0);
    lv_obj_set_style_border_color(card_incdec, lv_color_hex(0x252525), 0);
    lv_obj_set_style_radius(card_incdec, 12, 0);
    lv_obj_set_style_pad_all(card_incdec, 0, 0);
    lv_obj_clear_flag(card_incdec, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_layout(card_incdec, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(card_incdec, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(card_incdec, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t * btn_dec = lv_button_create(card_incdec);
    lv_obj_set_size(btn_dec, 138, 118);
    lv_obj_set_style_bg_opa(btn_dec, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_color(btn_dec, lv_color_hex(0x2A1414), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(btn_dec, LV_OPA_COVER, LV_STATE_PRESSED);
    lv_obj_set_style_border_width(btn_dec, 0, 0);
    lv_obj_set_style_radius(btn_dec, 12, 0);
    lv_obj_t * lbl_dec_val = lv_label_create(btn_dec);
    lv_label_set_text(lbl_dec_val, "-5°");
    lv_obj_set_style_text_font(lbl_dec_val, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(lbl_dec_val, lv_color_hex(0xFF6666), 0);
    lv_obj_center(lbl_dec_val);
    lv_obj_add_event_cb(btn_dec, target_dec_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t * divider = lv_obj_create(card_incdec);
    lv_obj_set_size(divider, 1, 80);
    lv_obj_set_style_bg_color(divider, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_border_width(divider, 0, 0);
    lv_obj_set_style_radius(divider, 0, 0);
    lv_obj_clear_flag(divider, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t * btn_inc = lv_button_create(card_incdec);
    lv_obj_set_size(btn_inc, 138, 118);
    lv_obj_set_style_bg_opa(btn_inc, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_color(btn_inc, lv_color_hex(0x142A18), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(btn_inc, LV_OPA_COVER, LV_STATE_PRESSED);
    lv_obj_set_style_border_width(btn_inc, 0, 0);
    lv_obj_set_style_radius(btn_inc, 12, 0);
    lv_obj_t * lbl_inc_val = lv_label_create(btn_inc);
    lv_label_set_text(lbl_inc_val, "+5°");
    lv_obj_set_style_text_font(lbl_inc_val, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(lbl_inc_val, lv_color_hex(0x66FF88), 0);
    lv_obj_center(lbl_inc_val);
    lv_obj_add_event_cb(btn_inc, target_inc_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t * col_center = lv_obj_create(page_container);
    lv_obj_set_size(col_center, 380, 410);
    lv_obj_set_style_bg_color(col_center, lv_color_hex(0x0E0E0E), 0);
    lv_obj_set_style_border_color(col_center, lv_color_hex(0x222222), 0);
    lv_obj_set_style_radius(col_center, 16, 0);
    lv_obj_clear_flag(col_center, LV_OBJ_FLAG_SCROLLABLE);

    steering_wheel_arc = lv_arc_create(col_center);
    lv_obj_set_size(steering_wheel_arc, 320, 300);
    lv_obj_align(steering_wheel_arc, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_range(steering_wheel_arc, -90, 90);
    lv_arc_set_bg_angles(steering_wheel_arc, 180, 0); 
    lv_obj_remove_style(steering_wheel_arc, NULL, LV_PART_KNOB);
    lv_obj_set_style_arc_width(steering_wheel_arc, 8, LV_PART_MAIN);
    lv_obj_set_style_arc_color(steering_wheel_arc, lv_color_hex(0x1F1E1E), LV_PART_MAIN);
    lv_obj_set_style_arc_width(steering_wheel_arc, 8, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(steering_wheel_arc, lv_color_hex(0xFF1E27), LV_PART_INDICATOR); 

    img_steering_wheel = lv_image_create(col_center);
    lv_image_set_src(img_steering_wheel, &steering_wheel);
    lv_obj_set_size(img_steering_wheel, LV_SIZE_CONTENT, LV_SIZE_CONTENT); 
    lv_image_set_scale(img_steering_wheel, 105);   
    lv_obj_align(img_steering_wheel, LV_ALIGN_CENTER, 0, 0);
    
    lbl_st_wheel_angle = lv_label_create(col_center);
    lv_obj_set_style_text_font(lbl_st_wheel_angle, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(lbl_st_wheel_angle, lv_color_white(), 0);
    lv_obj_align(lbl_st_wheel_angle, LV_ALIGN_CENTER, 0, 0);

    // COLUMN 3: RIGHT HAND - COMPACT STATUS CARD + EMBEDDED KEYPAD SLOT
    // (Kept transparent/unbordered as a whole so it no longer reads as one
    // oversized box - only the status card itself is a bordered card, sized
    // to match the left-hand cards.)
    lv_obj_t * col_right = lv_obj_create(page_container);
    lv_obj_set_size(col_right, 290, 410);
    lv_obj_set_style_bg_opa(col_right, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(col_right, 0, 0);
    lv_obj_set_style_pad_all(col_right, 0, 0);
    lv_obj_set_style_pad_row(col_right, 10, 0);
    lv_obj_set_layout(col_right, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(col_right, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(col_right, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(col_right, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * card_status = lv_obj_create(col_right);
    lv_obj_set_size(card_status, 280, 120);
    lv_obj_set_style_bg_color(card_status, lv_color_hex(0x141414), 0);
    lv_obj_set_style_border_color(card_status, lv_color_hex(0x252525), 0);
    lv_obj_set_style_radius(card_status, 12, 0);
    lv_obj_clear_flag(card_status, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * lbl_r1_t = lv_label_create(card_status);
    lv_label_set_text(lbl_r1_t, "Steering Sensor Status");
    lv_obj_set_style_text_font(lbl_r1_t, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_r1_t, lv_color_hex(0x888888), 0);
    lv_obj_align(lbl_r1_t, LV_ALIGN_TOP_LEFT, 12, 10);

    lbl_sensor_status = lv_label_create(card_status);
    lv_label_set_text(lbl_sensor_status, LV_SYMBOL_OK " No Error");
    lv_obj_set_style_text_font(lbl_sensor_status, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(lbl_sensor_status, lv_color_hex(0x00FF66), 0);
    lv_obj_align(lbl_sensor_status, LV_ALIGN_BOTTOM_LEFT, 12, -10);

    // Reserved slot directly below the status card - this is where the
    // numeric keypad appears (built on demand) when the target-angle box is tapped.
    keypad_slot = lv_obj_create(col_right);
    lv_obj_set_size(keypad_slot, 280, 280);
    lv_obj_set_style_bg_opa(keypad_slot, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(keypad_slot, 0, 0);
    lv_obj_set_style_pad_all(keypad_slot, 0, 0);
    lv_obj_clear_flag(keypad_slot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_layout(keypad_slot, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(keypad_slot, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(keypad_slot, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
}

/************************************************
 * CUSTOM NUMERIC KEYPAD FOR TARGET ANGLE ENTRY
 * Built in-page (inside keypad_slot, right below the
 * Steering Sensor Status card) instead of a full-width
 * overlay, so the footer nav stays visible while typing.
 * Colors are dark-theme + red accent to match the rest
 * of the UI and stay readable on the HDMI touchscreen.
 ************************************************/
static void target_ta_clicked_cb(lv_event_t * e)
{
    LV_UNUSED(e);
    build_target_keypad();
}

static void keypad_digit_cb(lv_event_t * e)
{
    lv_obj_t * btn = lv_event_get_target(e);
    char c = (char)(intptr_t)lv_obj_get_user_data(btn);
    if (ta_target_angle == NULL) return;

    // Avoid leading-zero padding: if the box currently shows just "0"
    // (the default / cleared state), the first digit REPLACES it instead
    // of appending, so typing "1" shows "1" and not "01".
    const char * txt = lv_textarea_get_text(ta_target_angle);
    if (txt != NULL && strcmp(txt, "0") == 0) {
        char buf[2] = { c, '\0' };
        lv_textarea_set_text(ta_target_angle, buf);
    } else {
        lv_textarea_add_char(ta_target_angle, (uint32_t)c);
    }
}

static void keypad_backspace_cb(lv_event_t * e)
{
    LV_UNUSED(e);
    if (ta_target_angle == NULL) return;
    lv_textarea_delete_char(ta_target_angle);
    const char * txt = lv_textarea_get_text(ta_target_angle);
    if (txt == NULL || txt[0] == '\0') {
        lv_textarea_set_text(ta_target_angle, "0");
    }
}

static void keypad_sign_cb(lv_event_t * e)
{
    LV_UNUSED(e);
    if (ta_target_angle == NULL) return;
    const char * txt = lv_textarea_get_text(ta_target_angle);
    char buf[16];
    if (txt != NULL && txt[0] == '-') {
        snprintf(buf, sizeof(buf), "%s", txt + 1);
    } else {
        snprintf(buf, sizeof(buf), "-%s", (txt != NULL) ? txt : "");
    }
    lv_textarea_set_text(ta_target_angle, buf);
}

static void keypad_confirm_cb(lv_event_t * e)
{
    LV_UNUSED(e);
    confirm_and_send_target();
    if (target_keypad != NULL) {
        lv_obj_del(target_keypad);
        target_keypad = NULL;
    }
}

static void build_target_keypad(void)
{
    if (keypad_slot == NULL || target_keypad != NULL) return;

    target_keypad = lv_obj_create(keypad_slot);
    lv_obj_set_size(target_keypad, 270, 270);
    lv_obj_set_style_bg_opa(target_keypad, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(target_keypad, 0, 0);
    lv_obj_set_style_pad_all(target_keypad, 0, 0);
    lv_obj_set_style_pad_row(target_keypad, 6, 0);
    lv_obj_clear_flag(target_keypad, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_layout(target_keypad, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(target_keypad, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(target_keypad, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    static const char * rows[4][3] = {
        {"1",    "2", "3"},
        {"4",    "5", "6"},
        {"7",    "8", "9"},
        {"+/-",  "0", "DEL"},
    };

    for (int r = 0; r < 4; r++) {
        lv_obj_t * row = lv_obj_create(target_keypad);
        lv_obj_set_size(row, 270, 48);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_style_pad_column(row, 6, 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_layout(row, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        for (int c = 0; c < 3; c++) {
            const char * key_txt = rows[r][c];

            lv_obj_t * key = lv_button_create(row);
            lv_obj_set_size(key, 84, 44);
            lv_obj_set_style_radius(key, 8, 0);
            lv_obj_set_style_border_width(key, 1, 0);
            lv_obj_set_style_border_color(key, lv_color_hex(0x333333), 0);

            lv_obj_t * lbl = lv_label_create(key);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
            lv_obj_center(lbl);

            if (strcmp(key_txt, "DEL") == 0) {
                lv_obj_set_style_bg_color(key, lv_color_hex(0x241414), 0);
                lv_obj_set_style_bg_color(key, lv_color_hex(0x3A1818), LV_STATE_PRESSED);
                lv_label_set_text(lbl, LV_SYMBOL_BACKSPACE);
                lv_obj_set_style_text_color(lbl, lv_color_hex(0xFF6666), 0);
                lv_obj_add_event_cb(key, keypad_backspace_cb, LV_EVENT_CLICKED, NULL);
            } else if (strcmp(key_txt, "+/-") == 0) {
                lv_obj_set_style_bg_color(key, lv_color_hex(0x1E1E1E), 0);
                lv_obj_set_style_bg_color(key, lv_color_hex(0x333333), LV_STATE_PRESSED);
                lv_label_set_text(lbl, "+/-");
                lv_obj_set_style_text_color(lbl, lv_color_hex(0xDDDDDD), 0);
                lv_obj_add_event_cb(key, keypad_sign_cb, LV_EVENT_CLICKED, NULL);
            } else {
                lv_obj_set_style_bg_color(key, lv_color_hex(0x1E1E1E), 0);
                lv_obj_set_style_bg_color(key, lv_color_hex(0x333333), LV_STATE_PRESSED);
                lv_label_set_text(lbl, key_txt);
                lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
                lv_obj_set_user_data(key, (void*)(intptr_t)key_txt[0]);
                lv_obj_add_event_cb(key, keypad_digit_cb, LV_EVENT_CLICKED, NULL);
            }
        }
    }

    lv_obj_t * btn_confirm = lv_button_create(target_keypad);
    lv_obj_set_size(btn_confirm, 270, 46);
    lv_obj_set_style_bg_color(btn_confirm, lv_color_hex(0xFF1E27), 0);
    lv_obj_set_style_bg_color(btn_confirm, lv_color_hex(0xCC1820), LV_STATE_PRESSED);
    lv_obj_set_style_radius(btn_confirm, 8, 0);
    lv_obj_t * lbl_confirm = lv_label_create(btn_confirm);
    lv_label_set_text(lbl_confirm, LV_SYMBOL_OK " Confirm");
    lv_obj_set_style_text_font(lbl_confirm, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(lbl_confirm, lv_color_white(), 0);
    lv_obj_center(lbl_confirm);
    lv_obj_add_event_cb(btn_confirm, keypad_confirm_cb, LV_EVENT_CLICKED, NULL);
}

/************************************************
 * TARGET ANGLE: CLAMP + APPLY TO INPUT BOX
 ************************************************/
static float target_angle_read_current(void)
{
    if (ta_target_angle == NULL) return 0.0f;
    const char * txt = lv_textarea_get_text(ta_target_angle);
    if (txt == NULL || txt[0] == '\0') return 0.0f;
    return strtof(txt, NULL);
}

static void target_angle_write(float val)
{
    if (val > STEER_ANGLE_MAX) val = STEER_ANGLE_MAX;
    if (val < STEER_ANGLE_MIN) val = STEER_ANGLE_MIN;
    char buf[16];
    snprintf(buf, sizeof(buf), "%.0f", val);
    if (ta_target_angle != NULL) lv_textarea_set_text(ta_target_angle, buf);
}

static void target_inc_cb(lv_event_t * e)
{
    float val = target_angle_read_current() + 5.0f;
    target_angle_write(val);
    pulse_widget(ta_target_angle);
    pulse_widget(lv_event_get_target(e));
}

static void target_dec_cb(lv_event_t * e)
{
    float val = target_angle_read_current() - 5.0f;
    target_angle_write(val);
    pulse_widget(ta_target_angle);
    pulse_widget(lv_event_get_target(e));
}

/************************************************
 * TARGET ANGLE: CLAMP, CONVERT TO RADIANS, SEND
 * OVER CAN (ID 0x103) TO THE STEERING TEENSY
 ************************************************/
static void confirm_and_send_target(void)
{
    float val = target_angle_read_current();
    if (val > STEER_ANGLE_MAX) val = STEER_ANGLE_MAX;
    if (val < STEER_ANGLE_MIN) val = STEER_ANGLE_MIN;
    target_angle_write(val); // re-format the display to the clamped value
    can_send_target_angle(val);
    pulse_widget(ta_target_angle);
}

static void target_ok_cb(lv_event_t * e)
{
    LV_UNUSED(e);
    confirm_and_send_target();
    pulse_widget(btn_target_ok);
    if (target_keypad != NULL) {
        lv_obj_del(target_keypad);
        target_keypad = NULL;
    }
}

/************************************************
 * VIEW PANELS: PAGE 5 - DRIVE MODE NATIVE SCALE
 ************************************************/
static void draw_mode_page(void)
{
    build_footer_menu(); 
    
    lv_obj_set_layout(page_container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(page_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(page_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(page_container, 5, 0);
    lv_obj_set_style_pad_row(page_container, 8, 0);

    // SUB-NAV SELECTOR TOGGLE BAR
    lv_obj_t * sub_nav_bar = lv_obj_create(page_container);
    lv_obj_set_size(sub_nav_bar, 980, 55);
    lv_obj_set_style_bg_color(sub_nav_bar, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_color(sub_nav_bar, lv_color_hex(0x222222), 0);
    lv_obj_set_style_radius(sub_nav_bar, 8, 0);
    lv_obj_set_layout(sub_nav_bar, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(sub_nav_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(sub_nav_bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(sub_nav_bar, 4, 0);
    lv_obj_set_style_pad_column(sub_nav_bar, 8, 0);
    lv_obj_clear_flag(sub_nav_bar, LV_OBJ_FLAG_SCROLLABLE);

    // Tab order: 0=MODE OPTIONS, 1=THROTTLE, 2=BRAKE
    const char * sub_tabs[] = {"MODE OPTIONS", "THROTTLE", "BRAKE"};
    for(int i = 0; i < 3; i++) {
        lv_obj_t * tab = lv_button_create(sub_nav_bar);
        lv_obj_set_size(tab, 180, 42);
        lv_obj_set_user_data(tab, (void*)(intptr_t)i);
        lv_obj_set_style_radius(tab, 8, 0);
        
        if(i == current_sub_mode_tab) {
            lv_obj_set_style_bg_color(tab, lv_color_hex(0xFF1E27), 0); 
        } else {
            lv_obj_set_style_bg_color(tab, lv_color_hex(0x1C1C1C), 0);
        }

        lv_obj_t * lbl = lv_label_create(tab);
        lv_label_set_text(lbl, sub_tabs[i]);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);

        lv_obj_add_event_cb(tab, sub_mode_tab_click_callback, LV_EVENT_CLICKED, NULL);
    }

    // MAIN SUB-PAGE DECK VIEWPORTS CONTAINER
    lv_obj_t * content_panel = lv_obj_create(page_container);
    lv_obj_set_size(content_panel, 980, 360); 
    lv_obj_set_style_bg_opa(content_panel, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(content_panel, 0, 0);
    lv_obj_set_style_pad_all(content_panel, 0, 0);
    lv_obj_clear_flag(content_panel, LV_OBJ_FLAG_SCROLLABLE);

    // Tab 0 = MODE OPTIONS
    if(current_sub_mode_tab == 0) {
        lv_obj_set_layout(content_panel, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(content_panel, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(content_panel, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_all(content_panel, 10, 0);

        const char * m_titles[]  = {"STANDBY", "AEB",  "LKA",  "ENDURANCE"};
        const char * m_subtitles[] = {
            "System in Standby State",
            "Autonomous Emergency Braking",
            "Lane Keep Assist",
            "Combination of AEB and LKA"
        };
        uint32_t m_colors[] = {0xFFB900, 0xFF1E27, 0x00F5FF, 0x00FF66};

        for(int j = 0; j < 4; j++) {
            lv_obj_t * grid_card = lv_obj_create(content_panel);
            lv_obj_set_size(grid_card, 220, 320);
            lv_obj_set_user_data(grid_card, (void*)(intptr_t)j);
            lv_obj_add_flag(grid_card, LV_OBJ_FLAG_CLICKABLE);

            lv_obj_set_style_bg_color(grid_card, lv_color_hex(0x141414), 0);
            lv_obj_set_style_border_color(grid_card,
                mode_active[j] ? lv_color_hex(m_colors[j]) : lv_color_hex(0x222222), 0);
            lv_obj_set_style_border_width(grid_card, mode_active[j] ? 3 : 1, 0);
            lv_obj_set_style_radius(grid_card, 16, 0);

            lv_obj_set_layout(grid_card, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(grid_card, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_flex_align(grid_card, LV_FLEX_ALIGN_SPACE_BETWEEN,
                                  LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
            lv_obj_set_style_pad_all(grid_card, 15, 0);
            lv_obj_clear_flag(grid_card, LV_OBJ_FLAG_SCROLLABLE);

            // Title (e.g. "AEB")
            lv_obj_t * title = lv_label_create(grid_card);
            lv_label_set_text(title, m_titles[j]);
            lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
            lv_obj_set_style_text_color(title, lv_color_white(), 0);

            // Subtitle description
            lv_obj_t * desc = lv_label_create(grid_card);
            lv_label_set_text(desc, m_subtitles[j]);
            lv_obj_set_width(desc, 185);   // try 180–190
            lv_obj_set_style_text_font(desc, &lv_font_montserrat_12, 0);
            lv_obj_set_style_text_color(desc, lv_color_hex(0x888888), 0);
            lv_obj_set_style_text_line_space(desc, 4, 0);

            // Active/Select status at bottom
            lv_obj_t * status = lv_label_create(grid_card);
            lv_label_set_text(status, mode_active[j] ? "ACTIVE" : "SELECT");
            lv_obj_set_style_text_font(status, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_color(status,
                mode_active[j] ? lv_color_hex(m_colors[j]) : lv_color_hex(0x444444), 0);

            lv_obj_add_event_cb(grid_card, preset_grid_click_callback, LV_EVENT_CLICKED, NULL);
        }
    }
    // Tab 1 = THROTTLE
    else if(current_sub_mode_tab == 1) {
        lv_obj_t * throttle_deck = lv_obj_create(content_panel);
        lv_obj_set_size(throttle_deck, 980, 360); 
        lv_obj_set_style_bg_opa(throttle_deck, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(throttle_deck, 0, 0);
        lv_obj_clear_flag(throttle_deck, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * center_img_placeholder = lv_obj_create(throttle_deck);
        lv_obj_set_size(center_img_placeholder, 400, 340); 
        lv_obj_set_style_bg_opa(center_img_placeholder, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(center_img_placeholder, 0, 0);
        lv_obj_clear_flag(center_img_placeholder, LV_OBJ_FLAG_SCROLLABLE); 
        lv_obj_align(center_img_placeholder, LV_ALIGN_CENTER, 0, 0);
        
        lv_obj_t * img_throttle = lv_image_create(center_img_placeholder);
        lv_image_set_src(img_throttle, &throttle_pedal);
        lv_image_set_scale(img_throttle, 65); 
        lv_obj_align(img_throttle, LV_ALIGN_CENTER, 0, 0);

        // Top Left: Throttle %
        lv_obj_t * tl_card = lv_obj_create(throttle_deck);
        lv_obj_set_size(tl_card, 260, 165); 
        lv_obj_set_style_bg_color(tl_card, lv_color_hex(0x141414), 0);
        lv_obj_set_style_border_color(tl_card, lv_color_hex(0x333333), 0);
        lv_obj_set_style_radius(tl_card, 16, 0);
        lv_obj_set_pos(tl_card, 10, 10); 
        lv_obj_clear_flag(tl_card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * tl_title = lv_label_create(tl_card);
        lv_label_set_text(tl_title, "Throttle %");
        lv_obj_set_style_text_font(tl_title, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(tl_title, lv_color_hex(0xAAAAAA), 0);
        lv_obj_align(tl_title, LV_ALIGN_TOP_MID, 0, 10);

        throttle_live_arc = lv_arc_create(tl_card);
        lv_obj_set_size(throttle_live_arc, 100, 100);
        lv_arc_set_rotation(throttle_live_arc, 270);
        lv_arc_set_bg_angles(throttle_live_arc, 0, 360);
        lv_obj_remove_style(throttle_live_arc, NULL, LV_PART_KNOB);
        lv_obj_set_style_arc_width(throttle_live_arc, 8, LV_PART_MAIN);
        lv_obj_set_style_arc_color(throttle_live_arc, lv_color_hex(0x222222), LV_PART_MAIN);
        lv_obj_set_style_arc_width(throttle_live_arc, 8, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(throttle_live_arc, lv_color_hex(0xFF1E27), LV_PART_INDICATOR); 
        lv_obj_align(throttle_live_arc, LV_ALIGN_TOP_MID, 0, 35);

        lbl_throttle_txt_metrics = lv_label_create(throttle_live_arc);
        lv_label_set_text(lbl_throttle_txt_metrics, "0%");
        lv_obj_set_style_text_font(lbl_throttle_txt_metrics, &lv_font_montserrat_22, 0);
        lv_obj_set_style_text_color(lbl_throttle_txt_metrics, lv_color_hex(0xFF1E27), 0);
        lv_obj_align(lbl_throttle_txt_metrics, LV_ALIGN_CENTER, 0, 0);

        // Bottom Left: Controller Temperature (was "Sensor Voltage")
        lv_obj_t * bl_card = lv_obj_create(throttle_deck);
        lv_obj_set_size(bl_card, 260, 165); 
        lv_obj_set_style_bg_color(bl_card, lv_color_hex(0x141414), 0);
        lv_obj_set_style_border_color(bl_card, lv_color_hex(0x333333), 0);
        lv_obj_set_style_radius(bl_card, 16, 0);
        lv_obj_set_pos(bl_card, 10, 185); 
        lv_obj_clear_flag(bl_card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * bl_title = lv_label_create(bl_card);
        lv_label_set_text(bl_title, "Controller Temp");
        lv_obj_set_style_text_font(bl_title, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(bl_title, lv_color_hex(0xAAAAAA), 0);
        lv_obj_align(bl_title, LV_ALIGN_TOP_MID, 0, 15);

        lbl_controller_temp = lv_label_create(bl_card);
        lv_label_set_text(lbl_controller_temp, "0\xC2\xB0" "C");
        lv_obj_set_style_text_font(lbl_controller_temp, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(lbl_controller_temp, lv_color_white(), 0);
        lv_obj_align(lbl_controller_temp, LV_ALIGN_CENTER, 0, 15);

        // Top Right: Motor RPM (unchanged)
        lv_obj_t * tr_card = lv_obj_create(throttle_deck);
        lv_obj_set_size(tr_card, 260, 165);
        lv_obj_set_style_bg_color(tr_card, lv_color_hex(0x141414), 0);
        lv_obj_set_style_border_color(tr_card, lv_color_hex(0x333333), 0);
        lv_obj_set_style_radius(tr_card, 16, 0);
        lv_obj_set_pos(tr_card, 710, 10); 
        lv_obj_clear_flag(tr_card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * tr_title = lv_label_create(tr_card);
        lv_label_set_text(tr_title, "Motor RPM");
        lv_obj_set_style_text_font(tr_title, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(tr_title, lv_color_hex(0xAAAAAA), 0);
        lv_obj_align(tr_title, LV_ALIGN_TOP_MID, 0, 15);

        lbl_st_motor_curr = lv_label_create(tr_card); 
        lv_label_set_text(lbl_st_motor_curr, "0");
        lv_obj_set_style_text_font(lbl_st_motor_curr, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(lbl_st_motor_curr, lv_color_white(), 0);
        lv_obj_align(lbl_st_motor_curr, LV_ALIGN_CENTER, 0, 15);

        // Bottom Right: Motor Temperature (was "COMMS Link")
        lv_obj_t * br_card = lv_obj_create(throttle_deck);
        lv_obj_set_size(br_card, 260, 165);
        lv_obj_set_style_bg_color(br_card, lv_color_hex(0x141414), 0);
        lv_obj_set_style_border_color(br_card, lv_color_hex(0x333333), 0);
        lv_obj_set_style_radius(br_card, 16, 0);
        lv_obj_set_pos(br_card, 710, 185); 
        lv_obj_clear_flag(br_card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * br_title = lv_label_create(br_card);
        lv_label_set_text(br_title, "Motor Temp");
        lv_obj_set_style_text_font(br_title, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(br_title, lv_color_hex(0xAAAAAA), 0);
        lv_obj_align(br_title, LV_ALIGN_TOP_MID, 0, 15);

        lbl_st_rate = lv_label_create(br_card); 
        lv_label_set_text(lbl_st_rate, "0\xC2\xB0" "C");
        lv_obj_set_style_text_font(lbl_st_rate, &lv_font_montserrat_22, 0);
        lv_obj_set_style_text_color(lbl_st_rate, lv_color_white(), 0);
        lv_obj_align(lbl_st_rate, LV_ALIGN_CENTER, 0, 15);
    }
    // Tab 2 = BRAKE
    else if(current_sub_mode_tab == 2) {
        lv_obj_t * brake_deck = lv_obj_create(content_panel);
        lv_obj_set_size(brake_deck, 980, 360); 
        lv_obj_set_style_bg_opa(brake_deck, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(brake_deck, 0, 0);
        lv_obj_clear_flag(brake_deck, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * center_img_placeholder = lv_obj_create(brake_deck);
        lv_obj_set_size(center_img_placeholder, 400, 340); 
        lv_obj_set_style_bg_opa(center_img_placeholder, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(center_img_placeholder, 0, 0);
        lv_obj_clear_flag(center_img_placeholder, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_align(center_img_placeholder, LV_ALIGN_CENTER, 0, 0);
        
        lv_obj_t * img_brake = lv_image_create(center_img_placeholder);
        lv_image_set_src(img_brake, &brake_rotor);
        lv_image_set_scale(img_brake, 65); 
        lv_obj_align(img_brake, LV_ALIGN_CENTER, 0, 0);

        // Top Left Card: Brake %
        lv_obj_t * tl_card = lv_obj_create(brake_deck);
        lv_obj_set_size(tl_card, 260, 165);
        lv_obj_set_style_bg_color(tl_card, lv_color_hex(0x141414), 0);
        lv_obj_set_style_border_color(tl_card, lv_color_hex(0x333333), 0);
        lv_obj_set_style_radius(tl_card, 16, 0);
        lv_obj_set_pos(tl_card, 10, 10);
        lv_obj_clear_flag(tl_card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * tl_title = lv_label_create(tl_card);
        lv_label_set_text(tl_title, "Brake %");
        lv_obj_set_style_text_font(tl_title, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(tl_title, lv_color_hex(0xAAAAAA), 0);
        lv_obj_align(tl_title, LV_ALIGN_TOP_MID, 0, 10);

        brake_live_arc = lv_arc_create(tl_card);
        lv_obj_set_size(brake_live_arc, 100, 100);
        lv_arc_set_rotation(brake_live_arc, 270);
        lv_arc_set_bg_angles(brake_live_arc, 0, 360);
        lv_obj_remove_style(brake_live_arc, NULL, LV_PART_KNOB);
        lv_obj_set_style_arc_width(brake_live_arc, 8, LV_PART_MAIN);
        lv_obj_set_style_arc_color(brake_live_arc, lv_color_hex(0x222222), LV_PART_MAIN);
        lv_obj_set_style_arc_width(brake_live_arc, 8, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(brake_live_arc, lv_color_hex(0xFF1E27), LV_PART_INDICATOR); 
        lv_obj_align(brake_live_arc, LV_ALIGN_TOP_MID, 0, 45);

        lv_obj_t * arc_val = lv_label_create(brake_live_arc);
        lv_label_set_text(arc_val, "0%");
        lv_obj_set_style_text_font(arc_val, &lv_font_montserrat_22, 0);
        lv_obj_set_style_text_color(arc_val, lv_color_hex(0xFF1E27), 0);
        lv_obj_align(arc_val, LV_ALIGN_CENTER, 0, 0);

        // Bottom Left Card: Brake Command
        lv_obj_t * bl_card = lv_obj_create(brake_deck);
        lv_obj_set_size(bl_card, 260, 165);
        lv_obj_set_style_bg_color(bl_card, lv_color_hex(0x141414), 0);
        lv_obj_set_style_border_color(bl_card, lv_color_hex(0x333333), 0);
        lv_obj_set_style_radius(bl_card, 16, 0);
        lv_obj_set_pos(bl_card, 10, 185);
        lv_obj_clear_flag(bl_card, LV_OBJ_FLAG_SCROLLABLE);

        lbl_brake_cmd_pct = lv_label_create(bl_card);
        lv_label_set_text(lbl_brake_cmd_pct, "0%");
        lv_obj_set_style_text_font(lbl_brake_cmd_pct, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(lbl_brake_cmd_pct, lv_color_white(), 0);
        lv_obj_align(lbl_brake_cmd_pct, LV_ALIGN_TOP_MID, 0, 20);

        brake_cmd_bar = lv_bar_create(bl_card);
        lv_obj_set_size(brake_cmd_bar, 200, 10);
        lv_obj_align(brake_cmd_bar, LV_ALIGN_TOP_MID, 0, 75); 
        lv_obj_set_style_bg_color(brake_cmd_bar, lv_color_hex(0x222222), LV_PART_MAIN);
        lv_obj_set_style_bg_color(brake_cmd_bar, lv_color_hex(0xFF1E27), LV_PART_INDICATOR);

        lbl_brake_pressure_metrics = lv_label_create(bl_card);
        lv_label_set_text(lbl_brake_pressure_metrics, "0.0 MPa");
        lv_obj_set_style_text_font(lbl_brake_pressure_metrics, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(lbl_brake_pressure_metrics, lv_color_hex(0x888888), 0);
        lv_obj_align(lbl_brake_pressure_metrics, LV_ALIGN_TOP_MID, 0, 110);

        // Top Right Card: Actuator Position
        lv_obj_t * tr_card = lv_obj_create(brake_deck);
        lv_obj_set_size(tr_card, 260, 165);
        lv_obj_set_style_bg_color(tr_card, lv_color_hex(0x141414), 0);
        lv_obj_set_style_border_color(tr_card, lv_color_hex(0x333333), 0);
        lv_obj_set_style_radius(tr_card, 16, 0);
        lv_obj_set_pos(tr_card, 710, 10);
        lv_obj_clear_flag(tr_card, LV_OBJ_FLAG_SCROLLABLE);

        lbl_brake_actuator = lv_label_create(tr_card);
        lv_label_set_text(lbl_brake_actuator, "14.5mm");
        lv_obj_set_style_text_font(lbl_brake_actuator, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(lbl_brake_actuator, lv_color_white(), 0);
        lv_obj_align(lbl_brake_actuator, LV_ALIGN_CENTER, 0, 0);

        // Bottom Right Card: System Status
        lv_obj_t * br_card = lv_obj_create(brake_deck);
        lv_obj_set_size(br_card, 260, 165);
        lv_obj_set_style_bg_color(br_card, lv_color_hex(0x141414), 0);
        lv_obj_set_style_border_color(br_card, lv_color_hex(0x333333), 0);
        lv_obj_set_style_radius(br_card, 16, 0);
        lv_obj_set_pos(br_card, 710, 185); 
        lv_obj_clear_flag(br_card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * br_abs_title = lv_label_create(br_card);
        lv_label_set_text(br_abs_title, "ABS Control");
        lv_obj_set_style_text_font(br_abs_title, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(br_abs_title, lv_color_hex(0x888888), 0);
        lv_obj_align(br_abs_title, LV_ALIGN_TOP_MID, 0, 25);

        lv_obj_t * br_abs_val = lv_label_create(br_card);
        lv_label_set_text(br_abs_val, LV_SYMBOL_OK " ACTIVE");
        lv_obj_set_style_text_font(br_abs_val, &lv_font_montserrat_22, 0);
        lv_obj_set_style_text_color(br_abs_val, lv_color_white(), 0);
        lv_obj_align(br_abs_val, LV_ALIGN_TOP_MID, 0, 70);
    }
}

/************************************************
 * VIEW PANELS: PAGE 6 - DIAGNOSTIC FAULTS
 ************************************************/
static void draw_errors_page(void)
{
    build_footer_menu(); 
    
    lv_obj_set_layout(page_container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(page_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(page_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(page_container, 20, 0);
    lv_obj_set_style_pad_row(page_container, 10, 0);

    fault_card_obj = lv_obj_create(page_container);
    lv_obj_set_size(fault_card_obj, 950, 100);
    lv_obj_set_style_bg_color(fault_card_obj, lv_color_hex(0x141414), 0); 
    lv_obj_set_style_border_color(fault_card_obj, lv_color_hex(0xFF3333), 0); // Default to red
    lv_obj_set_style_radius(fault_card_obj, 10, 0);

    lbl_err_title = lv_label_create(fault_card_obj);
    lv_label_set_text(lbl_err_title, "CRITICAL: ERR04 - AS5600 STEERING ENCODER DISCONNECT");
    lv_obj_set_style_text_font(lbl_err_title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(lbl_err_title, lv_color_hex(0xFF3333), 0);
    lv_obj_align(lbl_err_title, LV_ALIGN_TOP_LEFT, 12, 10);

    lbl_err_desc = lv_label_create(fault_card_obj);
    lv_label_set_text(lbl_err_desc, "I2C Comm Failure. Check AS5600 wiring to Teensy immediately.");
    lv_obj_set_style_text_font(lbl_err_desc, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_err_desc, lv_color_hex(0xDDDDDD), 0);
    lv_obj_align(lbl_err_desc, LV_ALIGN_BOTTOM_LEFT, 12, -10);
}

/************************************************
 * VIEW PANELS: PAGE 7 - SYSTEM TUNING SETTINGS
 ************************************************/
static void draw_settings_page(void)
{
    build_footer_menu(); 
    lv_obj_set_layout(page_container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(page_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(page_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_top(page_container, 15, 0);
    lv_obj_set_style_pad_row(page_container, 12, 0);
    
    lv_obj_t *lbl = lv_label_create(page_container);
    lv_label_set_text(lbl, "SYSTEM CONFIGURATION\nBAJA CONTROLLER CONNECTED");
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t * can_hdr = lv_label_create(page_container);
    lv_label_set_text(can_hdr, "CAN BUS ID REFERENCE");
    lv_obj_set_style_text_font(can_hdr, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(can_hdr, lv_color_hex(0x888888), 0);

    // ---- CAN ID <-> PURPOSE REFERENCE GRID ----
    typedef struct { const char * purpose; const char * can_id; bool pending; } can_ref_t;
    static const can_ref_t can_refs[] = {
        { "Steering Target Command",      "0x103",      false },
        { "Steering Sensor Status",       "0x063",      false },
        { "Steering Angle Feedback",      "0x203",      false },
        { "Brake Feedback",               "0x202",      false },
        { "Battery Voltage\nMotor RPM",  "0x180117EF", false },
        { "Controller Temperature\nMotor Temperature\nGear Status","0x180217EF", false },
        { "Lateral Autonomy",       "0x301",      false },
        { "Longitudinal Autonomy", "0x302",      false },
        { "AEB Node\nLKA Node\nEndurance Node",   "0x400",      false },
       // { "LKA - Lane Keep Assist",       "0x400",      false },
       // { "Endurance Mode",               "0x400",      false },
    };
    const int can_ref_count = (int)(sizeof(can_refs) / sizeof(can_refs[0]));

    lv_obj_t * can_grid = lv_obj_create(page_container);
    lv_obj_set_size(can_grid, 990, 350);
    lv_obj_set_style_bg_opa(can_grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(can_grid, 0, 0);
    lv_obj_set_style_pad_all(can_grid, 0, 0);
    lv_obj_set_style_pad_row(can_grid, 10, 0);
    lv_obj_set_style_pad_column(can_grid, 10, 0);
    lv_obj_set_layout(can_grid, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(can_grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(can_grid, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);

    for (int i = 0; i < can_ref_count; i++) {
        lv_obj_t * card = lv_obj_create(can_grid);
        lv_obj_set_size(card, 315, 78);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x141414), 0);
        lv_obj_set_style_border_color(card, lv_color_hex(0x252525), 0);
        lv_obj_set_style_radius(card, 10, 0);
        lv_obj_set_style_pad_all(card, 10, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * purpose_lbl = lv_label_create(card);
        lv_label_set_text(purpose_lbl, can_refs[i].purpose);
        lv_obj_set_style_text_font(purpose_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(purpose_lbl, lv_color_hex(0x999999), 0);
        lv_label_set_long_mode(purpose_lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(purpose_lbl, 190);
        lv_obj_align(purpose_lbl, LV_ALIGN_TOP_LEFT, 0, 0);

        lv_obj_t * id_pill = lv_obj_create(card);
        lv_obj_set_size(id_pill, 92, 30);
        lv_obj_set_style_radius(id_pill, 15, 0);
        lv_obj_set_style_border_width(id_pill, 0, 0);
        lv_obj_set_style_pad_all(id_pill, 0, 0);
        lv_obj_clear_flag(id_pill, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_align(id_pill, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
        lv_obj_set_style_bg_color(id_pill, can_refs[i].pending ? lv_color_hex(0x262626) : lv_color_hex(0x2A1416), 0);

        lv_obj_t * id_lbl = lv_label_create(id_pill);
        lv_label_set_text(id_lbl, can_refs[i].can_id);
        lv_obj_set_style_text_font(id_lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(id_lbl, can_refs[i].pending ? lv_color_hex(0x777777) : lv_color_hex(0xFF5A63), 0);
        lv_obj_center(id_lbl);
    }
}

/************************************************
 * REAL-TIME RUNTIME DATA BACKGROUND TIMERS
 ************************************************/
static void application_background_loop(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    char buf[512]; 

    system_clock_seconds++;
    
    time_t rawtime;
    struct tm * timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);

    int hour = timeinfo->tm_hour % 12;
    if (hour == 0) hour = 12;
    const char * ampm = (timeinfo->tm_hour >= 12) ? "PM" : "AM";

    snprintf(buf, sizeof(buf), "%02d:%02d:%02d %s", hour, timeinfo->tm_min, timeinfo->tm_sec, ampm);
    if (header_time_label != NULL) lv_label_set_text(header_time_label, buf);

#ifndef _MSC_VER
    pthread_mutex_lock(&data_mutex);
#endif
    if (dot_auto) lv_obj_set_style_bg_color(dot_auto, live_car_data.ind_auto > 0 ? lv_color_hex(0x00FF66) : lv_color_hex(0x444444), 0);
    if (dot_lat) lv_obj_set_style_bg_color(dot_lat, live_car_data.ind_lat > 0 ? lv_color_hex(0x00FF66) : lv_color_hex(0x444444), 0);
    if (dot_long) lv_obj_set_style_bg_color(dot_long, live_car_data.ind_long > 0 ? lv_color_hex(0x00FF66) : lv_color_hex(0x444444), 0);

    if (dot_radar) lv_obj_set_style_bg_color(dot_radar,( mode_active[1] || mode_active[3]) ? lv_color_hex(0x00FF66) : lv_color_hex(0x444444), 0);
    if (dot_camera) lv_obj_set_style_bg_color(dot_camera, (mode_active[2] || mode_active[3]) ? lv_color_hex(0x00FF66) : lv_color_hex(0x444444), 0);
    if (dot_imu) lv_obj_set_style_bg_color(dot_imu, mode_active[3] ? lv_color_hex(0x00FF66) : lv_color_hex(0x444444), 0);

    if (live_car_data.ind_auto > 0) live_car_data.ind_auto--;

    int current_speed = live_car_data.vehicle_speed_kmh;
    float current_steer = live_car_data.steer_angle_deg;
    float current_throttle = live_car_data.throttle_pct; // derived from vehicle speed only
    float current_brake = live_car_data.brake_pct;
    int current_m_amps = live_car_data.motor_rpm; 
    int current_wheel_rpm = live_car_data.wheel_rpm; 
    int current_boot = live_car_data.boot_progress;
    int steering_err = live_car_data.steering_status; // 0 = OK, 1 = Error
    float current_battery_voltage = live_car_data.battery_voltage;
    int current_controller_temp = live_car_data.controller_temp;
    int current_motor_temp = live_car_data.motor_temp;
    int current_gear_raw = live_car_data.gear_raw;
#ifndef _MSC_VER
    pthread_mutex_unlock(&data_mutex);
#endif

    if (current_page == PAGE_HOME) {
        if (current_boot < 100) {
            #ifndef _MSC_VER
            pthread_mutex_lock(&data_mutex);
            #endif
            live_car_data.boot_progress += 2;
            current_boot = live_car_data.boot_progress;
            #ifndef _MSC_VER
            pthread_mutex_unlock(&data_mutex);
            #endif
            
            if (loading_bar != NULL) lv_bar_set_value(loading_bar, current_boot, LV_ANIM_OFF);
            snprintf(buf, sizeof(buf), " %d%%", current_boot);
            if (lbl_boot_pct != NULL) lv_label_set_text(lbl_boot_pct, buf);
        } else {
            if (status_label != NULL) {
                lv_label_set_text(status_label, "READY TO DRIVE");
                lv_obj_set_style_text_color(status_label, lv_color_hex(0x00FF66), 0);
            }
            if (lbl_boot_pct != NULL) {
                lv_label_set_text(lbl_boot_pct, " 100%");
                lv_obj_set_style_text_color(lbl_boot_pct, lv_color_hex(0x00FF66), 0);
            }
        }
    } 
    else if (current_page == PAGE_VEHICLE) {
        if (lbl_velocity != NULL) {
            snprintf(buf, sizeof(buf), "%d km/h", current_speed);
            lv_label_set_text(lbl_velocity, buf);
        }
        if (lbl_steer_angle != NULL) {
            if(steering_err != 0) {
                lv_label_set_text(lbl_steer_angle, "ERR");
                lv_obj_set_style_text_color(lbl_steer_angle, lv_color_hex(0xFF3333), 0);
            } else {
                snprintf(buf, sizeof(buf), "%+.1f°", current_steer);
                lv_label_set_text(lbl_steer_angle, buf);
                lv_obj_set_style_text_color(lbl_steer_angle, lv_color_white(), 0);
            }
        }
        // Throttle Percentage (single source of truth: derived from Vehicle Speed)
        if (lbl_throttle_cmd != NULL) {
            snprintf(buf, sizeof(buf), "%d%%", (int)current_throttle);
            lv_label_set_text(lbl_throttle_cmd, buf);
        }
        // Battery Voltage (decoded Bus Voltage from 0x180117EF)
        if (lbl_brake_cmd != NULL) {
            snprintf(buf, sizeof(buf), "%.1f V", current_battery_voltage);
            lv_label_set_text(lbl_brake_cmd, buf);
        }
        // Motor RPM
        if (lbl_g_force != NULL) {
            snprintf(buf, sizeof(buf), "%d RPM", current_m_amps);
            lv_label_set_text(lbl_g_force, buf);
        }
        // Gear
        if (lbl_brake_pressure != NULL) {
            lv_label_set_text(lbl_brake_pressure, gear_to_string(current_gear_raw));
        }
        
        if (lbl_wheel_fl != NULL) { snprintf(buf, sizeof(buf), "FL: %d RPM", current_wheel_rpm); lv_label_set_text(lbl_wheel_fl, buf); }
        if (lbl_wheel_fr != NULL) { snprintf(buf, sizeof(buf), "FR: %d RPM", current_wheel_rpm); lv_label_set_text(lbl_wheel_fr, buf); }
        if (lbl_wheel_rl != NULL) { snprintf(buf, sizeof(buf), "RL: %d RPM", current_wheel_rpm); lv_label_set_text(lbl_wheel_rl, buf); }
        if (lbl_wheel_rr != NULL) { snprintf(buf, sizeof(buf), "RR: %d RPM", current_wheel_rpm); lv_label_set_text(lbl_wheel_rr, buf); }
    }
    else if (current_page == PAGE_MODE) {
        // Tab 1 = THROTTLE
        if(current_sub_mode_tab == 1 && throttle_live_arc != NULL) {
            lv_arc_set_value(throttle_live_arc, (int)current_throttle);
            lv_obj_t * center_lbl = lv_obj_get_child(throttle_live_arc, 0);
            if(center_lbl) {
                snprintf(buf, sizeof(buf), "%d%%", (int)current_throttle);
                lv_label_set_text(center_lbl, buf);
            }
            if(lbl_st_motor_curr != NULL) { // Motor RPM
                snprintf(buf, sizeof(buf), "%d", current_m_amps);
                lv_label_set_text(lbl_st_motor_curr, buf);
            }
            if(lbl_controller_temp != NULL) { // Controller Temperature
                snprintf(buf, sizeof(buf), "%d\xC2\xB0" "C", current_controller_temp);
                lv_label_set_text(lbl_controller_temp, buf);
            }
            if(lbl_st_rate != NULL) { // Motor Temperature
                snprintf(buf, sizeof(buf), "%d\xC2\xB0" "C", current_motor_temp);
                lv_label_set_text(lbl_st_rate, buf);
            }
        } 
        // Tab 2 = BRAKE
        else if(current_sub_mode_tab == 2 && brake_live_arc != NULL) {
            lv_arc_set_value(brake_live_arc, (int)current_brake);
            lv_obj_t * center_lbl = lv_obj_get_child(brake_live_arc, 0);
            if(center_lbl) {
                snprintf(buf, sizeof(buf), "%d%%", (int)current_brake);
                lv_label_set_text(center_lbl, buf);
            }
            if(lbl_brake_cmd_pct) {
                snprintf(buf, sizeof(buf), "%d%%", (int)current_brake);
                lv_label_set_text(lbl_brake_cmd_pct, buf);
            }
            if(brake_cmd_bar) {
                lv_bar_set_value(brake_cmd_bar, (int)current_brake, LV_ANIM_OFF);
            }
        }
    }
    else if (current_page == PAGE_STEERING) {
        if (lbl_st_curr_angle != NULL) { 
            if(steering_err != 0) {
                lv_label_set_text(lbl_st_curr_angle, "ERR");
                lv_obj_set_style_text_color(lbl_st_curr_angle, lv_color_hex(0xFF3333), 0);
            } else {
                snprintf(buf, sizeof(buf), "%+.1f°", current_steer); 
                lv_label_set_text(lbl_st_curr_angle, buf); 
            }
        }
        if (steering_wheel_arc != NULL) lv_arc_set_value(steering_wheel_arc, (int)current_steer);
        if (lbl_st_wheel_angle != NULL) { 
            snprintf(buf, sizeof(buf), "%+.0f°", current_steer); 
            lv_label_set_text(lbl_st_wheel_angle, buf); 
        }
        if (img_steering_wheel != NULL) { 
            int32_t rotation_units = (int32_t)(-current_steer * 10.0f); 
            lv_image_set_rotation(img_steering_wheel, rotation_units); 
        }
        if (lbl_sensor_status != NULL) {
            if (steering_err != 0) {
                lv_label_set_text(lbl_sensor_status, LV_SYMBOL_WARNING " Error");
                lv_obj_set_style_text_color(lbl_sensor_status, lv_color_hex(0xFF3333), 0);
            } else {
                lv_label_set_text(lbl_sensor_status, LV_SYMBOL_OK " No Error");
                lv_obj_set_style_text_color(lbl_sensor_status, lv_color_hex(0x00FF66), 0);
            }
        }
    }
    // <-- ADDED: LIVE ERROR PAGE UPDATES
    else if (current_page == PAGE_ERRORS) {
        if (fault_card_obj != NULL && lbl_err_title != NULL && lbl_err_desc != NULL) {
            if (steering_err != 0) {
                // If Teensy sends 1, the AS5600 is disconnected
                lv_label_set_text(lbl_err_title, "CRITICAL: ERR04 - AS5600 STEERING ENCODER DISCONNECT");
                lv_obj_set_style_text_color(lbl_err_title, lv_color_hex(0xFF3333), 0);
                lv_label_set_text(lbl_err_desc, "I2C Comm Failure. Check AS5600 wiring to Teensy immediately.");
                lv_obj_set_style_border_color(fault_card_obj, lv_color_hex(0xFF3333), 0);
            } else {
                // If Teensy sends 0, everything is perfectly fine
                lv_label_set_text(lbl_err_title, "SYSTEM NOMINAL - AS5600 ENCODER ONLINE");
                lv_obj_set_style_text_color(lbl_err_title, lv_color_hex(0x00FF66), 0); // Green color
                lv_label_set_text(lbl_err_desc, "Steering position sensor is communicating correctly over I2C.");
                lv_obj_set_style_border_color(fault_card_obj, lv_color_hex(0x00FF66), 0);
            }
        }
    }
}

/************************************************
 * TIMEOUT CALLBACK IMPLEMENTATIONS
 ************************************************/
static void sub_mode_tab_click_callback(lv_event_t * e)
{
    lv_obj_t * tab = lv_event_get_target(e);
    current_sub_mode_tab = (int)(intptr_t)lv_obj_get_user_data(tab);
    lv_obj_clean(page_container);
    draw_mode_page();
}

static void mode_overlay_click_cb(lv_event_t * e)
{
    if (mode_overlay) {
        lv_obj_delete(mode_overlay);
        mode_overlay = NULL;
        pending_confirm_mode_idx = -1;
    }
}

/* YES button on the confirm dialog: activate the pending mode */
static void mode_confirm_yes_cb(lv_event_t * e)
{
    if(pending_confirm_mode_idx >= 0 && pending_confirm_mode_idx < 4) {
        // Deactivate all others, activate selected
        for(int k = 0; k < 4; k++) mode_active[k] = false;
        mode_active[pending_confirm_mode_idx] = true;
        selected_mission_preset_idx = pending_confirm_mode_idx;

        /* ---- CAN MODIFICATION: transmit the new mode over CAN ID 0x400 to the Jetson Nano.
         * Only send when the mode actually changed (i.e. user didn't re-confirm the mode
         * that was already active) and update can_active_mode after a successful send. ---- */
        if (pending_confirm_mode_idx != can_active_mode) {
            can_send_mode((uint16_t)pending_confirm_mode_idx); // 0=Standby,1=AEB,2=LKA,3=Endurance
            can_active_mode = pending_confirm_mode_idx;
        }
    }
    pending_confirm_mode_idx = -1;
    if (mode_overlay) {
        lv_obj_delete(mode_overlay);
        mode_overlay = NULL;
    }
    // Refresh the mode page to show ACTIVE state
    lv_obj_clean(page_container);
    draw_mode_page();
}

/* NO button on the confirm dialog: just close */
static void mode_confirm_no_cb(lv_event_t * e)
{
    pending_confirm_mode_idx = -1;
    if (mode_overlay) {
        lv_obj_delete(mode_overlay);
        mode_overlay = NULL;
    }
}

static void preset_grid_click_callback(lv_event_t * e)
{
    if (mode_overlay != NULL) return; // Prevent multiple popups

    lv_obj_t * card = lv_event_get_target(e);
    int mode_idx = (int)(intptr_t)lv_obj_get_user_data(card);
    pending_confirm_mode_idx = mode_idx;

    // Node names for the dialog
    const char * node_questions[] = {
        "Do you want to switch to\nStandby Mode?",
        "Do you want to run\nAutonomous Emergency Braking Node?",
        "Do you want to run\nLane Keep Assist Node?",
        "Do you want to run\nEndurance Node?"
    };

    // Create a modal overlay on main_screen
    mode_overlay = lv_obj_create(main_screen);
    lv_obj_set_size(mode_overlay, 1024, 600);
    lv_obj_set_pos(mode_overlay, 0, 0);
    lv_obj_set_style_bg_color(mode_overlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(mode_overlay, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mode_overlay, 0, 0);
    lv_obj_set_style_radius(mode_overlay, 0, 0);
    lv_obj_clear_flag(mode_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(mode_overlay, LV_OBJ_FLAG_FLOATING);
    lv_obj_add_flag(mode_overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(mode_overlay, mode_overlay_click_cb, LV_EVENT_CLICKED, NULL);

    // Dialog box centered
    lv_obj_t * dlg = lv_obj_create(mode_overlay);
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(dlg, 450, 220);
    lv_obj_align(dlg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(dlg, lv_color_hex(0x1A1A1A), 0);
    lv_obj_set_style_border_color(dlg, lv_color_hex(0xFF1E27), 0);
    lv_obj_set_style_border_width(dlg, 2, 0);
    lv_obj_set_style_radius(dlg, 16, 0);
    lv_obj_set_style_pad_all(dlg, 20, 0);
    lv_obj_clear_flag(dlg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_layout(dlg, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(dlg, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(dlg, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Question text
    lv_obj_t * q_lbl = lv_label_create(dlg);
    lv_label_set_text(q_lbl, (mode_idx >= 0 && mode_idx < 4) ? node_questions[mode_idx] : "");
    lv_obj_set_style_text_font(q_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(q_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_align(q_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(q_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(q_lbl, 410);

    // Button row
    lv_obj_t * btn_row = lv_obj_create(dlg);
    lv_obj_set_size(btn_row, 410, 60);
    lv_obj_set_style_bg_opa(btn_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btn_row, 0, 0);
    lv_obj_set_style_pad_all(btn_row, 0, 0);
    lv_obj_set_layout(btn_row, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(btn_row, LV_OBJ_FLAG_SCROLLABLE);

    // YES button
    lv_obj_t * yes_btn = lv_button_create(btn_row);
    lv_obj_set_size(yes_btn, 160, 48);
    lv_obj_set_style_bg_color(yes_btn, lv_color_hex(0x00AA44), 0);
    lv_obj_set_style_radius(yes_btn, 10, 0);
    lv_obj_t * yes_lbl = lv_label_create(yes_btn);
    lv_label_set_text(yes_lbl, "YES");
    lv_obj_set_style_text_font(yes_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(yes_lbl, lv_color_white(), 0);
    lv_obj_align(yes_lbl, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(yes_btn, mode_confirm_yes_cb, LV_EVENT_CLICKED, NULL);

    // NO button
    lv_obj_t * no_btn = lv_button_create(btn_row);
    lv_obj_set_size(no_btn, 160, 48);
    lv_obj_set_style_bg_color(no_btn, lv_color_hex(0xCC2222), 0);
    lv_obj_set_style_radius(no_btn, 10, 0);
    lv_obj_t * no_lbl = lv_label_create(no_btn);
    lv_label_set_text(no_lbl, "NO");
    lv_obj_set_style_text_font(no_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(no_lbl, lv_color_white(), 0);
    lv_obj_align(no_lbl, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(no_btn, mode_confirm_no_cb, LV_EVENT_CLICKED, NULL);
}

/************************************************
 * BACKEND LINUX SOCKETCAN TRANSCEIVER LISTENER
 ************************************************/
#ifndef _MSC_VER
static void* background_socketcan_thread(void* arg)
{
    (void)arg;
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("SocketCAN creation failed. Running in visual simulation mode.");
        
        while(1) {
            pthread_mutex_lock(&data_mutex);
            live_car_data.vehicle_speed_kmh = 45 + (rand() % 10);
            live_car_data.steer_angle_deg = sinf(time(NULL) * 0.5f) * 35.0f;
            live_car_data.brake_pct = (rand() % 15);
            live_car_data.motor_rpm = 1500 + (rand() % 200);
            live_car_data.battery_voltage = 72.0f + ((rand() % 50) / 10.0f);
            live_car_data.controller_temp = 35 + (rand() % 5);
            live_car_data.motor_temp = 40 + (rand() % 5);
            live_car_data.gear_raw = 3; // D1 in simulation mode

            // Throttle % derived SOLELY from vehicle speed (single source of truth)
            float sim_throttle = ((float)live_car_data.vehicle_speed_kmh / 40.0f) * 100.0f;
            if (sim_throttle > 100.0f) sim_throttle = 100.0f;
            if (sim_throttle < 0.0f)   sim_throttle = 0.0f;
            live_car_data.throttle_pct = sim_throttle;
            pthread_mutex_unlock(&data_mutex);
            usleep(100000); 
        }
        return NULL;
    }

    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(s);
        return NULL;
    }

    while (1) {
        int nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) continue;

        pthread_mutex_lock(&data_mutex);
        live_car_data.boot_progress = 100; 

        bool is_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
        uint32_t parsed_id = is_extended ? (frame.can_id & CAN_EFF_MASK) : (frame.can_id & CAN_SFF_MASK);

        if (is_extended) {
            if (parsed_id == MC_CAN_ID_MSG1) { // 0x180117EF: Bus Voltage / Bus Current / Phase Current / Speed

                // Bus Voltage (bytes 0-1, LE): 0.1V/bit, offset 0, range 0-300V -> Battery Voltage
                uint16_t voltage_raw = (uint16_t)frame.data[0] | ((uint16_t)frame.data[1] << 8);
                live_car_data.battery_voltage = (float)voltage_raw * 0.1f;

                // Read Speed (Little Endian, bytes 6-7)
                uint16_t speed_raw = (uint16_t)frame.data[6] | ((uint16_t)frame.data[7] << 8);
                
                // Convert to Motor RPM
                int16_t motorRPM = (int16_t)(speed_raw - 32000);
                
                // Magnitude of RPM
                float motorRPM_abs = fabs((float)motorRPM);
                
                // Wheel RPM (using macro)
                float wheelRPM = motorRPM_abs / GEAR_RATIO; 
                
                // Vehicle Speed (using macro)
                float vehicleSpeed = (wheelRPM * WHEEL_CIRCUMFERENCE * 60.0f) / 1000.0f; 

                // Store ALL calculated values safely to telemetry struct
                live_car_data.motor_rpm = (int)motorRPM;
                live_car_data.wheel_rpm = (int)wheelRPM;
                live_car_data.vehicle_speed_kmh = (int)vehicleSpeed;

                // Throttle % is derived SOLELY from Vehicle Speed - single source of
                // truth used on both the Throttle page and the Vehicle page.
                // (max speed = 40 km/h -> 100%, clamped 0-100%)
                float throttle_from_speed = (vehicleSpeed / 40.0f) * 100.0f;
                if (throttle_from_speed > 100.0f) throttle_from_speed = 100.0f;
                if (throttle_from_speed < 0.0f)   throttle_from_speed = 0.0f;
                live_car_data.throttle_pct = throttle_from_speed;
            }
            else if (parsed_id == MC_CAN_ID_MSG2) { // 0x180217EF: Controller Temp / Motor Temp / Gear / Brake / Errors

                // Controller Temperature (byte 0): 1C/bit, offset -40C
                live_car_data.controller_temp = (int)frame.data[0] - 40;

                // Motor Temperature (byte 1): 1C/bit, offset -40C
                live_car_data.motor_temp = (int)frame.data[1] - 40;

                // Gear (byte 3, bits 2-0): 0=NO,1=R,2=N,3=D1,4=D2,5=D3,6=S,7=P
                live_car_data.gear_raw = frame.data[3] & 0x07;
            }
        } else {
            switch(parsed_id) {
                case 0x202: { 
                    uint16_t raw_brake_fb = (uint16_t)frame.data[0] | ((uint16_t)frame.data[1] << 8);
                    live_car_data.brake_pct = (float)raw_brake_fb / 10.0f;
                    break;
                }
                case STEER_CAN_FEEDBACK_ID: { // 0x203 - encoded uint16 (matches Teensy encode: (wheelAngleRad+OFFSET)/SCALE)
                    uint16_t encoded = (uint16_t)frame.data[0] | ((uint16_t)frame.data[1] << 8);
                    float wheelAngleRad = ((float)encoded * STEER_ENC_SCALE) - STEER_ENC_OFFSET;
                    float steeringWheelAngleRad = wheelAngleRad * STEER_RATIO;
                    live_car_data.steer_angle_deg = steeringWheelAngleRad * (180.0f / (float)M_PI);
                    live_car_data.ind_auto = 40;
                    break;
                }
                case STEER_CAN_STATUS_ID: { // 0x063 - AS5600 status: 0 = No Error, 1 = Error
                    live_car_data.steering_status = frame.data[0]; 
                    break;
                }
                case 0x301: {
                    if (frame.can_dlc >= 2) {
                        uint16_t lat_val = (uint16_t)frame.data[0] | ((uint16_t)frame.data[1] << 8);
                        live_car_data.ind_lat = (lat_val > 0) ? 1 : 0;
                    }
                    break;
                }
                case 0x302: {
                    if (frame.can_dlc >= 2) {
                        uint16_t long_val = (uint16_t)frame.data[0] | ((uint16_t)frame.data[1] << 8);
                        live_car_data.ind_long = (long_val > 0) ? 1 : 0;
                    }
                    break;
                }
                // NOTE: Throttle Percentage is intentionally NOT decoded from any CAN
                // message (e.g. the old 0x204 / 0x06C handlers have been removed).
                // It is derived solely from Vehicle Speed above.
            }
        }
        pthread_mutex_unlock(&data_mutex);
    }

    close(s);
    return NULL;
}

/************************************************
 * DEDICATED SOCKETCAN TX SOCKET (STEERING CMDS)
 ************************************************/
static int steer_tx_socket = -1;

static void can_tx_init(void)
{
    steer_tx_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (steer_tx_socket < 0) {
        perror("SocketCAN TX socket creation failed. Steering commands will not be sent.");
        steer_tx_socket = -1;
        return;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    if (ioctl(steer_tx_socket, SIOCGIFINDEX, &ifr) < 0) {
        close(steer_tx_socket);
        steer_tx_socket = -1;
        return;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(steer_tx_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(steer_tx_socket);
        steer_tx_socket = -1;
    }
}

/************************************************
 * TARGET ANGLE ENCODE + SEND (Linux -> Teensy)
 * Mirrors the Teensy handleCAN() decode in reverse:
 *   steeringWheelAngleRad = target_deg * (PI/180)
 *   wheelAngleRad         = steeringWheelAngleRad / STEER_RATIO
 *   encoded               = (wheelAngleRad + STEER_ENC_OFFSET) / STEER_ENC_SCALE
 * Sent as CAN ID 0x103, 2-byte uint16, little-endian.
 ************************************************/
static void can_send_target_angle(float target_deg)
{
    if (target_deg > STEER_ANGLE_MAX) target_deg = STEER_ANGLE_MAX;
    if (target_deg < STEER_ANGLE_MIN) target_deg = STEER_ANGLE_MIN;

    float steeringWheelAngleRad = target_deg * ((float)M_PI / 180.0f);
    float wheelAngleRad = steeringWheelAngleRad / STEER_RATIO;
    float encodedF = (wheelAngleRad + STEER_ENC_OFFSET) / STEER_ENC_SCALE;

    if (encodedF < 0.0f) encodedF = 0.0f;
    if (encodedF > 65535.0f) encodedF = 65535.0f;
    uint16_t encoded = (uint16_t)(encodedF + 0.5f);

    if (steer_tx_socket < 0) return; // Simulation mode / no live CAN bus - nothing to send to

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = STEER_CAN_CMD_ID; // 0x103
    frame.can_dlc = 2;
    frame.data[0] = encoded & 0xFF;        // LSB
    frame.data[1] = (encoded >> 8) & 0xFF; // MSB

    write(steer_tx_socket, &frame, sizeof(struct can_frame));
}

/************************************************
 * ---- CAN MODIFICATION: AUTONOMOUS MODE SELECTION SEND ----
 * Transmits the currently selected autonomous mode (Standby/AEB/
 * LKA/Endurance) to the Jetson Nano so it can launch the matching
 * node. Sent as CAN ID 0x400, DLC=2, uint16_t little-endian.
 * Reuses the existing steer_tx_socket - no new SocketCAN socket is
 * created. Called only when the mode actually changes (see
 * mode_confirm_yes_cb and the startup send in main()).
 ************************************************/
static void can_send_mode(uint16_t mode)
{
    if (steer_tx_socket < 0) return; // Simulation mode / no live CAN bus - nothing to send to

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = AUTONOMOUS_MODE_CAN_ID; // 0x400
    frame.can_dlc = 2;
    frame.data[0] = mode & 0xFF;        // LSB
    frame.data[1] = (mode >> 8) & 0xFF; // MSB

    write(steer_tx_socket, &frame, sizeof(struct can_frame));
}
#else
static void can_tx_init(void) { }
static void can_send_target_angle(float target_deg) { (void)target_deg; }
/* ---- CAN MODIFICATION: non-Linux (simulation) stub - no live CAN bus to send to ---- */
static void can_send_mode(uint16_t mode) { (void)mode; }
#endif

/************************************************
 * MAIN TERMINAL LINUX EXECUTION PIPELINE
 ************************************************/

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

#ifndef _MSC_VER
    setenv("TZ", "Asia/Kolkata", 1);
    tzset();
#endif

    lv_init();
    lv_display_t *disp = sdl_hal_init(1024, 600);
    (void)disp;

    motorex_ui_init();
    build_footer_menu();
    draw_home_page();

    can_tx_init();

#ifndef _MSC_VER
    pthread_t can_thread_id;
    if(pthread_create(&can_thread_id, NULL, background_socketcan_thread, NULL) != 0) {
        perror("Failed to spin up POSIX thread.");
    }
#endif

    lv_timer_create(application_background_loop, 50, NULL);

    while(1)
    {
        uint32_t sleep_time_ms = lv_timer_handler();
        if(sleep_time_ms == LV_NO_TIMER_READY) {
            sleep_time_ms = LV_DEF_REFR_PERIOD;
        }
#ifdef _MSC_VER
        Sleep(sleep_time_ms);
#else
        usleep(sleep_time_ms * 1000);
#endif
    }
    return 0;
}
