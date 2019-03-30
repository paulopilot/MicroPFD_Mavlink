//#include "mavlink/1.0/common/mavlink.h"
//#include "mavlink/1.0/mavlink_types.h"
//#include "mavlink/1.0/ardupilotmega/mavlink.h"


// Initialize the required buffers 
mavlink_message_t         receivedMsg;
mavlink_status_t          mav_status;
mavlink_set_mode_t        mode;
mavlink_system_t          mavlink_system;
mavlink_message_t         heartbeatMsg;
mavlink_heartbeat_t       heartbeat;

uint8_t bufTx[MAVLINK_MAX_PACKET_LEN];
uint8_t bufRx[MAVLINK_MAX_PACKET_LEN];
uint8_t system_type = MAV_TYPE_FIXED_WING; //MAV_TYPE_GROUND_ROVER; //MAV_TYPE_HELICOPTER;//MAV_TYPE_FIXED_WING;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC; //MAV_AUTOPILOT_ARDUPILOTMEGA



// true when we have received at least 1 MAVLink packet
static bool mavlink_active;
static uint8_t crlf_count = 0;

static int packet_drops = 0;
static int parse_error = 0;

void clearVAriables() {
  apm.connected           = false;
  apm.hb_count            = 0;
  apm.vbatA               = 0;
  apm.ibatA               = 0;
  apm.remainingA          = 0;
  apm.motor_armed         = false;
  apm.position.lat        = 0;
  apm.position.lon        = 0;
  apm.position.alt        = 0;
  apm.fix_type            = GPS_NO;
  apm.satellites_visible  = 0;
  apm.airspeed            = 0;
  apm.groundspeed         = 0;
  apm.heading             = 0;
  apm.throttle            = 0;
  apm.alt                 = 0;
  apm.climb               = 0;
  apm.ahrs.pitch          = 0;
  apm.ahrs.roll           = 0;
  apm.ahrs.yaw            = 0;
}


void request_mavlink_rates()
{
  const int  maxStreams = 6;
  const uint8_t MAVStreams[maxStreams] = {
      MAV_DATA_STREAM_RAW_SENSORS,
      MAV_DATA_STREAM_EXTENDED_STATUS,
      MAV_DATA_STREAM_RC_CHANNELS,
      MAV_DATA_STREAM_POSITION,
      MAV_DATA_STREAM_EXTRA1, 
      MAV_DATA_STREAM_EXTRA2};
  const uint16_t MAVRates[maxStreams] = {0x02, 0x02, 0x05, 0x02, 0x05, 0x02};
  for (int i=0; i < maxStreams; i++) {
    //mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,apm_mav_system, apm_mav_component, MAVStreams[i], MAVRates[i], 1);
  }
}

void handle_Messages(){
  mavlink_message_t msg; 
  mavlink_status_t status;

  if((((millis() - heartbeatTimer_RX) > heartbeatInterval_RX)) && apm.connected) { // if no HEARTBEAT from APM in 3.0s then we are not connected
    clearVAriables();
  } 
    
  while(telem.available()>0) {
    uint8_t c = telem.read();
  
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &receivedMsg, &mav_status)) {
        //print_heartbeat();
        if(receivedMsg.msgid > 0){
          //debug.print(" -> Msg ID: ");
          //debug.println(receivedMsg.msgid, DEC);
          if(receivedMsg.msgid == 11){
            if(mode.base_mode == MAV_MODE_GUIDED_ARMED || mode.base_mode == MAV_MODE_MANUAL_ARMED){
                Serial.println("SYSTEM ARMED");
                //print_setMode();
                //motor_control = 1;
              } else {
                debug.print("--> New Base Mode: ");
                debug.println(mode.base_mode);
              }
          }
        }

        switch(receivedMsg.msgid)
        {
          case MAVLINK_MSG_ID_HEARTBEAT:
                mavbeat = 1;
                apm_mav_system    = receivedMsg.sysid;
                apm_mav_component = receivedMsg.compid;
                apm_mav_type      = mavlink_msg_heartbeat_get_type(&receivedMsg);            
                flight_mode = (uint8_t)mavlink_msg_heartbeat_get_custom_mode(&receivedMsg);
                
                base_mode = mavlink_msg_heartbeat_get_base_mode(&receivedMsg);
                if(bitRead(base_mode,7)) 
                  apm.motor_armed = true;
                else 
                  apm.motor_armed = false;

                osd_nav_mode = 0;          
                //base_mode = (mavlink_msg_heartbeat_get_base_mode(&msg) & 0x80) > 7;
                heartbeatTimer_RX = millis(); //Reset receive timer
                if(!apm.connected); {
                  apm.hb_count++; 
                  if((apm.hb_count++) > 10) { // If received > 10 heartbeats from MavLink then we are connected
                    apm.connected = true;
                    apm.hb_count=0;
                    //digitalWrite(led,HIGH); // LED will be ON when connected to MavLink, else it will slowly blink
                  }
                }
                break;
          case MAVLINK_MSG_ID_SYS_STATUS:
                apm.vbatA = (mavlink_msg_sys_status_get_voltage_battery(&receivedMsg) / 1000.0f); //Battery voltage, in millivolts (1 = 1 millivolt)
                //apm.ibatA = mavlink_msg_sys_status_get_current_battery(&receivedMsg); //Battery current, in 10*milliamperes (1 = 10 milliampere)   
                apm.ibatA = (float(mavlink_msg_sys_status_get_current_battery(&receivedMsg)) * .01);    
                apm.remainingA = mavlink_msg_sys_status_get_battery_remaining(&receivedMsg); //Remaining battery energy: (0%: 0, 100%: 100)
                //Serial.printf("VbatA: %02.1fV IbatA: %03.1fma\n", apm.vbatA, apm.ibatA);
                break;
          case MAVLINK_MSG_ID_GPS_RAW_INT:
                apm.position.lat = mavlink_msg_gps_raw_int_get_lat(&receivedMsg) / 10000000.0f;
                apm.position.lon = mavlink_msg_gps_raw_int_get_lon(&receivedMsg) / 10000000.0f;
                apm.fix_type = mavlink_msg_gps_raw_int_get_fix_type(&receivedMsg);
                apm.satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&receivedMsg);
                //Serial.printf("Lat: %f Lon: %f Sat: %d\n", apm.position.lat, apm.position.lon, (int)apm.satellites_visible);
                break;
          case MAVLINK_MSG_ID_VFR_HUD:
                apm.airspeed = mavlink_msg_vfr_hud_get_airspeed(&receivedMsg);
                apm.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&receivedMsg);
                apm.heading = mavlink_msg_vfr_hud_get_heading(&receivedMsg); // 0..360 deg, 0=north
                apm.throttle = mavlink_msg_vfr_hud_get_throttle(&receivedMsg);
                apm.alt = mavlink_msg_vfr_hud_get_alt(&receivedMsg);
                apm.climb = mavlink_msg_vfr_hud_get_climb(&receivedMsg);
                //Serial.printf("IAS: %03d GS: %03d HDG: %03d ALT: %03d\n", (int)apm.airspeed, (int)apm.groundspeed, (int)apm.heading, (int)apm.alt);
                break;
          case MAVLINK_MSG_ID_ATTITUDE:
                apm.ahrs.roll = mavlink_msg_attitude_get_roll(&msg) *180/3.1416;
                apm.ahrs.pitch = mavlink_msg_attitude_get_pitch(&msg) *180/3.1416;
                apm.ahrs.yaw = mavlink_msg_attitude_get_yaw(&msg) * 180/3.1416;
                Serial.printf("APM Pitch: %03.0f Roll: %03.0f Yaw: %03.0f\n", apm.ahrs.pitch, apm.ahrs.roll, apm.ahrs.yaw);
                break;
          case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
                nav.ahrs.roll = mavlink_msg_nav_controller_output_get_nav_roll(&msg);
                nav.ahrs.pitch = mavlink_msg_nav_controller_output_get_nav_pitch(&msg);
                nav.ahrs.yaw = mavlink_msg_nav_controller_output_get_nav_bearing(&msg);
                nav.wp_target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(&msg);
                nav.wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
                nav.alt_error = mavlink_msg_nav_controller_output_get_alt_error(&msg);
                nav.aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(&msg);
                nav.xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(&msg);
                Serial.printf("NAV Pitch: %03.0f Roll: %03.0f Yaw: %03.0f\n", nav.ahrs.pitch, nav.ahrs.roll, nav.ahrs.yaw);
                //Serial.printf("NAV WP Dist: %03d Roll: %03d Yaw: %03d\n", (int)nav.wp_dist, (int)nav.ahrs.roll, (int)nav.ahrs.yaw);
                
                break;
          case MAVLINK_MSG_ID_MISSION_CURRENT:
                nav.wp_number = (uint8_t)mavlink_msg_mission_current_get_seq(&msg);
                //Serial.printf("WP: %03d\n", (int)nav.wp_number);
                break;
          case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                //chan1_raw = mavlink_msg_rc_channels_raw_get_chan1_raw(&msg);
                //chan2_raw = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
                //osd_chan5_raw = mavlink_msg_rc_channels_raw_get_chan5_raw(&msg);
                //osd_chan6_raw = mavlink_msg_rc_channels_raw_get_chan6_raw(&msg);
                //osd_chan7_raw = mavlink_msg_rc_channels_raw_get_chan7_raw(&msg);
                //osd_chan8_raw = mavlink_msg_rc_channels_raw_get_chan8_raw(&msg);
                //osd_rssi = mavlink_msg_rc_channels_raw_get_rssi(&msg);
                break;
            case MAVLINK_MSG_ID_WIND:
                //osd_winddirection = mavlink_msg_wind_get_direction(&msg); // 0..360 deg, 0=north
                //osd_windspeed = mavlink_msg_wind_get_speed(&msg); //m/s
                //osd_windspeedz = mavlink_msg_wind_get_speed_z(&msg); //m/s
                break;
            default:
                //Do nothing
                break;
        }
    }
  }
}

void send_heartbeat(){
    memset(bufTx, 0xFF, sizeof(bufTx));
    mavlink_system.sysid = MAV_TYPE_FIXED_WING ;
    mavlink_system.compid = MAV_AUTOPILOT_GENERIC; 
    
    // Pack the message 
    mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &heartbeatMsg, system_type, autopilot_type, heartbeat.base_mode, heartbeat.custom_mode, heartbeat.system_status);
  
    // Copy the message to send buffer 
    uint16_t len = mavlink_msg_to_send_buffer(bufTx, &heartbeatMsg);
 
    //Write Message    
    telem.write(bufTx, len);        
    heartbeatTimer_TX = millis();
    Serial.println("Heartbeat"); 
}
