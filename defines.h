#define DEBUG_SPEED     115200
#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port

#define debug Serial
#define telem Serial1

#define PFD_RST    8
#define PFD_DC     9
#define PFD_CS     10
#define TFT_MOSI   MOSI
#define TFT_MISO   MISO
#define TFT_SCK    SCK
#define MFD_RST    16
#define MFD_DC     15
#define MFD_CS     21

ILI9341_t3n pfd = ILI9341_t3n(PFD_CS, PFD_DC, PFD_RST, TFT_MOSI, TFT_SCK, TFT_MISO, &SPIN);
#define USE_MFD
#ifdef USE_MFD
ILI9341_t3n mfd = ILI9341_t3n(MFD_CS, MFD_DC, MFD_RST, TFT_MOSI, TFT_SCK, TFT_MISO, &SPIN);
#endif

SdFatSdioEX SD;
File file;

uint32_t heartbeatTimer_TX        = millis();
uint32_t heartbeatTimer_RX        = millis();
uint32_t heartbeatInterval_TX     = 1.0L * 1000L;
uint32_t heartbeatInterval_RX     = 3.0L * 1000L;
bool     mavLink_Connected        = false;

#ifdef TEST_MODE
uint64_t amountRead = 0;
uint32_t samples = 0;
uint32_t anaVal;

uint32_t anaValMin = 0;
uint32_t anaValMax = 1023;
#endif 
//uint32_t minRefresh = 1000000;
//uint32_t maxRefresh = 0;
//uint32_t refreshInterval = 0.5L * 1000L;
//uint32_t refreshInterval = 0.04L * 1000L;
//uint32_t refreshInterval = 30000;
//uint32_t refreshInterval = 60000;
//uint32_t refreshInterval = 90000;
//uint32_t refreshInterval = 57142;
//uint32_t refreshTimer = micros();

float     converts = 3.6;   // convert m/s to km/h
float     converth = 1.0;   // disatance to m

//#define FRAME_BUFFER_SIZE   (ILI9341_TFTWIDTH * ILI9341_TFTHEIGHT * 2)
//uint32_t * scr = screen32;
//uint16_t * scr = screen16;
DMAMEM uint16_t frame_buffer[ILI9341_TFTWIDTH][ILI9341_TFTHEIGHT];
uint16_t * scr = (uint16_t*)&frame_buffer[0][0];

//uint16_t  *scr = NULL;

#define TFT_WIDTH           320 //ILI9341_TFTWIDTH
#define TFT_HEIGHT          240 //ILI9341_TFTHEIGHT 
#define XMIN                (int32_t)0                // Left horizon
#define XMAX                (int32_t)(TFT_WIDTH)      // Right horizon
#define YMIN                (int32_t)20               // Top horizon 
#define YMAX                (int32_t)(TFT_HEIGHT -11) // Bot horizon
#define H_ROWS              220
#define H_COLS              TFT_WIDTH
#define X_PIVOT             H_COLS / 2
#define Y_PIVOT             88

const uint16_t SKY_COLOR        = pfd.color565(0, 76, 255);
const uint16_t GND_COLOR        = pfd.color565(120, 59, 20);
const uint16_t TFT_TRANSPARENT  = ILI9341_MAGENTA;
const uint16_t TFT_OPAQUE       = pfd.color565(0, 36, 0);

static float                alt_prev = 0;             // previous altitude
static uint8_t              alt_cnt = 0;              // counter for stable osd_alt
//MAVLink session control
static boolean              mavbeat = 0;
static float                lastMAVBeat = 0;
static boolean              waitingMAVBeats = 1;
static uint8_t              apm_mav_system; 
static uint8_t              apm_mav_component;
static boolean              enable_mav_request = 0;
static uint8_t              flight_mode = 0;                   // Navigation mode from RC AC2 = CH5, APM = CH8
static uint8_t              osd_nav_mode = 0;               // Navigation mode from RC AC2 = CH5, APM = CH8
static uint8_t              apm_mav_type = 0;
static uint32_t             time_boot_ms = 0;

typedef struct {
  int                   x = 0;                   // [rad] Roll angle (-pi..+pi)
  int                   y = 0;                  // [rad] Pitch angle (-pi..+pi)
} __attribute__((__packed__))POINT;

//typedef struct __mavlink_attitude_t
//{
// uint64_t usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
// float roll; ///< Roll angle (rad)
// float pitch; ///< Pitch angle (rad)
// float yaw; ///< Yaw angle (rad)
// float rollspeed; ///< Roll angular speed (rad/s)
// float pitchspeed; ///< Pitch angular speed (rad/s)
// float yawspeed; ///< Yaw angular speed (rad/s)
//} mavlink_attitude_t;

typedef struct {
  float                   roll = 0;                   // [rad] Roll angle (-pi..+pi)
  float                   pitch = 0;                  // [rad] Pitch angle (-pi..+pi)
  float                   yaw = 0;                    // [rad] Yaw angle (-pi..+pi)
  float                   rollspeed;                  //  Roll angular speed (rad/s)
  float                   pitchspeed;                 //  Pitch angular speed (rad/s)
  float                   yawspeed;                   //  Yaw angular speed (rad/s)
  float                   sinRoll = sin(roll * DEG_TO_RAD);                // Seno angle Roll
  float                   cosRoll = cos(roll * DEG_TO_RAD);                // Cosine angle Roll
} __attribute__((__packed__))AHRS;

typedef struct {
  float                   lat ;
  float                   lon;
  float                   alt;
} __attribute__((__packed__))WAYPOINT;

typedef struct {
  float                   direction ;
  float                   speed;
  float                   speed_z;
} __attribute__((__packed__))WIND;

typedef struct {
  uint8_t                 got_home = 0;               // tels if got home position or not
  WAYPOINT                position; 
  float                   distance = 0;               // distance from home
  float                   direction;                  // {Arrow direction pointing to home (1-16 to CW loop)}
  float                   glide = 0;
  uint32_t                ete = 0;                    // etimated time enroute in seconds
} __attribute__((__packed__))HOME;

typedef struct {
  AHRS                    ahrs;                       // [deg] Current desired roll/pitch/yaw
  WAYPOINT                wp;
  int16_t                 wp_target_bearing = 0;      // [deg] Current desired heading
  int16_t                 target_bearing;             // [deg] Bearing to current waypoint/target
  uint16_t                wp_dist = 0;                // [m] Distance to active waypoint
  float                   alt_error = 0;              // Current altitude error in meters
  float                   aspd_error = 0;             // Current airspeed error in meters/second
  float                   xtrack_error = 0;           // Current crosstrack error on x-y plane in meters
  uint8_t                 wp_number = 0;              // Current waypoint number
  uint32_t                ete = 0;                    // etimated time enroute in seconds
} __attribute__((__packed__))NAV;

//enum GPS_MODE { 
//  GPS_NO = 0,
//  GPS_NO_FIX = 1,
//  GPS_2D = 2,
//  GPS_3D = 3
//}

#define                   GPS_FIX_TYPE_NO_GPS     0 //No GPS connected
#define                   GPS_FIX_TYPE_NO_FIX     1 //No position information, GPS is connected
#define                   GPS_FIX_TYPE_2D_FIX     2 //2D position
#define                   GPS_FIX_TYPE_3D_FIX     3 //3D position
#define                   GPS_FIX_TYPE_DGPS       4 //DGPS/SBAS aided 3D position
#define                   GPS_FIX_TYPE_RTK_FLOAT  5 //RTK float, 3D position
#define                   GPS_FIX_TYPE_RTK_FIXED  6 //RTK Fixed, 3D position
#define                   GPS_FIX_TYPE_STATIC     7 //Static fixed, typically used for base stations
#define                   GPS_FIX_TYPE_PPP        8 //PPP, 3D position.

typedef struct {
  bool                    connected = false;
  uint16_t                hb_count = 0;               // heart beat count
  float                   vbatA = 0;                  // Battery A voltage in milivolt
  float                   ibatA = 0;                  // Battery A current
  int8_t                  remainingA = 0;             // 0 to 100 <=> 0 to 1000
  bool                    motor_armed = 0;
  /* MAVLINK_MSG_ID_GPS_RAW_INT */
  WAYPOINT                position;
  uint8_t                 fix_type = GPS_FIX_TYPE_NO_GPS;   // GPS lock 0-1=no fix, 2=2D, 3=3D
  uint8_t                 satellites_visible = 0;     // number of satelites
  /* MAVLINK_MSG_ID_VFR_HUD 74 */
  float                   airspeed = 0;               // [m/s] Current indicated airspeed (IAS).
  float                   groundspeed = 0;            // [m/s] Current ground speed.
  float                   alt = 0;                    // [m] Current altitude (MSL).
  float                   relative_alt = 0;           // [m] Current altitide (AGL)
  float                   climb = 0;                  // [m/s] Current climb rate.
  int16_t                 heading = 0;                // [deg] Current heading in compass units (0-360, 0=north).
  uint16_t                throttle = 0;               // [%] Current throttle setting (0 to 100).
  
  AHRS                    ahrs;
} __attribute__((__packed__))APM;

APM             apm;
NAV             nav;
HOME            home;
WIND            wind;

#define MAX_PITCH   60.0f
#define MIN_PITCH   -60.0f
