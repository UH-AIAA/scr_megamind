// hardware tunables
#define LORA_FREQ (426.85E6)
#define SD_FREQ (20E6)

// memory tunables
#define MAX_SENSOR_QUEUE_SIZE (200U)  // napkin math says this is abt 1s of data?

// calibration tunables
#define NUM_CALIBRATION_SAMPLES (4096U)
#define CALIBRATION_DIVERGE_THRESH (10)

// filtering tunables
#define BMP_LOWPASS_ALPHA (0.35)

// state machine tunables
#define ASCENT_THRESHOLD (40U)          // m/s^2, roughly 4Gs
#define REQ_COUNT_STATE_CHANGE (5U)     // number of valid checks required before a state change is allowed
#define APG_NEG_VEL_THRESH (-1.5E6)       // negative velocity threshold in apogee detection
#define LANDING_DIVERGENCE_THRESH (1)   // divergence threshold for landing detection