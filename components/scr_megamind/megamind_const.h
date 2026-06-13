// hardware tunables
#define LORA_FREQ (915E6)
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
#define REQ_COUNT_STATE_CHANGE (5U)      // number of valid checks required before a state change is allowed
#define LANDING_DIVERGENCE_THRESH (1)   // divergence threshold for landing detection