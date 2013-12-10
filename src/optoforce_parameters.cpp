
#include"optoforce_parameters.h"

const char* ifaces_USB[NR_FORCE_SENSORS] = { "/dev/ttyACM0", "/dev/ttyACM1","/dev/ttyACM2","/dev/ttyACM3" };
const unsigned int baudrate = 115200;

const unsigned int frequency_sampling_sensor = 1000;
const unsigned int frequency_filter = 0;
const unsigned int flag_raw = 1;
const unsigned int runs_for_calibration = 5;
const float optoforce_prop_const = 10.0/2.2;

