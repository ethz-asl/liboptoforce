
#ifndef OPTOFORCE_PARAMETERS_H
#define OPTOFORCE_PARAMETERS_H
 
//For debugging
#define VERBOSE 0 //0: no add. output, 1: Additional debugging output

//For using the sensor
#define NR_FORCE_SENSORS 4
extern const char* ifaces_USB[NR_FORCE_SENSORS];
extern const unsigned int baudrate;

extern const unsigned int frequency_sampling_sensor; //Sensor options: 1000(def),333,100, 33 [hz]
extern const unsigned int frequency_filter; //Options: 00 (no filter),150,50, 15 [hz]
extern const unsigned int flag_raw; //Readout option: flag_raw=0: raw (def), 1: force
extern const unsigned int runs_for_calibration; //Execute the iteration x times before recalibrating the sensors

extern const float optoforce_prop_const;

#endif
