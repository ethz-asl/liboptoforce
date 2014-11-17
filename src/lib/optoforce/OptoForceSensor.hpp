/*
* OptoForceSensor.hpp
*
*  Created on: Aug, 2. 2013
*      Author: markho
*/

#ifndef OPTOFORCESENSOR_HPP
#define OPTOFORCESENSOR_HPP

//Comments: Calibration so far only removes a constant bias

//ToDo: Get rid of this SL trap:
#undef E //Dangerous: E is defined by SL, so undef only works at the correct location

#include<iostream>
#include<fstream>
#include <time.h>    
#include <deque>

#include <optoforce/OptoForceParameters.hpp>
#include <optoforce/DisplayPackage.hpp>
#include <optoforce/StateMachine.hpp>

class OptoForceSensor{
  //Not used so far:
  inline friend std::ostream& operator<<(std::ostream& output, const
      OptoForceSensor & gd) {
    //  output << *(gd.data_);
    return output;
  };
public:
  inline OptoForceSensor() {
    state_machine_ = new StateMachine();
    baudrate_ = 115200; //Set the default baudrate
    //		device_name_ = "unknown";
    device_name_  = (char *)"unknown";
    //		tigetstr((char *)"setf");
  };

  inline OptoForceSensor(std::string device_name, int baudrate) {
    baudrate_ = baudrate; //ToDo: Check for valid baudrates already here
    device_name_ = new char [device_name.length()+1];
    std::strcpy(device_name_, device_name.c_str());

    //Set the member variables to valid numbers:
    for(int i=0;i<3;i++){
      readings_xyz_calibrated[i] =  0;
      readings_xyz_raw[i] = 0;
      readings_xyz_offset[i] = 0;
    }

    std::cout << "OptoForcerSensor: Generating state machine for sensor " <<
      device_name_ << " with Baud Rate: " << baudrate_ << std::endl;
    state_machine_ = new StateMachine();
  };

  inline ~OptoForceSensor() {
    if (state_machine_->is_open()) {
      state_machine_->close();
    }
    delete state_machine_;
    delete device_name_;
  };


  inline void init() {
    if (state_machine_->is_open()) {
      state_machine_->close();
    }

    std::cout << "OptoForcerSensor-init: Serial communication: " <<
      "Device name: " << device_name_ << ", Baud Rate: " << baudrate_ <<
      std::endl;

    if(!state_machine_->open(device_name_, baudrate_)) {
      std::cerr << "OptoForceSensor: " <<
        "Error at opening the device in the state machine" << std::endl;
      std::exit(-1);
    }

    //Configure the sensor:
    state_machine_->write_configuration(frequency_sampling_sensor,
      frequency_filter, flag_raw);
  };

  inline void update() {
    state_machine_->read(packs, offsets);

    if (packs.size() > 0) {
      sp::DisplayPackage& current_value = packs.back();

      readings_xyz_raw[0] = current_value.x;
      readings_xyz_raw[1] = current_value.y;
      readings_xyz_raw[2] = current_value.z;
      
      for (int i = 0; i < 3; i++) {
        readings_xyz_calibrated[i] = (int) ( optoforce_prop_const *
          (float)(readings_xyz_raw[i] - readings_xyz_offset[i]) );
      }

      if (VERBOSE) {
        const int output_mod_const = 1000;
        static int counter = 0;
        counter++;
        gettimeofday(&time_n, NULL);
        long d_sec, d_usec;
        d_sec  = time_n.tv_sec  - time_n_minus_one.tv_sec; //So far unused
        d_usec = time_n.tv_usec - time_n_minus_one.tv_usec;
        gettimeofday(&time_n_minus_one, NULL);

        if (counter % output_mod_const == 0)  {
          for (unsigned int i = 0; i < packs.size(); i++) {
            printf("Pack: %i, delta time us: %i \t Data raw: !%5i,%5i,%5i$\n",
              i, (int)d_usec, readings_xyz_raw[0], readings_xyz_raw[1],
              readings_xyz_raw[2]);
            printf("Pack: %i, delta time us: %i \t Data cal: !%5i,%5i,%5i$\n",
              i, (int)d_usec, readings_xyz_calibrated[0],
              readings_xyz_calibrated[1], readings_xyz_calibrated[2]);
          } //End print for all packs
        } //End if counter modulo
      } //End if verbose
      packs.clear();
    } //End if packs > 0
  };

  //Stores offsets and removes the offset from the sensor values
  inline void storeSensorOffset(std::string filename, unsigned int
      nr_elements_for_filter) {
    //Simple avg filter:
    static int offsets[3];

    for(int i = 0; i < 3; i++) {
      readings_xyz_offset[i] = 0;
    } //Set the actual offset to zero //ToDo: Ev. protection with semaphores

    for(int j = 0; j < nr_elements_for_filter; ++j) {
      std::cout << "OptoForceSensor> Storing value nr " << j << std::endl;
      
      update();

      //Don't count the initial readings, when only zeros are transmitted:
      if ((readings_xyz_raw[0] == 0) && (readings_xyz_raw[1] == 0) &&
          (readings_xyz_raw[2] == 0) ) {
        j--;
        continue;
      }

      for (int i = 0; i < 3; i++){
        readings_xyz_offset[i] += readings_xyz_raw[i];
      }

      //Let the sensor produce data (Could be adapted according to sampling rate of the sensor)
      usleep(5000);
    } //End for all the sensor samples

    for(int i = 0; i < 3; i++) {
      readings_xyz_offset[i] = (int) ( (float) readings_xyz_offset[i] /
        (float) nr_elements_for_filter);      
    }

    //Store the offsets in a file
    FILE * myfile = fopen( filename.c_str(), "w" );

    if (myfile != NULL) {
      fprintf(myfile, "%5i, %5i, %5i\n", readings_xyz_offset[0],
        readings_xyz_offset[1], readings_xyz_offset[2]);
      fclose(myfile);
      std::cout << "Stored the sensor offset in file: " << filename <<
        std::endl;
    }
    else {
      std::cerr << "OptoForceSensor: Error at writing to file " << filename <<
        std::endl;
    }
  };

  //Load offsets from file and removes the offset from the sensor values
  inline void loadSensorOffset(std::string filename) {
    std::cout << "Loading the sensor offset" << std::endl;

    FILE * myfile = fopen( filename.c_str(), "r" );

    if (myfile!=NULL) {
      rewind(myfile);
      int n = fscanf(myfile, "%i, %i, %i", readings_xyz_offset,
        readings_xyz_offset+1, readings_xyz_offset+2);
      fclose (myfile);
    }
    else {
      std::cerr << "OptoForceSensor: Error at reading the bias from file " <<
        filename << std::endl;
    }
  };

  inline int * returnXYZRaw(void) {
    return readings_xyz_raw;
  };
  
  inline int * returnXYZCalibrated(void) {
    return readings_xyz_calibrated;
  };

  inline void exit() {
    if (state_machine_->is_open()) {
      state_machine_->close();
    }
  };
private:
  int baudrate_;
  char * device_name_;
  
  int readings_xyz_raw[3];
  int readings_xyz_calibrated[3];
  int readings_xyz_offset[3];

  StateMachine * state_machine_;
  std::deque<sp::DisplayPackage> packs;
  sp::DisplayPackage offsets;
  struct timeval time_n_minus_one, time_n;
};

#endif // OPTOFORCESENSOR_HPP
