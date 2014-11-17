/*
* ForceSensor.hpp
*
*  Created on: Aug. 5, 2013
*      Author: markho
*/

#ifndef FORCESENSOR_HPP
#define FORCESENSOR_HPP

#include <iostream>

#include <optoforce/ForceSensorBase.hpp>

//!  of the ForceSensor Class
/*! This is a class that provides a general interface
* to manage ForceSensors
*/

template <class T>
class ForceSensor : public ForceSensorBase{
  //Not used so far:
  inline friend std::ostream& operator<<(std::ostream& output, const
      ForceSensor & fd) {
    //	output << *(fd.sensor_) << std::endl;
    return output;
  };
public:
  inline ForceSensor() : ForceSensorBase() {
    sensor_ = NULL;
  };
  
  inline ~ForceSensor() {
    if (sensor_ != NULL) {
      sensor_->exit();
    }
    delete sensor_;
  };

  void operator= (T & sensor){
    if (sensor_ != NULL) {
      delete sensor_;
    }
    sensor_ = new T(sensor); //Use the copy constructor
  };

  inline void init() {
    if (sensor_ != NULL) {
      sensor_->init();
    }
  };

  inline void update() {
    if (sensor_ != NULL) {
      sensor_->update();
    }
  };

  inline void exit() {
    if(sensor_ != NULL) {
      sensor_->exit();
    }
  };

  inline void storeSensorOffset(std::string filename, unsigned int
      nr_elements_for_filter) {
    if (sensor_ != NULL) {
      sensor_->storeSensorOffset(filename, nr_elements_for_filter);
    }
  };

  inline void loadSensorOffset(std::string filename) {
    if(sensor_!=NULL) { sensor_->loadSensorOffset(filename); }
  };

  inline int * returnXYZRaw(void) {
    return sensor_->returnXYZRaw();
  };
  
  inline int * returnXYZCalibrated(void){
    return sensor_->returnXYZCalibrated();
  };
private:
  T * sensor_;
}; //End of class

#endif /* FORCESENSOR_HPP */
