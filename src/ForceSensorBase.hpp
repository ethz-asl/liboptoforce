/*
 * FORCESENSORBASE_HPP_
 *
 *  Created on: Aug. 5, 2013
 *      Author: markho
 */

#ifndef FORCESENSORBASE_HPP_
#define FORCESENSORBASE_HPP_

#include<string>

class ForceSensorBase{
public:
  ForceSensorBase();
  ~ForceSensorBase();
  std::string name(); //Returns the name of the sensor

  virtual void init() = 0;
  virtual void update() = 0;
  virtual void exit() = 0;
  virtual void storeSensorOffset(std::string filename,unsigned int nr_elements_for_filter) = 0;
  virtual void loadSensorOffset(std::string filename) = 0;

  virtual int * returnXYZRaw() = 0;
  virtual int * returnXYZCalibrated() = 0;

private:
  std::string name_;

}; //End of class


#endif /* FORCESENSORBASE_HPP_ */
