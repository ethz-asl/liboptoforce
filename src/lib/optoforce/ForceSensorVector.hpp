/*
* ForceSensorVector.hpp
*
*  Created on: Aug. 5, 2013
*      Author: markho
*/

#ifndef FORCESENSORVECTOR_HPP
#define FORCESENSORVECTOR_HPP

#include <boost/ptr_container/ptr_vector.hpp>

#include <optoforce/ForceSensorBase.hpp>

class ForceSensorVector {
public:
  ForceSensorVector();
  ~ForceSensorVector();

  /*! Adds a Sensor
  * @param	pointer to the sensor
  */
  void  operator+= (ForceSensorBase * sensor);

  /*! Removes a sensor by index.
  * @param	index	index of sensor (first index is 0)
  */
  void operator-=(unsigned int index);

  /*! Gets the Sensor by index
  * @param index index of the Sensor (first index is 0)
  * @return	pointer to the Sensor
  */
  ForceSensorBase *  operator[] (unsigned int index);

  int size() {return forceSensorVector_.size();}
protected:
  boost::ptr_vector<ForceSensorBase> forceSensorVector_;
}; //End of class

#endif /* FORCESENSORVECTOR_HPP */
