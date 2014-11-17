/*
 * FootholdFeatureBase.cpp
 *
 *   Created on: Aug. 5, 2013
 *      Author: markho
 */

#include"ForceSensorVector.hpp"


#include <boost/lexical_cast.hpp>
#include <stdexcept> //Exceptions, e.g. std::out_of_range
#include<iostream>

 ForceSensorVector:: ForceSensorVector(){
   std::cout << " ForceSensorVector: Constructor called" << std::endl;
}

 ForceSensorVector::~ ForceSensorVector(){
   forceSensorVector_.clear();
}


void   ForceSensorVector::operator+= (ForceSensorBase * forceSensor) {
	std::cout << "Adding a forceSensor to Vector with " << forceSensorVector_.size() << " Elements" << std::endl;
	forceSensorVector_.push_back(forceSensor);
}


void  ForceSensorVector::operator-=(unsigned int index){
  
  try {
    if (index >= forceSensorVector_.size()) {
      std::string error = " ForceSensorVector: Could not remove sensor with index " + boost::lexical_cast<std::string>(index) + "!"; //Lexical: Convert to std::string
      throw std::out_of_range(error);
    }
    forceSensorVector_.erase(forceSensorVector_.begin()+index);
    std::cout << "Removing sensor with index " << index << std::endl;
    
  }	catch (std::exception& e) {
    std::cout << e.what() << std::endl;
  }  
}


ForceSensorBase *   ForceSensorVector::operator[] (unsigned int index){
	try {
		if (index >= forceSensorVector_.size()) {
			std::string error = " ForceSensorVector: Could not acceess the force sensor with index "
					+ boost::lexical_cast<std::string>(index) + "!"; //Lexical: Convert to std::string
			throw std::out_of_range(error);
		}
		return &(forceSensorVector_[index]);

	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
	}
	return NULL;
}
