/*
 * ForceSensor.hpp
 *
 *  Created on: Aug. 5, 2013
 *      Author: markho
 */

#ifndef FORCESENSOR_HPP_
#define FORCESENSOR_HPP_

#include"ForceSensorBase.hpp"
#include<iostream>


//!  of the ForceSensor Class
/*! This is a class that provides a general interface
 * to manage ForceSensors
 */

template <class T>
class ForceSensor : public ForceSensorBase{

	//Not used so far:
	friend std::ostream& operator<<(std::ostream& output, const ForceSensor & fd) {
		//	output << *(fd.sensor_) << std::endl;
		return output;
	}

public:
	ForceSensor() : ForceSensorBase(){ sensor_ = NULL;}
	~ForceSensor(){
		if(sensor_!=NULL) { sensor_->exit();  }
		delete sensor_; }

	void operator= (T & sensor){
		if(sensor_!=NULL) delete sensor_;
		sensor_ = new T(sensor); //Use the copy constructor
	}

	void init(){
		if(sensor_!=NULL) { sensor_->init(); }
	};

	void update(){
		if(sensor_!=NULL) { sensor_->update(); }
	};

	void exit(){
		if(sensor_!=NULL) { sensor_->exit();  }
	}

	void storeSensorOffset(std::string filename,unsigned int nr_elements_for_filter){


		if(sensor_!=NULL) { sensor_->storeSensorOffset(filename, nr_elements_for_filter); }

	}

	void loadSensorOffset(std::string filename) {
		if(sensor_!=NULL) { sensor_->loadSensorOffset(filename); }
	}

	int * returnXYZRaw(void){ return sensor_->returnXYZRaw(); }
	int * returnXYZCalibrated(void){ return sensor_->returnXYZCalibrated(); }



private:
	T * sensor_;

}; //End of class




#endif /* FORCESENSOR_HPP_ */
