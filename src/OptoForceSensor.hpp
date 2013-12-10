/*
 * OptoForceSensor.hpp
 *
 *  Created on: Aug, 2. 2013
 *      Author: markho
 */

#ifndef OPTOFORCESENSOR_HPP_
#define OPTOFORCESENSOR_HPP_



//Comments: Calibration so far only removes a constant bias

//ToDo: Get rid of this SL trap:
#undef E //Dangerous: E is defined by SL, so undef only works at the correct location


#include<iostream>
#include<fstream>
#include <time.h>    
#include <deque>



#include"optoforce_parameters.h"
#include"helper.h" //Contains struct for pack
#include"state_machine_wo_qt.h"


using namespace std;
class OptoForceSensor{

	//Not used so far:
	friend std::ostream& operator<<(std::ostream& output, const OptoForceSensor & gd) {
		//  output << *(gd.data_);
		return output;
	}

public:
	OptoForceSensor(){
		state_machine_ = new state_machine();
		baudrate_ = 115200; //Set the default baudrate
		//		device_name_ = "unknown";
		device_name_  = (char *)"unknown";
		//		tigetstr((char *)"setf");

	}

	OptoForceSensor(std::string device_name, int baudrate){

		baudrate_ = baudrate; //ToDo: Check for valid baudrates already here
		device_name_ = new char [device_name.length()+1];
		std::strcpy(device_name_, device_name.c_str());

		//Set the member variables to valid numbers:
		for(int i=0;i<3;i++){
			readings_xyz_calibrated[i] =  0;
			readings_xyz_raw[i] = 0;
			readings_xyz_offset[i] = 0;
		}

		cout << "OptoForcerSensor: Generating state machine for sensor " << device_name_ << " with Baud Rate: " << baudrate_ << endl;
		state_machine_ = new state_machine();
	}

	~OptoForceSensor(){
		if (state_machine_->is_open()) {
			state_machine_->close();
		}
		delete state_machine_;
		delete device_name_;
	}


	void init() {

		if (state_machine_->is_open()) {
			state_machine_->close();
		}

		cout << "OptoForcerSensor-init: Serial communication: Device name: " << device_name_ << ", Baud Rate: " << baudrate_ << endl;

		if(!state_machine_->open(device_name_, baudrate_)) {
			cerr << "OptoForceSensor: Error at opening the device in the state machine" << endl;
			std::exit(-1);
		}

		//Configure the sensor:
		state_machine_->writeConfiguration(frequency_sampling_sensor,frequency_filter,flag_raw);

	}

	void update(){
		state_machine_->read(packs, offsets);

		if(packs.size()>0) {
			sp::display_package& current_value = packs.back();
			readings_xyz_raw[0] = current_value.s1-current_value.s3;
			readings_xyz_raw[1] = current_value.s4-current_value.s2;
			readings_xyz_raw[2] = (current_value.s1+current_value.s2+current_value.s3+current_value.s4)/4;

			for(int i=0;i<3;i++){
				readings_xyz_calibrated[i] = (int) ( optoforce_prop_const * (float)(readings_xyz_raw[i] - readings_xyz_offset[i]) );
			}


			if(VERBOSE) {
				const int output_mod_const = 1000;
				static int counter = 0;
				counter++;
				gettimeofday(&time_n, NULL);
				long d_sec, d_usec;
				d_sec  = time_n.tv_sec  - time_n_minus_one.tv_sec; //So far unused
				d_usec = time_n.tv_usec - time_n_minus_one.tv_usec;
				gettimeofday(&time_n_minus_one, NULL);

				if(counter % output_mod_const == 0)  {
					for(unsigned int i=0;i< packs.size();i++) {
						printf("Pack: %i, delta time us: %i \t Data raw: !%5i,%5i,%5i$\n", i, d_usec, readings_xyz_raw[0], readings_xyz_raw[1], readings_xyz_raw[2]);
						printf("Pack: %i, delta time us: %i \t Data cal: !%5i,%5i,%5i$\n", i, d_usec, readings_xyz_calibrated[0],readings_xyz_calibrated[1],readings_xyz_calibrated[2]);
					} //End print for all packs
				} //End if counter modulo
			}//End if verbose
			packs.clear();
		}//End if packs > 0
	}


	//Stores offsets and removes the offset from the sensor values
	void storeSensorOffset(std::string filename,unsigned int nr_elements_for_filter){



		//Simple avg filter:
		static int offsets[3];


		for(int i=0;i<3;i++){ readings_xyz_offset[i]=0; } //Set the actual offset to zero //ToDo: Ev. protection with semaphores

		for(int j=0;j<nr_elements_for_filter;++j) {

			cout << "OptoForceSensor> Storing value nr " << j << endl;

						if (!state_machine_->is_open()) {
							init();
						}
			update();

			//Don't count the initial readings, when only zeros are transmitted:
			if( (readings_xyz_raw[0]==0) &  (readings_xyz_raw[1]==0) & (readings_xyz_raw[2]==0) ) {
				j--;
				continue;
			}

			for(int i=0;i<3;i++){
				readings_xyz_offset[i] += readings_xyz_raw[i];
			}

			//Let the sensor produce data (Could be adapted according to sampling rate of the sensor)
			usleep(5000);
		} //End for all the sensor samples

		for(int i=0;i<3;i++) { readings_xyz_offset[i] = (int) ( (float) readings_xyz_offset[i] / (float) nr_elements_for_filter); }

		//Store the offsets in a file
		FILE * myfile = fopen( filename.c_str(), "w" );

		if (myfile!=NULL)
		{
			fprintf(myfile, "%5i, %5i, %5i\n", readings_xyz_offset[0],readings_xyz_offset[1],readings_xyz_offset[2]);
			fclose (myfile);
			std::cout << "Stored the sensor offset in file: " << filename << std::endl;
		}
		else {
			std:cerr << "OptoForceSensor: Error at writing to file " << filename << std::endl;
		}
	}

	//Load offsets from file and removes the offset from the sensor values
	void loadSensorOffset(std::string filename){

		cout << "Loading the sensor offset" << endl;

		FILE * myfile = fopen( filename.c_str(), "r" );

		if (myfile!=NULL)
		{
			rewind(myfile);
			fscanf(myfile, "%i, %i, %i", readings_xyz_offset,readings_xyz_offset+1,readings_xyz_offset+2);
			fclose (myfile);
		}
		else {
			std:cerr << "OptoForceSensor: Error at reading the bias from file " << filename << std::endl;
		}

	}




	int * returnXYZRaw(void){return readings_xyz_raw;}
	int * returnXYZCalibrated(void){return readings_xyz_calibrated;}

	void exit(){
		if (state_machine_->is_open()) {
			state_machine_->close();
		}
	}

private:

	int baudrate_;
	char * device_name_;



	int readings_xyz_raw[3];
	int readings_xyz_calibrated[3];
	int readings_xyz_offset[3];

	state_machine * state_machine_;
	std::deque<sp::display_package> packs;
	sp::display_package offsets;
	struct timeval time_n_minus_one, time_n;


};


#endif /* OptoForceSensor_HPP_ */
