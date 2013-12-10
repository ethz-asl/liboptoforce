
#include "state_machine_wo_qt.h"

#include <stdint.h>
#include <stdio.h>
#include <deque>
#include <iostream>



//Boost
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>


using namespace std;


state_machine::state_machine ()
: c(0)
, state(1)
, first_valid(false)
, vs(_66)
{
	baudrate_ = 115200;  //Default baudrate
	device_ = "unknown";
	serialDevice_ = NULL;
}

/* try open a port with portname */
bool state_machine::open (string device, int baudrate){ //const QString& portname) {
	first_valid = false;
	device_ = device;
	baudrate_ = baudrate;
	cout << "state_machine: Opeing port " << device_ << " with baudrate: " << baudrate_ << endl;

	if(serialDevice_!=NULL) {
		cout << "state_machine: serialDevice is not NULL - deleting the device  " << endl;
		delete serialDevice_; }

	try
	{
		serialDevice_ = new SerialDevice(io_service_, baudrate_, device_);
	}
	catch(boost::exception const&  ex)
	{
		//ToDo: Output more information about the error
		std::cout << "Error at generating a serial Device" << std::endl;
		serialDevice_ = NULL;
		std::exit(-1);
	}

	boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service_));

	return true;
}

bool state_machine::is_open () {
	if(serialDevice_!=NULL){
		return serialDevice_->is_open();
	}
	else return 0;
}

void state_machine::close () {
	first_valid = false;
	serialDevice_->close();
}

//Added by markho@ethz.ch to allow for configuration of the sensor
// possible values: frequency_hz_sampling: 1000, 333, 100, 10
//  frequency_hz_filter:  0 (no filter - default), 150, 50, 15
// Raw == 1: Raw data streaming (default), Raw == 0: Force (not working?) 
int state_machine::writeConfiguration(int frequency_hz_sampling, int frequency_hz_filter, int flag_raw){

	//Initialization of the config_value
	char config_value = 0b10100000; //Bit 7 and bit 5 are always set to 1

	//Frequency:
	switch(frequency_hz_sampling){
	case 1000:
		//Here bit 4 and 3 are 0
		break;
	case 333:
		config_value |= 1 << 3;  //Bit 3 is 1
		break;
	case 100:
		config_value |= 1 << 4;  //Bit 4 is 1
	case 30:
		config_value |= (1 << 3) | (1 << 4);  //Bit 3 and 4 are 1
	default:
		break;
	}  //End switch frequency

	switch(frequency_hz_filter){
	case 0: //No filter:
		//Here bit 2 and 1 are 0
		break;
	case 150:
		config_value |= 1 << 1; //Here bit 1 is 1
		break;
	case 50:
		config_value |= 1 << 2;  //Bit 2 is 1
		break;
	case 15:
		config_value |= (1 << 2) | (1 << 1);  //Bits 1 and 2 set to 1
	default:
		//No filter:
		break;
	} //End switch filter

	switch(flag_raw) {
	case 1:
		//Bit 0 remains 0
		break;
	case 0:
		config_value |= 1;
		break;
	default:
		//Bit 0 remains 0
		break;
	} //End switch raw

	const char msg = config_value;

	try
	{
		if(serialDevice_!=NULL){
			serialDevice_->write(msg);
		}
		else throw;
	}
	catch(boost::exception const&  ex)
	{
		//ToDo: Better error handling
		std::cout << "Error at writing to the serial Device" << std::endl;
		std::exit(-1);
	}

	return 0;
}

/* read and processing all data from input */
void state_machine::read(std::deque<sp::display_package>& packs, const sp::display_package& offset)  {

	uint16_t actual;
	uint8_t checksum;
	sp::display_package pack;

	char * data_in;
	int nr_bytes;

	if(serialDevice_!=NULL){
		nr_bytes = serialDevice_->return_read_data_size();
		if(nr_bytes!=0) {
		data_in = serialDevice_->return_read_data();
		}
	}

	int i;
	for(i = 0; i <  nr_bytes; ++i) {
		c = data_in[i]; // the actual byte in processing
		switch (state) { // branch with actual state in fsm
		case 1:
			if (c == 55) state = 2; // first checker byte
			else         state = 1; // restart this state
			break;
		case 2:
			if (c == 66) { // second checker byte for .66 version
				state = 10; // jump the .66 version reading
				vs = _66;
			} else if (c == 67) { // seconf checker byte for .67 version
				state = 20; // jump the .67 version reading
				vs = _67;
			} else
				state = 1; // restart the fsm
			break;

			/* .66 version package */
		case 10:
			if (!first_valid) { // this is a first valid package
				first_valid = true;
			}
			actual = ((uint16_t)c) * 256;
			state = 11;
			break;
		case 11:
			actual += (uint16_t)c;
			pack.s1 = ((unsigned int)actual)+UINT_MAX+1 - offset.s1;
			state = 12;
			break;
		case 12:
			actual = ((uint16_t)c) * 256;
			state = 13;
			break;
		case 13:
			actual += (uint16_t)c;
			pack.s2 = ((unsigned int)actual)+UINT_MAX+1 - offset.s2;
			state = 14;
			break;
		case 14:
			actual = ((uint16_t)c) * 256;
			state = 15;
			break;
		case 15:
			actual += (uint16_t)c;
			pack.s3 = ((unsigned int)actual)+UINT_MAX+1 - offset.s3;
			pack = 0;
			state = 16;
			break;
		case 16:
			actual = ((uint16_t)c) * 256;
			state = 17;
			break;
		case 17:
			actual += (uint16_t)c;
			pack.s4 = ((unsigned int)actual)+UINT_MAX+1 - offset.s4;
			packs.push_back(pack);
			pack = 0;
			state = 1;
			break;




			/* .67 version package */
		case 20:
			if (!first_valid) { // this is a first valid package
				//   emit first_valid_read(); // first valid event
				first_valid = true;
			}
			state = 21;
			checksum = c;
			break;

		case 21:
			actual = ((uint16_t)c) * 256;
			state = 22;
			break;
		case 22:
			actual += (uint16_t)c;
			pack.s1 = ((unsigned int)actual)+UINT_MAX+1 - offset.s1;
			state = 23;
			checksum += actual;
			break;
		case 23:
			actual = ((uint16_t)c) * 256;
			state = 24;
			break;
		case 24:
			actual += (uint16_t)c;
			pack.s2 = ((unsigned int)actual)+UINT_MAX+1 - offset.s2;
			state = 25;
			checksum += actual;
			break;
		case 25:
			actual = ((uint16_t)c) * 256;
			state = 26;
			break;
		case 26:
			actual += (uint16_t)c;
			pack.s3 = ((unsigned int)actual)+UINT_MAX+1 - offset.s3;
			state = 27;
			checksum += actual;
			break;
		case 27:
			actual = ((uint16_t)c) * 256;
			state = 28;
			break;
		case 28:
			actual += (uint16_t)c;
			pack.s4 = ((unsigned int)actual)+UINT_MAX+1 - offset.s4;
			state = 29;
			checksum += actual;
			break;
		case 29:
			actual = ((uint16_t)c) * 256;
			state = 30;
			break;
		case 30:
			actual += (uint16_t)c;
			pack.temp = ((unsigned int)actual)+UINT_MAX+1;
			state = 31;
			checksum += actual;
			break;
		case 31:
			if (checksum == c) {
				packs.push_back(pack);
			}
			pack = 0;
			state = 1;
			break;
		}
	}
}
