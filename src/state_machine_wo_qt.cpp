
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


	//Depricated
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
	uint16_t checksum2;
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

	

	//States, compared to new version:
	//  Old:   New:
	//  1      first_check
	//  2      second_check
	// 10     _66_Start


	for(i = 0; i <  nr_bytes; ++i) {
		c = data_in[i]; // the actual byte in processing

		switch (state) { // branch with actual state in fsm
	

		case first_check: //1:
			if (c == 55) state = second_check; // first checker byte
			else         state = first_check; // restart this state
			break;
		case second_check:
			if (c == 66) { // second checker byte for .66 version
			  state = _66_Start; //v<1.1: 10; // jump the .66 version reading
				vs = _66;

				// pack[0].vs=_66;
				// virtualSensor=false;
				// sensorSize=1;
			} else if (c == 67) { // seconf checker byte for .67 version
				state = _67_Start; // jump the .67 version reading



				vs = _67;
			}  else if (c == 68) { // seconf checker byte for .67 version
				state = _68_Start; // jump the .67 version reading
				vs = _68;

				} 
			else
				state = 1; // restart the fsm
			break;

			/* .66 version package */
			/*
		case _66_Start: //10:
			if (!first_valid) { // this is a first valid package
				first_valid = true;
			}
			state =  state = _66_S1H;

		case _66_S1H:
			actual = ((uint16_t)c) * 256; // read a one byte - read the high part of first data
			state = _66_S1L; //11; next state
			break;
		case _66_S1L: //11:
	

			actual += (uint16_t)c; // read a one byte - read the low part of first data - and add to high part
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


			*/

			/* .67 version package */
			
		case  _67_Start: //20:
			if (!first_valid) { // this is a first valid package
				//   emit first_valid_read(); // first valid event
				first_valid = true;
			}
			state = _67_Config;
		case _67_Config:	
			//			state = 21;
			checksum = c;
			state = _67_I1H; //state = 21;
			break;

		case _67_I1H:  //21:
			actual = ((uint16_t)c) * 256;
			state = _67_I1L; // next state... //22;
			break;
		case  _67_I1L: //22:
			actual += (uint16_t)c;
			pack.s1 = ((unsigned int)actual)+UINT_MAX+1 - offset.s1;
			state = _67_I2H; //23;
			checksum += actual;
			break;
		case _67_I2H:  //23:
			actual = ((uint16_t)c) * 256;
			state = _67_I2L; //24;
			break;
		case _67_I2L: //24:
			actual += (uint16_t)c;
			pack.s2 = ((unsigned int)actual)+UINT_MAX+1 - offset.s2;
			state =  _67_I3H; //25;
			checksum += actual;
			break;
		case  _67_I3H: //25:
			actual = ((uint16_t)c) * 256;
			state = _67_I3L; // 26;
			break;
		case _67_I3L://26:
			actual += (uint16_t)c;
			pack.s3 = ((unsigned int)actual)+UINT_MAX+1 - offset.s3;
			state =  _67_I4H; //27;
			checksum += actual;
			break;
		case _67_I4H: //27:
			actual = ((uint16_t)c) * 256;
			state = _67_I4L; //28;
			break;
		case _67_I4L: //28:
			actual += (uint16_t)c;
			pack.s4 = ((unsigned int)actual)+UINT_MAX+1 - offset.s4;
			state =  _67_TH; //29;
			checksum += actual;
			break;
		case  _67_TH: //29:
			actual = ((uint16_t)c) * 256;
			state = _67_TL; //30;
			break;
		case _67_TL: //30:
			actual += (uint16_t)c;
			pack.temp = ((unsigned int)actual)+UINT_MAX+1;
			state = _67_Checksum;; //31;
			checksum += actual;
			break;
		case _67_Checksum: //31:
			if (checksum == c) {
				packs.push_back(pack);
			}
			pack = 0;
			state =  _67_Start; //1;
			break;

			

 /* .68 version package */
			// New package that was added in API Version 1.1
            case _68_Start:

	      //   cout << "Starting to decode 68er pack" << endl;

                if (!first_valid) {
                    first_valid = true;
		    //                   first_valid_state = true;
		    //   sensors.append(OptoSensor());
                }
                state = _68_Config;
            case _68_Config:
	      //    last_conf = SensorConfig::from_uint8_t(c);
              //  pack[0].config=last_conf;
                checksum2 = c;
                state = _68_FXH;
                break;
            case _68_FXH:
                actual = ((uint16_t)c) * 256;
		checksum2 += c;
                state = _68_FXL;
                break;
            case _68_FXL:
                actual += (uint16_t)c;
		pack.x = ((int16_t)actual);
		//        pack[0].x = ((int16_t)actual);
                checksum2 += c;
                state = _68_FYH;
                break;
            case _68_FYH:
                actual = ((uint16_t)c) * 256;
                checksum2 += c;
                state = _68_FYL;
                break;
            case _68_FYL:
                actual += (uint16_t)c;
		  pack.y = ((int16_t)actual);
		//       pack[0].y = ((int16_t)actual);
                checksum2 += c;
                state = _68_FZH;
                break;
            case _68_FZH:
                actual = ((uint16_t)c) * 256;
                checksum2 += c;
                state = _68_FZL;
                break;
            case _68_FZL:
                actual += (uint16_t)c;
		 pack.z = ((unsigned int)actual)+UINT_MAX+1;
		//        pack[0].z = ((unsigned int)actual)+UINT_MAX+1;
                state = _68_TH;
                checksum2 += c;
                break;

            case _68_TH:
                actual = ((uint16_t)c) * 256;
                checksum2 += c;
                state = _68_TL;
                break;
            case _68_TL:
                actual += (uint16_t)c;
		//      pack[0].temp=(((unsigned int)actual)+UINT_MAX+1);
                checksum2 += c;
                state = _68_S1H;
                break;

            case _68_S1H:
                actual = ((uint16_t)c) * 256;
                checksum2 += (uint16_t)c;
                state = _68_S1L;
                break;
            case _68_S1L:
                actual += (uint16_t)c;
		
		pack.s1 = ((unsigned int)actual)+UINT_MAX+1 - offset.s1;
		//	cout << "pack.s1: " << pack.s1 << endl;
		//       pack[0].s1 = ((unsigned int)actual)+UINT_MAX+1;
                checksum2 += c;
                state = _68_S2H;
                break;
            case _68_S2H:
                actual = ((uint16_t)c) * 256;
                checksum2 += c;
                state = _68_S2L;
                break;
            case _68_S2L:
                actual += (uint16_t)c;
		pack.s2 = (((unsigned int)actual)+UINT_MAX+1) - offset.s2; 
		//       pack[0].s2=(((unsigned int)actual)+UINT_MAX+1);
                checksum2 += c;
                state = _68_S3H;
                break;
            case _68_S3H:
                actual = ((uint16_t)c) * 256;
                checksum2 += c;
                state = _68_S3L;
                break;
            case _68_S3L:
                actual += (uint16_t)c;
		pack.s3 = (((unsigned int)actual)+UINT_MAX+1) - offset.s3; 
		//        pack[0].s3=(((unsigned int)actual)+UINT_MAX+1);
                checksum2 += c;
                state = _68_S4H;
                break;
            case _68_S4H:
                actual = ((uint16_t)c) * 256;
                checksum2 += c;
                state = _68_S4L;
                break;
            case _68_S4L:
                actual += (uint16_t)c;
		pack.s4 = (((unsigned int)actual)+UINT_MAX+1) - offset.s4; 
		//        pack[0].s4=(((unsigned int)actual)+UINT_MAX+1);
		packs.push_back(pack);
		//			cout << "test2" << endl;
		pack = 0;

                checksum2 += c;
                state = _68_S1TH;
                break;
            case _68_S1TH:
                actual = ((uint16_t)c) * 256;
                checksum2 += c;
                state = _68_S1TL;
                break;
            case _68_S1TL:
                actual += (uint16_t)c;
		pack.s1c=(((unsigned int)actual)+UINT_MAX+1);	
		//        pack[0].s1c=(((unsigned int)actual)+UINT_MAX+1);
                checksum2 += c;
                state = _68_S2TH;
                break;
            case _68_S2TH:
                actual = ((uint16_t)c) * 256;
                checksum2 += c;
                state = _68_S2TL;
                break;
            case _68_S2TL:
                actual += (uint16_t)c;
	
		  pack.s2c=(((unsigned int)actual)+UINT_MAX+1);
		//      pack[0].s2c=(((unsigned int)actual)+UINT_MAX+1);
                checksum2 += c;
                state = _68_S3TH;
                break;
            case _68_S3TH:
                actual = ((uint16_t)c) * 256;
                checksum2 += c;
                state = _68_S3TL;
                break;
            case _68_S3TL:
                actual += (uint16_t)c;
		pack.s3c=(((unsigned int)actual)+UINT_MAX+1);
		//        pack[0].s3c=(((unsigned int)actual)+UINT_MAX+1);
                checksum2 += c;
                state = _68_S4TH;
                break;
            case _68_S4TH:
                actual = ((uint16_t)c) * 256;
                checksum2 += c;
                state = _68_S4TL;
                break;
            case _68_S4TL:
                actual += (uint16_t)c;
		 pack.s4c=(((unsigned int)actual)+UINT_MAX+1);
		//        pack[0].s4c=(((unsigned int)actual)+UINT_MAX+1);
                checksum2 += c;
                state = _68_ChecksumH;
                break;
            case _68_ChecksumH:
                actual = ((uint16_t)c) * 256;
                state = _68_ChecksumL;
                break;
            case _68_ChecksumL:
                actual += (uint16_t)c;
                //qDebug()<<"chk"<<checksum2<<"chkact"<<actual;


		//ToDo: Add this check
		/*
                if (checksum2 == actual) { // check the checksum
		  //          pack[0].saveInconsistent();
		  //           pack[0].setInvariant(sensors[0].offset); // invariant calculate with offset
		  //           packs[0].push_back(pack[0]);
		  if (!checksum_state) {
		    checksum_state = true;
		  }
                } else {
		  if (checksum_state) {
		    checksum_state = false;
		  }
                }
		*/


		//                pack[0] = 0;
                state = first_check; // restart the fsm
                break;







		}
	}
}
