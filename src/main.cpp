
#include"ForceSensorVector.hpp"
#include"ForceSensor.hpp"
#include"OptoForceSensor.hpp"
#include"optoforce_parameters.h"



#include <iostream>


using namespace std; 

int main(int argc, char *argv[])
{
 ForceSensorVector forceSensorVector_;

 ForceSensor<OptoForceSensor> * forceSensorLF_ = new ForceSensor<OptoForceSensor>();
 //ForceSensor<OptoForceSensor> * forceSensorRF_ = new ForceSensor<OptoForceSensor>();

 *forceSensorLF_ = *(new OptoForceSensor(ifaces_USB[3], baudrate));
 //*forceSensorRF_ = *(new OptoForceSensor(ifaces_USB[1], baudrate));

  forceSensorVector_+=forceSensorLF_;  
//  forceSensorVector_+=forceSensorRF_;

 std::cout << "Size of the sensor vector: " << forceSensorVector_.size() << std::endl;

 for(int i=0;i<forceSensorVector_.size();i++){
   forceSensorVector_[i]->init();

   //As an example: Set the force sensor to zero and store the offset:
   //forceSensorVector_[i]->storeSensorOffset("teest",100); //Filename, number of elements for the filter
   //forceSensorVector_[i]->loadSensorOffset("teest"); //Filename, number of elements for the filter

 }



 while(1) {
   usleep(2500);
   for(int i=0;i<forceSensorVector_.size();i++){
     forceSensorVector_[i]->update();
     int * readings_xyz = forceSensorVector_[i]->returnXYZCalibrated();
     cout << "main.cpp: Sensor wo bias " << i << " readings: X: " << readings_xyz[0] << "\tY: " <<  readings_xyz[1] << "\tZ: " <<  readings_xyz[2] << endl;
   } //End for all sensors
 } //End while
} //End main()
