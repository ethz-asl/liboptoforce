#include <iostream>

#include <optoforce/ForceSensorVector.hpp>
#include <optoforce/ForceSensor.hpp>
#include <optoforce/OptoForceSensor.hpp>
#include <optoforce/OptoForceParameters.hpp>

using namespace std; 

int main(int argc, char *argv[]) {
  if (argc < 2) {
    cerr << "usage: optoforce-unbiased DEV1 [DEV2 [DEV3 ...]]" << endl;
    return 1;
  }
  
  ForceSensorVector forceSensorVector_;

  ForceSensor<OptoForceSensor> * forceSensor_ =
    new ForceSensor<OptoForceSensor>();

  for (int i = 1; i < argc; ++i) {
    ForceSensor<OptoForceSensor> * forceSensor_ =
      new ForceSensor<OptoForceSensor>();
    * forceSensor_= *(new OptoForceSensor(argv[i], baudrate));
    forceSensorVector_ += forceSensor_;  
  }

  cout << "Size of the sensor vector: " << forceSensorVector_.size() << endl;

  for (int i = 0; i < forceSensorVector_.size(); i++) {
    forceSensorVector_[i]->init();

    // As an example: Set the force sensor to zero and store the offset:
    // Filename, number of elements for the filter
    //   forceSensorVector_[i]->storeSensorOffset("test", 100);
    // Filename, number of elements for the filter
    //   forceSensorVector_[i]->loadSensorOffset("test"); 
  }

  while (true) {
    usleep(2500);
   
    for (int i = 0; i < forceSensorVector_.size(); i++) {
      forceSensorVector_[i]->update();
      int * readings_xyz = forceSensorVector_[i]->returnXYZCalibrated();
      fprintf(stdout, "Sensor %2d w/o bias: X = %6d  Y = %6d  Z = %6d\n",
        i, readings_xyz[0], readings_xyz[1], readings_xyz[2]);
    } //End for all sensors
  } //End while
  
  return 0;
} //End main()
