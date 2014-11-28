/******************************************************************************
 * Copyright (C) 2014 by Ralf Kaestner                                        *
 * ralf.kaestner@gmail.com                                                    *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include <vector>
#include <string>
#include <iostream>

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/program_options.hpp>
#include <boost/chrono.hpp>

#include <optoforce/Sensor.hpp>

using namespace optoforce;

int main(int argc, char *argv[]) {
  boost::program_options::options_description description(
    "Supported options");
  boost::program_options::variables_map variables;
  boost::program_options::positional_options_description positional;

  description.add_options()
    ("device,d",
      boost::program_options::value<std::vector<std::string> >()->required(),
      "serial device filenames")
    ("baudrate,b",
      boost::program_options::value<unsigned int>()->default_value(115200),
      "common device baudrate in [Bd]")
    ("buffer,b",
      boost::program_options::value<unsigned int>()->default_value(32),
      "size of the package buffer")
    ("factor,f",
      boost::program_options::value<double>()->default_value(1e-3),
      "signal-to-force factor")
    ("compensated,c", "display compensated values")
    ("num-samples,n",
      boost::program_options::value<unsigned int>()->default_value(1000),
      "number of calibration samples")
    ("help,h", "produce help message");
  positional.add("device", -1);
  
  boost::program_options::store(
    boost::program_options::command_line_parser(argc, argv).options(
      description).positional(positional).run(), variables);

  if (variables.count("help")) {
    std::cout <<
      "Usage: optoforce-unbiased [options] dev1 [dev2 [...]]\n"
      "\n"
      "Perform zero weight calibration of all OptoForce sensors connected at\n"
      "the specified serial devices (usually /dev/ttyACM*) and print out the\n"
      "determined force and torque offsets.\n"
      "\n" <<
      description;
    return 0;
  }
  
  boost::program_options::notify(variables);    
  std::vector<std::string> devices =
    variables["device"].as<std::vector<std::string> >();
  unsigned int baudrate = variables["baudrate"].as<unsigned int>();
  unsigned int buffer = variables["buffer"].as<unsigned int>();
  double factor = variables["factor"].as<double>();
  bool compensated = variables.count("compensated");
  unsigned int numSamples = variables["num-samples"].as<unsigned int>();
  
  boost::ptr_vector<Sensor> sensors;
  for (int i = 0; i < devices.size(); ++i) {
    Sensor* sensor = new Sensor(buffer, factor);
    sensors.push_back(sensor);
  }

  for (int i = 0; i < sensors.size(); i++) {
    try {
      sensors[i].connect(devices[i], baudrate);
      sensors[i].calibrate(numSamples);
    }
    catch (boost::exception& exception) {}
  }
  
  fprintf(stdout,
    "%-20s  %-12s  %-10s  %10s  %12s  %12s  %12s  %12s  %12s  %12s\n",
    "Device", "State", "Checksum", "Progress", 
    "F0_x in [N]", "F0_y in [N]", "F0_z in [N]",
    "T0_x in [Nm]", "T0_y in [Nm]", "T0_z in [Nm]");
  bool complete = false;
  
  while (!complete) {
    complete = true;
    
    for (int i = 0; i < sensors.size(); i++) {
      SensorReading offset(NAN, NAN, NAN);
      std::string state = "N/A";
      std::string checksum = "N/A";
      
      if (sensors[i].isConnected()) {
        if (sensors[i].hasPackages()) {
          SensorPackage package = sensors[i].getPackage();
          
          switch (package.getConfig().getState()) {
            case SensorConfig::state_overload_x:
              state = "OVERLOAD X";
              break;
            case SensorConfig::state_overload_y:
              state = "OVERLOAD Y";
              break;
            case SensorConfig::state_overload_z:
              state = "OVERLOAD Z";
              break;
            case SensorConfig::state_sensor_failure:
              state = "SENSOR FAIL";
              break;
            case SensorConfig::state_sensor_ok:
              state = "OK";
              break;
            case SensorConfig::state_connection_failure:
              state = "CONN FAIL";
              break;
            default:
              state = "NO SENSOR";
          }
          
          switch (package.getChecksum()) {
            case SensorPackage::checksum_okay:
              checksum = "OK";
              break;
            case SensorPackage::checksum_error:
              checksum = "ERROR";
              break;
            default:
              checksum = "NONE";
          }
        }        
      }
      
      if (sensors[i].isCalibrating()) {
        complete = false;
        
        fprintf(stdout,
          "\r%-20s  %-12s  %-10s  %9.0f%%  %12.3f  %12.3f  %12.3f  "
            "%12.3f  %12.3f  %12.3f\n",
          sensors[i].getDeviceFilename().c_str(),
          state.c_str(),
          checksum.c_str(),
          1e2*sensors[i].getNumCalibrationReadings()/numSamples,
          offset.getForceX(), offset.getForceY(), offset.getForceZ(),
          offset.getTorqueX(), offset.getTorqueY(), offset.getTorqueZ());
      }
      else {
        if (sensors[i].isConnected()) {
          if (compensated)
            offset = sensors[i].getCompensatedZeroWeightOffset();
          else
            offset = sensors[i].getZeroWeightOffset();
        }
        
        fprintf(stdout,
          "\r%-20s  %-12s  %-10s  %10s  %12.3f  %12.3f  %12.3f  "
            "%12.3f  %12.3f  %12.3f\n",
          sensors[i].getDeviceFilename().c_str(),
          state.c_str(),
          checksum.c_str(),
          sensors[i].isConnected() ? "100%" : "N/A",
          offset.getForceX(), offset.getForceY(), offset.getForceZ(),
          offset.getTorqueX(), offset.getTorqueY(), offset.getTorqueZ());
      }
    }
      
    fprintf(stdout, "%c[%dA\r", 0x1B, (int)sensors.size());
  }
  
  fprintf(stdout, "%c[%dB", 0x1B, (int)sensors.size());
  
  return 0;
}
