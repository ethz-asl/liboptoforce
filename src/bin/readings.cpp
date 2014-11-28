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

inline int64_t now() {
  return boost::chrono::duration_cast<boost::chrono::nanoseconds>(
    boost::chrono::high_resolution_clock::now().time_since_epoch()).count();
};

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
    ("update,u",
      boost::program_options::value<double>()->default_value(20.0),
      "update frequency in [Hz]")
    ("buffer,b",
      boost::program_options::value<unsigned int>()->default_value(32),
      "size of the package buffer")
    ("factor,f",
      boost::program_options::value<double>()->default_value(1e-3),
      "signal-to-force factor")
    ("compensated,c", "display compensated values")
    ("num-samples,n",
      boost::program_options::value<unsigned int>()->default_value(0),
      "number of calibration samples")
    ("persistent,p", "maintain persistent connection")
    ("single-line,s", "output on single line(s)")
    ("help,h", "produce help message");
  positional.add("device", -1);
  
  boost::program_options::store(
    boost::program_options::command_line_parser(argc, argv).options(
      description).positional(positional).run(), variables);

  if (variables.count("help")) {
    std::cout <<
      "Usage: optoforce-unbiased [options] dev1 [dev2 [...]]\n"
      "\n"
      "Acquire unbiased force readings from all OptoForce sensors connected\n"
      "at the specified serial devices (usually /dev/ttyACM*).\n"
      "\n" <<
      description;
    return 0;
  }
  
  boost::program_options::notify(variables);    
  std::vector<std::string> devices =
    variables["device"].as<std::vector<std::string> >();
  unsigned int baudrate = variables["baudrate"].as<unsigned int>();
  double update = variables["update"].as<double>();
  unsigned int buffer = variables["buffer"].as<unsigned int>();
  double factor = variables["factor"].as<double>();
  bool compensated = variables.count("compensated");
  unsigned int numSamples = variables["num-samples"].as<unsigned int>();
  bool persistent = variables.count("persistent");
  bool singleLine = variables.count("single-line");
  
  boost::ptr_vector<Sensor> sensors;
  for (int i = 0; i < devices.size(); ++i) {
    Sensor* sensor = new Sensor(buffer, factor);
    sensors.push_back(sensor);
  }

  for (int i = 0; i < sensors.size(); i++) {
    try {
      sensors[i].connect(devices[i], baudrate);
      if (numSamples)
        sensors[i].calibrate(numSamples);
    }
    catch (boost::exception& exception) {}
  }
  
  fprintf(stdout,
    "%-20s  %-12s  %-10s  %12s  %12s  %12s  %12s  %12s  %12s\n",
    "Device", "State", "Checksum",
    "F_x in [N]", "F_y in [N]", "F_z in [N]",
    "T_x in [Nm]", "T_y in [Nm]", "T_z in [Nm]");
  
  while (true) {
    int64_t then = now();
    
    for (int i = 0; i < sensors.size(); i++) {
      SensorReading reading(NAN, NAN, NAN);
      std::string state = "N/A";
      std::string checksum = "N/A";
      
      if (!sensors[i].isConnected() && persistent) {
        try {
          sensors[i].connect(devices[i], baudrate);
          if (numSamples)
            sensors[i].calibrate(numSamples);
        }
        catch (boost::exception& exception) {}
      }
      
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

          reading = sensors[i].getReading(
            Sensor::buffer_position_newest, compensated);
        }
      }
      
      if (sensors[i].isCalibrating()) {
        fprintf(stdout,
          "%s%-20s  %-12s  %-10s    %75s %3.0f%%\n",
          singleLine ? "\r" : "",
          sensors[i].getDeviceFilename().c_str(),
          state.c_str(),
          checksum.c_str(),
          "Calibrating...",
          1e2*sensors[i].getNumCalibrationReadings()/numSamples);
      }
      else {
        fprintf(stdout,
          "%s%-20s  %-12s  %-10s  %12.3f  %12.3f  %12.3f  "
            "%12.3f  %12.3f  %12.3f\n",
          singleLine ? "\r" : "",
          sensors[i].getDeviceFilename().c_str(),
          state.c_str(),
          checksum.c_str(),
          reading.getForceX(), reading.getForceY(), reading.getForceZ(),
          reading.getTorqueX(), reading.getTorqueY(), reading.getTorqueZ());
      }
    }

    if (singleLine)
      fprintf(stdout, "%c[%dA\r", 0x1B, (int)sensors.size());
    
    int64_t sleep = 1e9/update-(now()-then);
    if (sleep > 0)
      usleep(1e-3*sleep);
  }
  
  if (singleLine)
    fprintf(stdout, "%c[%dB", 0x1B, (int)sensors.size());
  
  return 0;
}
