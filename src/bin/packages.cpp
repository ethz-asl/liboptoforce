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
    ("frequency,f",
      boost::program_options::value<double>()->default_value(20.0),
      "update frequency in [Hz]")
    ("buffer,b",
      boost::program_options::value<unsigned int>()->default_value(32),
      "size of the package buffer")
    ("persistent,p", "maintain persistent connection")
    ("single-line,s", "output on single line(s)")
    ("help,h", "produce help message");
  positional.add("device", -1);
  
  boost::program_options::store(
    boost::program_options::command_line_parser(argc, argv).options(
      description).positional(positional).run(), variables);

  if (variables.count("help")) {
    std::cout <<
      "Usage: optoforce-packages [options] dev1 [dev2 [...]]\n"
      "\n"
      "Acquire packages from all OptoForce sensors connected at the\n"
      "specified serial devices (usually /dev/ttyACM*) and print out\n"
      "their content for inspection.\n"
      "\n" <<
      description;
    return 0;
  }
  
  boost::program_options::notify(variables);    
  std::vector<std::string> devices =
    variables["device"].as<std::vector<std::string> >();
  unsigned int baudrate = variables["baudrate"].as<unsigned int>();
  double frequency = variables["frequency"].as<double>();
  unsigned int buffer = variables["buffer"].as<unsigned int>();
  bool persistent = variables.count("persistent");
  bool singleLine = variables.count("single-line");
  
  boost::ptr_vector<Sensor> sensors;
  for (int i = 0; i < devices.size(); ++i) {
    Sensor* sensor = new Sensor(buffer);
    sensors.push_back(sensor);
  }

  for (int i = 0; i < sensors.size(); i++) {
    try {
      sensors[i].connect(devices[i], baudrate);
    }
    catch (boost::exception& exception) {}
  }
  
  fprintf(stdout,
    "%-20s  %-12s  %-10s  %-10s  %s\n",
    "Device", "State", "Checksum", "Version", "Content");
  
  while (true) {
    int64_t then = now();
    
    for (int i = 0; i < sensors.size(); i++) {
      SensorReading reading(NAN, NAN, NAN);
      std::string state = "N/A";
      std::string checksum = "N/A";
      std::string version = "N/A";
      std::string content;
      
      if (!sensors[i].isConnected() && persistent) {
        try {
          sensors[i].connect(devices[i], baudrate);
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
          
          switch (package.getVersion()) {
            case SensorPackage::version_66:
              version = "66";
              break;
            case SensorPackage::version_67:
              version = "67";
              break;
            case SensorPackage::version_68:
              version = "68";
              break;
            case SensorPackage::version_94:
              version = "94";
              break;
            default:
              version = "NONE";
          }
          
          const std::vector<int>& rawSignals =
            package.getRawSignals();
          const std::vector<int>& compensatedSignals =
            package.getCompensatedSignals();
          const std::vector<int>& force = package.getForce();
          const std::vector<int>& temperature = package.getTemperature();
          
          for (unsigned int j = 0; j < rawSignals.size(); ++j) {
            char string[128];
            sprintf(string, "%sS%d=%-6d", content.empty() ? "" : "  ", j+1,
              rawSignals[j]);
            
            content += string;
          }
          
          for (unsigned int j = 0; j < compensatedSignals.size(); ++j) {
            char string[128];
            sprintf(string, "%scS%d=%-6d", content.empty() ? "" : "  ", j+1,
              compensatedSignals[j]);
            
            content += string;
          }
          
          for (unsigned int j = 0; j < force.size(); ++j) {
            char string[128];
            sprintf(string, "%sF%d=%-6d", content.empty() ? "" : "  ", j+1,
              force[j]);
            
            content += string;
          }
          
          for (unsigned int j = 0; j < temperature.size(); ++j) {
            char string[128];
            sprintf(string, "%sT%d=%-6d", content.empty() ? "" : "  ", j+1,
              temperature[j]);
            
            content += string;
          }
        }
      }
      
      fprintf(stdout,
        "%s%-20s  %-12s  %-10s  %-10s  %s\n",
        singleLine ? "\r" : "",
        sensors[i].getDeviceFilename().c_str(),
        state.c_str(),
        checksum.c_str(),
        version.c_str(),
        content.empty() ? "N/A" : content.c_str());
    }

    if (singleLine)
      fprintf(stdout, "%c[%dA\r", 0x1B, (int)sensors.size());
    
    int64_t sleep = 1e9/frequency-(now()-then);
    if (sleep > 0)
      usleep(1e-3*sleep);
  }
  
  if (singleLine)
    fprintf(stdout, "%c[%dB", 0x1B, (int)sensors.size());
  
  return 0;
}
