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
    ("timeout,t",
      boost::program_options::value<unsigned int>()->default_value(25),
      "device timeout in [ms]")
    ("speed,s",
      boost::program_options::value<unsigned int>(),
      "new device speed in [Hz]")
    ("filter,f",
      boost::program_options::value<unsigned int>(),
      "new device filter frequency in [Hz]")
    ("mode,m",
      boost::program_options::value<std::string>(),
      "new device mode [raw|force]")
    ("help,h", "produce help message");
  positional.add("device", -1);
  
  boost::program_options::store(
    boost::program_options::command_line_parser(argc, argv).options(
      description).positional(positional).run(), variables);

  if (variables.count("help")) {
    std::cout <<
      "Usage: optoforce-configure [options] dev1 [dev2 [...]]\n"
      "\n"
      "Read and write the configuration parameters of all OptoForce sensors\n"
      "connected at the specified serial devices (usually /dev/ttyACM*).\n" 
      "\n" <<
      description;
    return 0;
  }
  
  boost::program_options::notify(variables);    
  std::vector<std::string> devices =
    variables["device"].as<std::vector<std::string> >();
  unsigned int baudrate = variables["baudrate"].as<unsigned int>();
  unsigned int timeout = variables["timeout"].as<unsigned int>();
  
  boost::ptr_vector<Sensor> sensors;
  for (int i = 0; i < devices.size(); ++i) {
    Sensor* sensor = new Sensor();
    sensors.push_back(sensor);
  }

  for (int i = 0; i < sensors.size(); i++) {
    try {
      sensors[i].connect(devices[i], baudrate);
    }
    catch (boost::exception& exception) {}
  }
  
  fprintf(stdout,
    "%-20s  %-12s  %-10s  %15s  %15s\n",
    "Device", "State", "Mode", "Speed in [Hz]", "Filter in [Hz]");
    
  for (int i = 0; i < sensors.size(); i++) {
    if (!sensors[i].hasPackages())
      usleep(1e3*timeout);
    
    if (sensors[i].hasPackages()) {
      SensorPackage package = sensors[i].getPackage();
      SensorConfig config = package.getConfig();
      
      if (variables.count("speed"))
        config.setSpeedHz(variables["speed"].as<unsigned int>());
      if (variables.count("filter"))
        config.setFilterHz(variables["filter"].as<unsigned int>());
      if (variables.count("mode")) {
        std::string mode = variables["mode"].as<std::string>();
        
        if (mode == "raw")
          config.setMode(SensorConfig::mode_raw);
        else if (mode == "force")
          config.setMode(SensorConfig::mode_force);
      }
      
      sensors[i].configure(config);
      sensors[i].clearPackages();
    }
    
    if (!sensors[i].hasPackages())
      usleep(1e3*timeout);
    
    if (sensors[i].hasPackages()) {
      std::string state = "N/A";
      std::string mode = "N/A";
      
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
      
      switch (package.getConfig().getMode()) {
        case SensorConfig::mode_force:
          mode = "FORCE";
          break;
        default:
          mode = "RAW";
      }
      
      fprintf(stdout,
        "%-20s  %-12s  %-10s  %15d  %15d\n",
        sensors[i].getDeviceFilename().c_str(),
        state.c_str(),
        mode.c_str(),
        package.getConfig().getSpeedHz(),
        package.getConfig().getFilterHz());
    }
    else
      fprintf(stdout,
        "%-20s  %-12s  %-10s  %15s  %15s\n",
        sensors[i].getDeviceFilename().c_str(),
        "N/A", "N/A", "N/A", "N/A");
  }
  
  return 0;
}
