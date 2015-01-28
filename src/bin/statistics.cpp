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
#include <cstdint>

#include <optoforce/Sensor.hpp>

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/program_options.hpp>
#include <boost/chrono.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

using namespace optoforce;

typedef boost::accumulators::accumulator_set<double,
  boost::accumulators::stats<boost::accumulators::tag::rolling_mean> >
  Accumulator;

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
    ("window,w",
      boost::program_options::value<unsigned int>()->default_value(20),
      "size of the accumulator window")
    ("persistent,p", "maintain persistent connection")
    ("single-line,s", "output on single line(s)")
    ("help,h", "produce help message");
  positional.add("device", -1);
  
  boost::program_options::store(
    boost::program_options::command_line_parser(argc, argv).options(
      description).positional(positional).run(), variables);

  if (variables.count("help")) {
    std::cout <<
      "Usage: optoforce-statistics [options] dev1 [dev2 [...]]\n"
      "\n"
      "Accumulate communication statistics for all OptoForce sensors\n"
      "connected at the specified serial devices (usually /dev/ttyACM*)\n"
      "and print out the results.\n" 
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
  unsigned int window = variables["window"].as<unsigned int>();
  bool persistent = variables.count("persistent");
  bool singleLine = variables.count("single-line");
  
  boost::ptr_vector<Sensor> sensors;
  for (int i = 0; i < devices.size(); ++i) {
    Sensor* sensor = new Sensor(buffer);
    sensors.push_back(sensor);
  }

  std::vector<size_t> deviceNumRead(sensors.size());
  std::vector<size_t> numReceivedPackages(sensors.size());
  std::vector<int64_t> time(sensors.size());
  std::vector<Accumulator> bitRateAccumulator(sensors.size(), Accumulator(
    boost::accumulators::tag::rolling_window::window_size = window));
  std::vector<Accumulator> pkgRateAccumulator(sensors.size(), Accumulator(
    boost::accumulators::tag::rolling_window::window_size = window));
  
  for (int i = 0; i < sensors.size(); i++) {
    try {
      sensors[i].connect(devices[i], baudrate);
      
      deviceNumRead[i] = sensors[i].getDeviceNumRead();
      numReceivedPackages[i] = sensors[i].getNumReceivedPackages();
      time[i] = now();
    }
    catch (boost::exception& exception) {}
  }
  
  fprintf(stdout,
    "%-20s  %-12s  %10s  %10s  %10s  %10s  %10s\n",
    "Device", "State",
    "Bytes Recv", "Bit Rate",
    "Pkgs Recv", "Pkgs Drop", "Pkg Rate");
  
  while (true) {
    int64_t then = now();
    
    for (int i = 0; i < sensors.size(); i++) {
      SensorReading reading(NAN, NAN, NAN);
      std::string state = "N/A";
      
      if (!sensors[i].isConnected() && persistent) {
        try {
          sensors[i].connect(devices[i], baudrate);
          
          deviceNumRead[i] = sensors[i].getDeviceNumRead();
          numReceivedPackages[i] = sensors[i].getNumReceivedPackages();
          time[i] = now();
        }
        catch (boost::exception& exception) {}
      }
      
      if (sensors[i].isConnected()) {
        if (sensors[i].hasPackages()) {
          SensorPackage package = sensors[i].dequeuePackage();
          
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
        }        
      }
      
      double dt = 1e-9*(now()-time[i]);
      bitRateAccumulator[i](
        8.0*(sensors[i].getDeviceNumRead()-deviceNumRead[i])/dt);
      pkgRateAccumulator[i](
        (sensors[i].getNumReceivedPackages()-numReceivedPackages[i])/dt);
      
      deviceNumRead[i] = sensors[i].getDeviceNumRead();
      numReceivedPackages[i] = sensors[i].getNumReceivedPackages();
      time[i] = now();
      
      fprintf(stdout,
        "%s%-20s  %-12s  %10d  %10.2f  %10d  %10d  %10.2f\n",
        singleLine ? "\r" : "",
        sensors[i].getDeviceFilename().c_str(),
        state.c_str(),
        (unsigned int)sensors[i].getDeviceNumRead(),
        boost::accumulators::rolling_mean(bitRateAccumulator[i]),
        (unsigned int)sensors[i].getNumReceivedPackages(),
        (unsigned int)sensors[i].getNumDroppedPackages(),
        boost::accumulators::rolling_mean(pkgRateAccumulator[i]));
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
