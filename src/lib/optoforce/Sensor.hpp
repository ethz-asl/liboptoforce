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

#ifndef OPTOFORCE_SENSOR_HPP
#define OPTOFORCE_SENSOR_HPP

#include <deque>
#include <cstdint>
#include <exception>

#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/exception/exception.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>

#include <optoforce/SerialDevice.hpp>
#include <optoforce/StateMachine.hpp>
#include <optoforce/SensorPackage.hpp>
#include <optoforce/SensorReading.hpp>

namespace optoforce {
  class Sensor {
  public:
    enum BufferPosition {
      buffer_position_oldest,
      buffer_position_newest
    };
    
    typedef boost::accumulators::accumulator_set<SensorReading,
      boost::accumulators::stats<boost::accumulators::tag::mean> >
      Accumulator;
  
    /* Sensor operation error */
    struct NoSuchPackageError : virtual std::exception,
      virtual boost::exception {};
    
    Sensor(size_t packageBufferSize = 32, double signalToForceFactor = 1e-3);
    ~Sensor();

    inline const std::string& getDeviceFilename() const {
      return serialDevice.getFilename();
    };
    
    inline size_t getDeviceNumRead() const {
      return serialDevice.getNumRead();
    };

    inline size_t getDeviceNumWritten() const {
      return serialDevice.getNumWritten();
    };
    
    SensorPackage getPackage(BufferPosition position = 
      buffer_position_newest) const;
    SensorReading getReading(BufferPosition position = 
      buffer_position_newest, bool compensated = false) const;
    
    inline void setSignalToForceFactor(double signalToForceFactor) {
      boost::lock_guard<boost::mutex> lock(mutex);
      this->signalToForceFactor = signalToForceFactor;
    };
      
    inline double getSignalToForceFactor() {
      return signalToForceFactor;
    };
    
    inline void setZeroWeightOffset(const SensorReading& zeroWeightOffset) {
      boost::lock_guard<boost::mutex> lock(mutex);
      this->zeroWeightOffset = zeroWeightOffset;
    };
    
    inline SensorReading getZeroWeightOffset() const {
      boost::lock_guard<boost::mutex> lock(mutex);
      return zeroWeightOffset;
    };

    inline void setCompensatedZeroWeightOffset(const SensorReading&
        compensatedZeroWeightOffset) {
      boost::lock_guard<boost::mutex> lock(mutex);
      this->compensatedZeroWeightOffset = compensatedZeroWeightOffset;
    };
    
    inline SensorReading getCompensatedZeroWeightOffset() const {
      boost::lock_guard<boost::mutex> lock(mutex);
      return compensatedZeroWeightOffset;
    };

    void setPackageBufferSize(size_t packageBufferSize);
    size_t getPackageBufferSize() const;
    
    inline size_t getNumPackages() const {
      boost::lock_guard<boost::mutex> lock(mutex);
      return packageBuffer.size();
    };
    
    inline size_t getNumReceivedPackages() const {
      return numReceivedPackages;
    };
    
    inline size_t getNumDroppedPackages() const {
      return numDroppedPackages;
    };
    
    inline size_t getNumCalibrationReadings() const {
      return boost::accumulators::count(zeroWeightAccumulator);
    };
    
    bool isConnected() const;
    
    inline bool hasPackages() const {
      boost::lock_guard<boost::mutex> lock(mutex);
      return !packageBuffer.empty();
    };
    
    inline bool isCalibrating() const {
      boost::lock_guard<boost::mutex> lock(mutex);
      return numCalibrationReadings;
    };
    
    void connect(const std::string& deviceFilename, unsigned int
      deviceBaudRate = 115200);
    void disconnect(bool wait = false);
    
    void configure(const SensorConfig& config);
    void calibrate(size_t numReadings);
    
    SensorPackage dequeuePackage(BufferPosition position = 
      buffer_position_newest);
    SensorReading dequeueReading(BufferPosition position = 
      buffer_position_newest, bool compensated = false);
    void clearPackages();
  protected:
    SerialDevice serialDevice;
    StateMachine stateMachine;
    
    std::deque<SensorPackage> packageBuffer;
    size_t packageBufferSize;
    
    double signalToForceFactor;
    
    SensorReading zeroWeightOffset;
    SensorReading compensatedZeroWeightOffset;
    Accumulator zeroWeightAccumulator;
    Accumulator compensatedZeroWeightAccumulator;
    size_t numCalibrationReadings;
    
    size_t numReceivedPackages;
    size_t numDroppedPackages;
    
    mutable boost::mutex mutex;
    
    void processReadData(const std::vector<unsigned char>& data,
      int64_t timestamp);
  };
};

#endif // OPTOFORCE_SENSOR_HPP
