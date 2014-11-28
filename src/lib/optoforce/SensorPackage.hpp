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

#ifndef OPTOFORCE_SENSORPACKAGE_HPP
#define OPTOFORCE_SENSORPACKAGE_HPP

#include <cstdint>
#include <vector>
#include <exception>

#include <boost/exception/exception.hpp>
#include <boost/concept_check.hpp>

#include <optoforce/SensorConfig.hpp>
#include <optoforce/SensorReading.hpp>

namespace optoforce {
  /* Representation of a sensor data package */
  class SensorPackage {
  friend class StateMachine;
  public:
    /* Operand size error */
    struct OperandSizeError : virtual std::exception,
      virtual boost::exception {};  
    /* Conversion error */
    struct ConversionError : virtual std::exception,
      virtual boost::exception {};  
    
    /* Package version */
    enum Version {
      version_undefined = 0,
      version_66 = 1,
      version_67 = 2,
      version_68 = 3,
      version_94 = 4
    };
    
    /* Package checksum */
    enum Checksum {
      checksum_none = 0,
      checksum_okay = 1,
      checksum_error = 2
    };
    
    SensorPackage(Version = version_undefined);
    SensorPackage(const SensorPackage& src);
    ~SensorPackage();

    inline Version getVersion() const {
      return version;
    };
    
    inline const SensorConfig& getConfig() const {
      return config;
    };
    
    inline const std::vector<int>& getRawSignals() const {
      return rawSignals;
    };
    
    inline const std::vector<int>& getCompensatedSignals() const {
      return compensatedSignals;
    };
    
    inline const std::vector<int>& getForce() const {
      return force;
    };
    
    inline const std::vector<int>& getTemperature() const {
      return temperature;
    };
    
    inline int64_t getTimestamp() const {
      return timestamp;
    };
    
    inline Checksum getChecksum() const {
      return checksum;
    };

    inline bool hasRawSignals() const {
      return !rawSignals.empty();
    };
    
    inline bool hasCompensatedSignals() const {
      return !compensatedSignals.empty();
    };
    
    inline bool hasForce() const {
      return !force.empty();
    };
    
    inline bool hasTemperature() const {
      return !temperature.empty();
    };
    
    inline bool matches(const SensorPackage& operand) const {
      return ((rawSignals.size() == operand.rawSignals.size()) &&
        (compensatedSignals.size() == operand.compensatedSignals.size()) &&
        (force.size() == operand.force.size()));
    };
    
    SensorPackage& operator=(const SensorPackage& src);

    SensorPackage& operator+=(const SensorPackage& summand);
    SensorPackage& operator-=(const SensorPackage& subtrahend);
    SensorPackage& operator*=(double factor);
    SensorPackage& operator/=(double divisor);
    
    SensorPackage operator+(const SensorPackage& summand) const;
    SensorPackage operator-(const SensorPackage& subtrahend) const;
    SensorPackage operator*(double factor) const;
    SensorPackage operator/(double divisor) const;
    
    void clear();
    
    void toReading(SensorReading& reading, double
      signalToForceFactor) const;
    void toCompensatedReading(SensorReading& reading, double
      signalToForceFactor) const;
  protected:
    Version version;
    SensorConfig config;

    std::vector<int> rawSignals;
    std::vector<int> compensatedSignals;

    std::vector<int> force;    
    std::vector<int> temperature;
    
    int64_t timestamp;
    
    Checksum checksum;
  };
};

#endif // OPTOFORCE_SENSORPACKAGE_HPP
