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

#include "SensorPackage.hpp"

#include <boost/throw_exception.hpp>

namespace optoforce {
  
/*****************************************************************************/
/* Constructors                                                              */
/*****************************************************************************/

SensorPackage::SensorPackage(Version version) :
  version(version),
  timestamp(0),
  checksum(checksum_none) {
}

SensorPackage::SensorPackage(const SensorPackage& src) :
  version(src.version),
  config(src.config),
  rawSignals(src.rawSignals),
  compensatedSignals(src.compensatedSignals),
  force(src.force),
  temperature(src.temperature),
  timestamp(src.timestamp),
  checksum(src.checksum) {
}

/*****************************************************************************/
/* Destructor                                                                */
/*****************************************************************************/

SensorPackage::~SensorPackage() {
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

SensorPackage& SensorPackage::operator=(const SensorPackage& src) {
  version = src.version;
  config = src.config;

  rawSignals = src.rawSignals;
  compensatedSignals = src.compensatedSignals;
  
  force = src.force;  
  temperature = src.temperature;
  
  timestamp = src.timestamp;
  
  checksum = src.checksum;
  
  return *this;
}

SensorPackage& SensorPackage::operator+=(const SensorPackage& summand) {
  if (matches(summand)) {
    for (size_t i = 0; i < rawSignals.size(); ++i)
      rawSignals[i] += summand.rawSignals[i];
    for (size_t i = 0; i < compensatedSignals.size(); ++i)
      compensatedSignals[i] += summand.compensatedSignals[i];
    for (size_t i = 0; i < force.size(); ++i)
      force[i] += summand.force[i];
  }
  else
    BOOST_THROW_EXCEPTION(OperandSizeError());
  
  return *this;
}
SensorPackage& SensorPackage::operator-=(const SensorPackage& subtrahend) {
  if (matches(subtrahend)) {
    for (size_t i = 0; i < rawSignals.size(); ++i)
      rawSignals[i] -= subtrahend.rawSignals[i];
    for (size_t i = 0; i < compensatedSignals.size(); ++i)
      compensatedSignals[i] -= subtrahend.compensatedSignals[i];
    for (size_t i = 0; i < force.size(); ++i)
      force[i] -= subtrahend.force[i];
  }
  else
    BOOST_THROW_EXCEPTION(OperandSizeError());
  
  return *this;
}

SensorPackage& SensorPackage::operator*=(double factor) {
  for (size_t i = 0; i < rawSignals.size(); ++i)
    rawSignals[i] *= factor;
  for (size_t i = 0; i < compensatedSignals.size(); ++i)
    compensatedSignals[i] *= factor;
  for (size_t i = 0; i < force.size(); ++i)
    force[i] *= factor;
  
  return *this;
}

SensorPackage& SensorPackage::operator/=(double divisor) {
  return operator*=(1.0/divisor);
}

SensorPackage SensorPackage::operator+(const SensorPackage& summand) const {
  SensorPackage sum = *this;
  sum += summand;
  return sum;
}

SensorPackage SensorPackage::operator-(const SensorPackage& subtrahend)
    const {
  SensorPackage diff = *this;
  diff -= subtrahend;
  return diff;
}

SensorPackage SensorPackage::operator*(double factor) const {
  SensorPackage prod = *this;
  prod *= factor;
  return prod;
}

SensorPackage SensorPackage::operator/(double divisor) const {
  SensorPackage quot = *this;
  quot /= divisor;
  return quot;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void SensorPackage::clear() {
  version = version_undefined;
  config.reset();
  
  rawSignals.clear();
  compensatedSignals.clear();
  
  force.clear();  
  temperature.clear();
  
  timestamp = 0;
  
  checksum = checksum_none;
}

void SensorPackage::toReading(SensorReading& reading, double
    signalToForceFactor) const {
  switch (version) {
    case version_66:
      if (rawSignals.size() == 4)  {
        reading.setForceX((rawSignals[0]-rawSignals[2])*signalToForceFactor);
        reading.setForceY((rawSignals[3]-rawSignals[1])*signalToForceFactor);
        reading.setForceZ((rawSignals[0]+rawSignals[1]+rawSignals[2]+
          rawSignals[3])*0.25*signalToForceFactor);
        reading.clearTorque();
        reading.setTimestamp(timestamp);
        
        break;
      }
    case version_67:
      if ((config.getMode() == SensorConfig::mode_raw) &&
          (rawSignals.size() == 4)) {
        reading.setForceX((rawSignals[0]-rawSignals[2])*signalToForceFactor);
        reading.setForceY((rawSignals[3]-rawSignals[1])*signalToForceFactor);
        reading.setForceZ((rawSignals[0]+rawSignals[1]+rawSignals[2]+
          rawSignals[3])*0.25*signalToForceFactor);
        reading.clearTorque();
        reading.setTimestamp(timestamp);
      
        break;
      }
      else if ((config.getMode() == SensorConfig::mode_force) &&
          (force.size() == 3)) {
        reading.setForceX(force[0]*signalToForceFactor);
        reading.setForceY(force[1]*signalToForceFactor);
        reading.setForceZ(force[2]*signalToForceFactor);
        reading.clearTorque();
        reading.setTimestamp(timestamp);
      
        break;
      }
    case version_68:
      if (force.size() == 3) {
        reading.setForceX(force[0]*signalToForceFactor);
        reading.setForceY(force[1]*signalToForceFactor);
        reading.setForceZ(force[2]*signalToForceFactor);
        reading.clearTorque();
        reading.setTimestamp(timestamp);
        
        break;
      }
    case version_94:
      if (force.size() == 12) {
        reading.setForceX((force[0]+force[3]+force[6]+force[9])*0.25*
          signalToForceFactor);
        reading.setForceY((force[1]+force[4]+force[7]+force[10])*0.25*
          signalToForceFactor);
        reading.setForceZ((force[2]+force[5]+force[8]+force[11])*0.0625*
          signalToForceFactor);
        reading.setTorqueX((force[2]-force[8])*0.5*signalToForceFactor);
        reading.setTorqueY((force[11]-force[5])*0.5*signalToForceFactor);
        reading.setTorqueZ((-force[0]-force[3]+force[7]+force[9])*0.25*
          signalToForceFactor);
        reading.setTimestamp(timestamp);
        
        break;
      }
    default:
      BOOST_THROW_EXCEPTION(ConversionError());
  }
}

void SensorPackage::toCompensatedReading(SensorReading& reading, double
    signalToForceFactor) const {
  switch (version) {
    case version_68:
      if (compensatedSignals.size() == 4)  {
        reading.setForceX((compensatedSignals[0]-compensatedSignals[2])*
          signalToForceFactor);
        reading.setForceY((compensatedSignals[3]-compensatedSignals[1])*
          signalToForceFactor);
        reading.setForceZ((compensatedSignals[0]+compensatedSignals[1]+
          compensatedSignals[2]+compensatedSignals[3])*0.25*
          signalToForceFactor);
        reading.clearTorque();
        reading.setTimestamp(timestamp);
        
        break;
      }
    default:
      BOOST_THROW_EXCEPTION(ConversionError());
  }
}

}
