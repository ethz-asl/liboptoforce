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

#include "SensorReading.hpp"

namespace optoforce {
  
/*****************************************************************************/
/* Constructors                                                              */
/*****************************************************************************/

SensorReading::SensorReading(double forceX, double forceY, double forceZ,
    int64_t timestamp) :
  force(3),
  timestamp(timestamp) {
  force[0] = forceX;
  force[1] = forceY;
  force[2] = forceZ;
}

SensorReading::SensorReading(std::vector<double>& force, std::vector<double>&
    torque, int64_t timestamp) :
  force(force),
  torque(torque),
  timestamp(timestamp) {
  if (this->force.size() != 3)
    this->force.resize(3, 0.0);
  if (!this->torque.empty() && (this->torque.size() != 3))
    this->torque.resize(3, 0.0);
}

SensorReading::SensorReading(double forceX, double forceY, double forceZ,
    double torqueX, double torqueY, double torqueZ, int64_t timestamp) :
  force(3),
  torque(3),
  timestamp(timestamp) {
  force[0] = forceX;
  force[1] = forceY;
  force[2] = forceZ;
  
  torque[0] = torqueX;
  torque[1] = torqueY;
  torque[2] = torqueZ;
}

SensorReading::SensorReading(const SensorReading& src) :
  force(src.force),
  torque(src.torque),
  timestamp(src.timestamp) {
}
  
/*****************************************************************************/
/* Destructor                                                                */
/*****************************************************************************/

SensorReading::~SensorReading() {
}
  
/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

SensorReading& SensorReading::operator=(const SensorReading& src) {
  force = src.force;
  torque = src.torque;
  
  timestamp = src.timestamp;
  
  return *this;
}
  
SensorReading& SensorReading::operator+=(const SensorReading& summand) {
  force[0] += summand.force[0];
  force[1] += summand.force[1];
  force[2] += summand.force[2];
  
  if (!summand.torque.empty()) {
    if (torque.empty())
      torque.resize(3, 0.0);
    
    torque[0] += summand.torque[0];
    torque[1] += summand.torque[1];
    torque[2] += summand.torque[2];
  }
  
  return *this;
}

SensorReading& SensorReading::operator-=(const SensorReading& subtrahend) {
  force[0] -= subtrahend.force[0];
  force[1] -= subtrahend.force[1];
  force[2] -= subtrahend.force[2];
  
  if (!subtrahend.torque.empty()) {
    if (torque.empty())
      torque.resize(3, 0.0);
    
    torque[0] -= subtrahend.torque[0];
    torque[1] -= subtrahend.torque[1];
    torque[2] -= subtrahend.torque[2];
  }
  
  return *this;
}

SensorReading& SensorReading::operator*=(double factor) {
  force[0] *= factor;
  force[1] *= factor;
  force[2] *= factor;
  
  if (!torque.empty()) {
    torque[0] *= factor;
    torque[1] *= factor;
    torque[2] *= factor;
  }
  
  return *this;
}

SensorReading& SensorReading::operator/=(double divisor) {
  force[0] /= divisor;
  force[1] /= divisor;
  force[2] /= divisor;
  
  if (!torque.empty()) {
    torque[0] /= divisor;
    torque[1] /= divisor;
    torque[2] /= divisor;
  }
  
  return *this;
}

SensorReading SensorReading::operator+(const SensorReading& summand) const {
  SensorReading sum = *this;
  sum += summand;
  return sum;
}

SensorReading SensorReading::operator-(const SensorReading& subtrahend) const {
  SensorReading diff = *this;
  diff -= subtrahend;
  return diff;
}

SensorReading SensorReading::operator*(double factor) const {
  SensorReading prod = *this;
  prod *= factor;
  return prod;
}

SensorReading SensorReading::operator/(double divisor) const {
  SensorReading quot = *this;
  quot /= divisor;
  return quot;
}

}
