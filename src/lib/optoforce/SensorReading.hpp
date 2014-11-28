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

#ifndef OPTOFORCE_SENSORREADING_HPP
#define OPTOFORCE_SENSORREADING_HPP

#include <cmath>
#include <cstdint>
#include <vector>

#include <optoforce/SensorConfig.hpp>

namespace optoforce {
  /* Representation of a sensor reading */
  class SensorReading {
  public:
    SensorReading(double forceX = 0, double forceY = 0, double forceZ = 0,
      int64_t timestamp = 0);
    SensorReading(std::vector<double>& force, std::vector<double>& torque,
      int64_t timestamp = 0);
    SensorReading(double forceX, double forceY, double forceZ,
      double torqueX, double torqueY, double torqueZ, int64_t timestamp = 0);
    SensorReading(const SensorReading& src);
    ~SensorReading();

    inline void setForce(const std::vector<double>& force) {
      this->force = force;
      if (!this->force.size() != 3)
        this->force.resize(3, 0.0);
    };
    
    inline void setForce(double forceX, double forceY, double forceZ) {
      force[0] = forceX;
      force[1] = forceY;
      force[2] = forceZ;
    };
    
    inline const std::vector<double>& getForce() const {
      return force;
    };
    
    inline void setTorque(const std::vector<double>& torque) {
      this->torque = torque;
      if (!this->torque.empty() && (this->torque.size() != 3))
        this->torque.resize(3, 0.0);
    };
    
    inline void setTorque(double torqueX, double torqueY, double torqueZ) {
      if (!torque.size() != 3)
        torque.resize(3, 0.0);
      
      torque[0] = torqueX;
      torque[1] = torqueY;
      torque[2] = torqueZ;
    };
    
    inline const std::vector<double>& getTorque() const {
      return torque;
    };
    
    inline void setForceX(double forceX) {
      force[0] = forceX;
    };
    
    inline double getForceX() const {
      return force[0];
    };
    
    inline void setForceY(double forceY) {
      force[1] = forceY;
    };
    
    inline double getForceY() const {
      return force[1];
    };
    
    void setForceZ(double forceZ) {
      force[2] = forceZ;
    };
    
    double getForceZ() const {
      return force[2];
    };

    inline void setTorqueX(double torqueX) {
      if (torque.empty())
        torque.resize(3, 0.0);
      torque[0] = torqueX;
    };
    
    inline double getTorqueX() const {
      return torque.empty() ? NAN : torque[0];
    };
    
    inline void setTorqueY(double torqueY) {
      if (torque.empty())
        torque.resize(3, 0.0);
      torque[1] = torqueY;
    };
    
    inline double getTorqueY() const {
      return torque.empty() ? NAN : torque[1];
    };
    
    inline void setTorqueZ(double torqueZ) {
      if (torque.empty())
        torque.resize(3, 0.0);
      torque[2] = torqueZ;
    };
    
    inline double getTorqueZ() const {
      return torque.empty() ? NAN : torque[2];
    };
    
    inline void setTimestamp(int64_t timestamp) {
      this->timestamp = timestamp;
    };
    
    inline int64_t getTimestamp() const {
      return timestamp;
    };
    
    inline bool hasTorque() const {
      return !torque.empty();
    };
    
    SensorReading& operator=(const SensorReading& src);
    
    SensorReading& operator+=(const SensorReading& summand);
    SensorReading& operator-=(const SensorReading& subtrahend);
    SensorReading& operator*=(double factor);
    SensorReading& operator/=(double divisor);
    
    SensorReading operator+(const SensorReading& summand) const;
    SensorReading operator-(const SensorReading& subtrahend) const;
    SensorReading operator*(double factor) const;
    SensorReading operator/(double divisor) const;
    
    inline void clearTorque() {
      torque.clear();
    };
  protected:
    std::vector<double> force;
    std::vector<double> torque;
    
    int64_t timestamp;
  };
};

#endif // OPTOFORCE_SENSORREADING_HPP
