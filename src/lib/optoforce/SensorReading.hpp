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

/** \file SensorReading.hpp
  * \brief Header file providing the SensorReading class
  */

#ifndef OPTOFORCE_SENSORREADING_HPP
#define OPTOFORCE_SENSORREADING_HPP

#include <cmath>
#include <cstdint>
#include <vector>

#include <optoforce/SensorConfig.hpp>

namespace optoforce {
  /** \brief Representation of an OptoForce sensor reading
    * 
    * The sensor reading provides SI-unit forces and torques and an
    * acquisition timestamp.
    */

  class SensorReading {
  public:
    /** \brief Default constructor
      * 
      * \param[in] forceX The linear force component acting in X-direction
      *   in [N].
      * \param[in] forceY The linear force component acting in Y-direction
      *   in [N].
      * \param[in] forceZ The linear force component acting in Z-direction
      *   in [N].
      * \param[in] timestamp The reading's acquisition timestamp in [ns]
      *   since the epoch.
      */
    SensorReading(double forceX = 0, double forceY = 0, double forceZ = 0,
      int64_t timestamp = 0);
    
    /** \brief Construct a reading from force and torque vectors
      * 
      * \param[in] force The linear force vector component of the reading
      *   in [N].
      * \param[in] torque The torque vector component of the reading in [Nm].
      * \param[in] timestamp The reading's acquisition timestamp in [ns]
      *   since the epoch.
      */
    SensorReading(std::vector<double>& force, std::vector<double>& torque,
      int64_t timestamp = 0);
    
    /** \brief Construct a reading from force and torque component values
      * 
      * \param[in] forceX The linear force component acting in X-direction
      *   in [N].
      * \param[in] forceY The linear force component acting in Y-direction
      *   in [N].
      * \param[in] forceZ The linear force component acting in Z-direction
      *   in [N].
      * \param[in] torqueX The torque component acting about the X-axis
      *   in [Nm].
      * \param[in] torqueY The torque component acting about the Y-axis
      *   in [Nm].
      * \param[in] torqueZ The torque component acting about the Z-axis
      *   in [Nm].
      * \param[in] timestamp The reading's acquisition timestamp in [ns]
      *   since the epoch.
      */
    SensorReading(double forceX, double forceY, double forceZ,
      double torqueX, double torqueY, double torqueZ, int64_t timestamp = 0);
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source sensor reading which is being copied
      *   to this sensor reading.
      */
    SensorReading(const SensorReading& src);
    
    /** \brief Destructor
      */
    ~SensorReading();

    /** \brief Set the linear force vector component of the reading
      * 
      * \param[in] force The new linear force vector component of the reading
      *   in [N]. Note that the dimensionality of the new force vector will be
      *   enforced to be exactly three.
      */ 
    inline void setForce(const std::vector<double>& force) {
      this->force = force;
      if (!this->force.size() != 3)
        this->force.resize(3, 0.0);
    };
    
    /** \brief Set the linear force component values of the reading
      * 
      * \param[in] forceX The new linear force component acting in X-direction
      *   in [N].
      * \param[in] forceY The new linear force component acting in Y-direction
      *   in [N].
      * \param[in] forceZ The new linear force component acting in Z-direction
      *   in [N].
      */ 
    inline void setForce(double forceX, double forceY, double forceZ) {
      force[0] = forceX;
      force[1] = forceY;
      force[2] = forceZ;
    };
    
    /** \brief Retrieve the linear force vector component of the reading
      * 
      * \return The linear force vector component of the reading in [N].
      */ 
    inline const std::vector<double>& getForce() const {
      return force;
    };
    
    /** \brief Set the torque vector component of the reading
      * 
      * \param[in] torque The new torque vector component of the reading
      *   in [Nm]. Note that the dimensionality of the new torque vector
      *   will be enforced to be exactly three if not empty.
      */ 
    inline void setTorque(const std::vector<double>& torque) {
      this->torque = torque;
      if (!this->torque.empty() && (this->torque.size() != 3))
        this->torque.resize(3, 0.0);
    };
    
    /** \brief Set the torque component values of the reading
      * 
      * \param[in] torqueX The new torque component acting about the X-axis
      *   in [Nm].
      * \param[in] torqueY The new torque component acting about the Y-axis
      *   in [Nm].
      * \param[in] torqueZ The new torque component acting about the Z-axis
      *   in [Nm].
      */ 
    inline void setTorque(double torqueX, double torqueY, double torqueZ) {
      if (!torque.size() != 3)
        torque.resize(3, 0.0);
      
      torque[0] = torqueX;
      torque[1] = torqueY;
      torque[2] = torqueZ;
    };
    
    /** \brief Retrieve the torque vector component of the reading
      * 
      * \return The torque vector component of the reading in [Nm].
      */ 
    inline const std::vector<double>& getTorque() const {
      return torque;
    };
    
    /** \brief Set the linear force acting in X-direction
      * 
      * \param[in] forceX The new linear force acting in X-direction in [N].
      */ 
    inline void setForceX(double forceX) {
      force[0] = forceX;
    };
    
    /** \brief Retrieve the linear force acting in X-direction
      * 
      * \return The linear force acting in X-direction in [N].
      */ 
    inline double getForceX() const {
      return force[0];
    };
    
    /** \brief Set the linear force acting in Y-direction
      * 
      * \param[in] forceY The new linear force acting in Y-direction in [N].
      */ 
    inline void setForceY(double forceY) {
      force[1] = forceY;
    };
    
    /** \brief Retrieve the linear force acting in Y-direction
      * 
      * \return The linear force acting in Y-direction in [N].
      */ 
    inline double getForceY() const {
      return force[1];
    };
    
    /** \brief Set the linear force acting in Z-direction
      * 
      * \param[in] forceZ The new linear force acting in Z-direction in [N].
      */ 
    void setForceZ(double forceZ) {
      force[2] = forceZ;
    };
    
    /** \brief Retrieve the linear force acting in Z-direction
      * 
      * \return The linear force acting in Z-direction in [N].
      */ 
    double getForceZ() const {
      return force[2];
    };

    /** \brief Set the torque acting about the X-axis
      * 
      * \param[in] torqueX The new torque acting about the X-axis in [Nm].
      */ 
    inline void setTorqueX(double torqueX) {
      if (torque.empty())
        torque.resize(3, 0.0);
      torque[0] = torqueX;
    };
    
    /** \brief Retrieve the torque acting about the X-axis
      * 
      * \return The torque acting about the X-axis in [Nm].
      */ 
    inline double getTorqueX() const {
      return torque.empty() ? NAN : torque[0];
    };
    
    /** \brief Set the torque acting about the Y-axis
      * 
      * \param[in] torqueY The new torque acting about the Y-axis in [Nm].
      */ 
    inline void setTorqueY(double torqueY) {
      if (torque.empty())
        torque.resize(3, 0.0);
      torque[1] = torqueY;
    };
    
    /** \brief Retrieve the torque acting about the Y-axis
      * 
      * \return The torque acting about the Y-axis in [Nm].
      */ 
    inline double getTorqueY() const {
      return torque.empty() ? NAN : torque[1];
    };
    
    /** \brief Set the torque acting about the Z-axis
      * 
      * \param[in] torqueZ The new torque acting about the Z-axis in [Nm].
      */ 
    inline void setTorqueZ(double torqueZ) {
      if (torque.empty())
        torque.resize(3, 0.0);
      torque[2] = torqueZ;
    };
    
    /** \brief Retrieve the torque acting about the Z-axis
      * 
      * \return The torque acting about the Z-axis in [Nm].
      */ 
    inline double getTorqueZ() const {
      return torque.empty() ? NAN : torque[2];
    };
    
    /** \brief Set the reading's acquisition timestamp
      * 
      * \param[in] timestamp The new acquisition timestamp of the reading
      *   in [ns] since the epoch.
      */ 
    inline void setTimestamp(int64_t timestamp) {
      this->timestamp = timestamp;
    };
    
    /** \brief Retrieve the reading's acquisition timestamp
      * 
      * \return The acquisition timestamp of the reading in [ns] since the
      *   epoch.
      */ 
    inline int64_t getTimestamp() const {
      return timestamp;
    };
    
    /** \brief Query if the reading has torque components
      * 
      * \return True, if the reading has torque components.
      */ 
    inline bool hasTorque() const {
      return !torque.empty();
    };
    
    /** \brief Assignment operator
      * 
      * \param[in] src The source sensor reading which is being copied to
      *   this sensor reading.
      * \return A non-const reference to this sensor reading.
      */
    SensorReading& operator=(const SensorReading& src);
    
    /** \brief Unary summation operator
      * 
      * \param[in] summand The sensor reading acting as second summand
      *   in the summation.
      * \return A non-const reference to this sensor reading.
      */
    SensorReading& operator+=(const SensorReading& summand);
    
    /** \brief Unary subtraction operator
      * 
      * \param[in] subtrahend The sensor reading acting as subtrahend
      *   in the subtraction.
      * \return A non-const reference to this sensor reading.
      */
    SensorReading& operator-=(const SensorReading& subtrahend);
    
    /** \brief Unary multiplication operator
      * 
      * \param[in] factor The second (scalar) factor in the multiplication.
      * \return A non-const reference to this sensor reading.
      */
    SensorReading& operator*=(double factor);
    
    /** \brief Unary division operator
      * 
      * \param[in] divisor The (scalar) divisor in the division.
      * \return A non-const reference to this sensor reading.
      */
    SensorReading& operator/=(double divisor);
    
    /** \brief Binary summation operator
      * 
      * \param[in] summand The sensor reading acting as second summand
      *   in the summation.
      * \return The sensor reading representing the sum of this sensor
      *   reading and the summand.
      */
    SensorReading operator+(const SensorReading& summand) const;
    
    /** \brief Binary subtraction operator
      * 
      * \param[in] subtrahend The sensor reading acting as subtrahend
      *   in the subtraction.
      * \return The sensor reading representing the difference of this
      *   sensor reading and the subtrahend.
      */
    SensorReading operator-(const SensorReading& subtrahend) const;
    
    /** \brief Binary multiplication operator
      * 
      * \param[in] factor The second (scalar) factor in the multiplication.
      * \return The sensor reading representing the product of this sensor
      *   reading and the factor.
      */
    SensorReading operator*(double factor) const;
    
    /** \brief Binary division operator
      * 
      * \param[in] divisor The (scalar) divisor in the division.
      * \return The sensor reading representing the quotient of this sensor
      *   reading and the divisor.
      */
    SensorReading operator/(double divisor) const;
    
    /** \brief Clear the reading's torque components
      * 
      * This method reduces a reading with linear force and torque components
      * to reading with linear force components only.
      */
    inline void clearTorque() {
      torque.clear();
    };
    
  protected:
    /** \brief The linear force component vector of the sensor reading in [Nm]
      */
    std::vector<double> force;
    
    /** \brief The torque component vector of the sensor reading in [Nm]
      */
    std::vector<double> torque;
    
    /** \brief The acquisition timestamp of the sensor reading in [ns] since
      *   the epoch
      */
    int64_t timestamp;
  };
};

#endif
