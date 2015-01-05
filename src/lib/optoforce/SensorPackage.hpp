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

/** \file SensorPackage.hpp
  * \brief Header file providing the SensorPackage class
  */

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
  /** \brief Representation of an OptoForce sensor data package
    * 
    * The sensor data package is the low-level communication entity of
    * the OptoForce sensor.
    */
  
  class SensorPackage {
  friend class StateMachine;
  public:
    /** \brief Enumerable representing the package version
      */ 
    enum Version {
      /** Version undefined
        */
      version_undefined = 0,
      /** .66 package version
        */
      version_66 = 1,
      /** .67 package version
        */
      version_67 = 2,
      /** .68 package version
        */
      version_68 = 3,
      /** .94 package version
        */
      version_94 = 4
    };
    
    /** \brief Enumerable representing the package checksum result
      */ 
    enum Checksum {
      /** No checksum result
        */ 
      checksum_none = 0,
      /** Checksum okay
        */ 
      checksum_okay = 1,
      /** Checksum error
        */ 
      checksum_error = 2
    };
    
    /** \brief Error indicating operand size mismatch
      */
    struct OperandSizeError : virtual std::exception,
      virtual boost::exception {};  
      
    /** \brief Error indicating bad package conversion
      */
    struct ConversionError : virtual std::exception,
      virtual boost::exception {};  
    
    /** \brief Default constructor
      * 
      * \param[in] version The sensor package version.
      */
    SensorPackage(Version version = version_undefined);
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source sensor package which is being copied
      *   to this sensor package.
      */
    SensorPackage(const SensorPackage& src);
    
    /** \brief Destructor
      */
    ~SensorPackage();

    /** \brief Retrieve the version of the sensor package
      * 
      * \return The version of the sensor package.
      */ 
    inline Version getVersion() const {
      return version;
    };
    
    /** \brief Retrieve the configuration indicated by the sensor package
      * 
      * \return The sensor configuration indicated by the sensor package.
      */ 
    inline const SensorConfig& getConfig() const {
      return config;
    };
    
    /** \brief Retrieve the raw signals of the sensor package
      * 
      * \return A variable-size array containing the raw signals of the
      *   sensor package.
      */ 
    inline const std::vector<int>& getRawSignals() const {
      return rawSignals;
    };
    
    /** \brief Retrieve the compensated signals of the sensor package
      * 
      * \return A variable-size array containing the temperature-compensated
      *   signals of the sensor package.
      */ 
    inline const std::vector<int>& getCompensatedSignals() const {
      return compensatedSignals;
    };
    
    /** \brief Retrieve the force values of the sensor package
      * 
      * \return A variable-size array providing the force values of the
      *   sensor package, where all values are based on the sensor-internal
      *   force representation.
      */ 
    inline const std::vector<int>& getForce() const {
      return force;
    };
    
    /** \brief Retrieve the temperature of the sensor package
      * 
      * \return A variable-size array providing the temperature values of
      *   the sensor package, where all values are based on the sensor-internal
      *   temperature representation.
      */ 
    inline const std::vector<int>& getTemperature() const {
      return temperature;
    };
    
    /** \brief Retrieve the acquisition timestamp of the sensor package
      * 
      * \return The acquisition timestamp of the sensor package in [ns]
      *   since the epoch.
      */ 
    inline int64_t getTimestamp() const {
      return timestamp;
    };
    
    /** \brief Retrieve the checksum result of the sensor package
      * 
      * \return The checksum result of the sensor package.
      */ 
    inline Checksum getChecksum() const {
      return checksum;
    };

    /** \brief Query if the sensor package provides raw signals
      * 
      * \return True, if the sensor package provides raw signals.
      */
    inline bool hasRawSignals() const {
      return !rawSignals.empty();
    };
    
    /** \brief Query if the sensor package provides compensated signals
      * 
      * \return True, if the sensor package provides temperature-compensated
      *   signals.
      */
    inline bool hasCompensatedSignals() const {
      return !compensatedSignals.empty();
    };
    
    /** \brief Query if the sensor package provides forces
      * 
      * \return True, if the sensor package provides forces.
      */
    inline bool hasForce() const {
      return !force.empty();
    };
    
    /** \brief Query if the sensor package provides temperature
      * 
      * \return True, if the sensor package provides temperature values.
      */
    inline bool hasTemperature() const {
      return !temperature.empty();
    };
    
    /** \brief Query if the sensor package matches an operand
      * 
      * Two sensor packages match if their contents are compatible with
      * respect to simple arithmetic operations (as assumed by the defined
      * package operators).
      * 
      * \param[in] operand The operand which is required to be compatible
      *   with this sensor package.
      * \return True, if the content of the sensor package matches the
      *   content of the operand.
      */
    inline bool matches(const SensorPackage& operand) const {
      return ((rawSignals.size() == operand.rawSignals.size()) &&
        (compensatedSignals.size() == operand.compensatedSignals.size()) &&
        (force.size() == operand.force.size()));
    };
    
    /** \brief Assignment operator
      * 
      * \param[in] src The source sensor package which is being copied to
      *   this sensor package.
      * \return A non-const reference to this sensor package.
      */
    SensorPackage& operator=(const SensorPackage& src);

    /** \brief Unary summation operator
      * 
      * \param[in] summand The sensor package acting as second summand
      *   in the summation.
      * \return A non-const reference to this sensor package.
      * 
      * \note If the summand is not compatible with this sensor package,
      *   OperandSizeError will be thrown.
      */
    SensorPackage& operator+=(const SensorPackage& summand);
    
    /** \brief Unary subtraction operator
      * 
      * \param[in] subtrahend The sensor package acting as subtrahend
      *   in the subtraction.
      * \return A non-const reference to this sensor package.
      * 
      * \note If the subtrahend is not compatible with this sensor package,
      *   OperandSizeError will be thrown.
      */
    SensorPackage& operator-=(const SensorPackage& subtrahend);
    
    /** \brief Unary multiplication operator
      * 
      * \param[in] factor The second (scalar) factor in the multiplication.
      * \return A non-const reference to this sensor package.
      */
    SensorPackage& operator*=(double factor);
    
    /** \brief Unary division operator
      * 
      * \param[in] divisor The (scalar) divisor in the division.
      * \return A non-const reference to this sensor package.
      */
    SensorPackage& operator/=(double divisor);
    
    /** \brief Binary summation operator
      * 
      * \param[in] summand The sensor package acting as second summand
      *   in the summation.
      * \return The sensor package representing the sum of this sensor
      *   package and the summand.
      * 
      * \note If the summand is not compatible with this sensor package,
      *   OperandSizeError will be thrown.
      */
    SensorPackage operator+(const SensorPackage& summand) const;
    
    /** \brief Binary subtraction operator
      * 
      * \param[in] subtrahend The sensor package acting as subtrahend
      *   in the subtraction.
      * \return The sensor package representing the difference of this
      *   sensor package and the subtrahend.
      * 
      * \note If the subtrahend is not compatible with this sensor package,
      *   OperandSizeError will be thrown.
      */
    SensorPackage operator-(const SensorPackage& subtrahend) const;
    
    /** \brief Binary multiplication operator
      * 
      * \param[in] factor The second (scalar) factor in the multiplication.
      * \return The sensor package representing the product of this sensor
      *   package and the factor.
      */
    SensorPackage operator*(double factor) const;
    
    /** \brief Binary division operator
      * 
      * \param[in] divisor The (scalar) divisor in the division.
      * \return The sensor package representing the quotient of this sensor
      *   package and the divisor.
      */
    SensorPackage operator/(double divisor) const;
    
    
    /** \brief Clear the sensor package
      * 
      * Clearing the sensor package implies the clearance of all
      * variable-size package members and re-initialization of all
      * other package members to their default values.
      */
    void clear();
    
    /** \brief Convert the sensor package's raw signals to a reading
      * 
      * This method performs a conversion of the package's raw signals into
      * a sensor reading. The conversion recipe thereby depends on the package
      * version.
      * 
      * \param[in,out] reading The sensor reading which will be assigned
      *   the conversion result.
      * \param[in] signalToForceFactor The signal-to-force factor which
      *   relates the sensor's internal force value representation to SI-unit
      *   forces and torques in [N] and [Nm], respectively.
      * 
      * \note If the package cannot be converted to a sensor reading, e.g.,
      *   due to missing package information, ConversionError will be thrown.
      */
    void toReading(SensorReading& reading, double
      signalToForceFactor) const;
      
    /** \brief Convert the sensor package's compensated signals to a reading
      * 
      * This method performs a conversion of the package's
      * temperature-compensated signals into a sensor reading. The conversion
      * recipe thereby depends on the package version.
      * 
      * \param[in,out] reading The sensor reading which will be assigned
      *   the conversion result.
      * \param[in] signalToForceFactor The signal-to-force factor which
      *   relates the sensor's internal force value representation to SI-unit
      *   forces and torques in [N] and [Nm], respectively.
      * 
      * \note If the package cannot be converted to a temperature-compensated
      *   sensor reading, e.g., due to missing package information,
      *   ConversionError will be thrown.
      */
    void toCompensatedReading(SensorReading& reading, double
      signalToForceFactor) const;
      
  protected:
    /** \brief The version of the sensor package
      */
    Version version;
    
    /** \brief The sensor configuration indicated by the sensor package
      */
    SensorConfig config;

    /** \brief The raw signals of the sensor package
      */
    std::vector<int> rawSignals;
    
    /** \brief The temperature-compensated signals of the sensor package
      */
    std::vector<int> compensatedSignals;

    /** \brief The force values of the sensor package
      */
    std::vector<int> force;    
    
    /** \brief The temperature values of the sensor package
      */
    std::vector<int> temperature;
    
    /** \brief The acquisition timestamp of the sensor package in [ns]
      *   since the epoch.
      */
    int64_t timestamp;
    
    /** \brief The checksum result of the sensor package
      */
    Checksum checksum;
  };
};

#endif
