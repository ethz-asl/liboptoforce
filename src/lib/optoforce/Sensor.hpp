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

/** \file Sensor.hpp
  * \brief Header file providing the Sensor class
  */

#ifndef OPTOFORCE_SENSOR_HPP
#define OPTOFORCE_SENSOR_HPP

#include <deque>
#include <cstdint>
#include <exception>

#include <optoforce/SerialDevice.hpp>
#include <optoforce/StateMachine.hpp>
#include <optoforce/SensorPackage.hpp>
#include <optoforce/SensorReading.hpp>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/exception/exception.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>

namespace optoforce {
  /** \brief Implementation of the OptoForce sensor
    * 
    * This class implements the OptoForce sensor. It provides the sensor's
    * associated serial communication device, its underlying state machine,
    * its package buffer, calibration data, and simple statistics.
    */
  
  class Sensor {
  public:
    /** \brief Enumerable indicating package buffer positions
      */
    enum BufferPosition {
      /** Least recent package in buffer
        */
      buffer_position_oldest,
      /** Most recent package in buffer
        */
      buffer_position_newest
    };
    
    /** \brief Type definition of the sensor reading mean accumulator
      */
    typedef boost::accumulators::accumulator_set<SensorReading,
      boost::accumulators::stats<boost::accumulators::tag::mean> >
      Accumulator;
  
    /** \brief Error indicating absence of an expected sensor package
      */
    struct NoSuchPackageError : virtual std::exception,
      virtual boost::exception {};
    
    /** \brief Default constructor
      * 
      * \param[in] packageBufferSize The size of the sensor package buffer.
      * \param[in] signalToForceFactor The factor relating the sensor's
      *   internal force value representation to SI-unit forces and torques.
      */
    Sensor(size_t packageBufferSize = 32, double signalToForceFactor = 1e-3);
    
    /** \brief Destructor
      */
    ~Sensor();

    /** \brief Retrieve the sensor's device filename
      * 
      * \return The filename of the character device which is associated
      *   with the sensor's serial communication device.
      * 
      * \see SerialDevice::getFilename
      */
    inline const std::string& getDeviceFilename() const {
      return serialDevice.getFilename();
    };
    
    /** \brief Retrieve the number of bytes read from the sensor
      * 
      * \return The number of bytes read from the sensor's associated
      *   serial communication device.
      * 
      * \see SerialDevice::getNumRead
      */
    inline size_t getDeviceNumRead() const {
      return serialDevice.getNumRead();
    };

    /** \brief Retrieve the number of bytes written to the sensor
      * 
      * \return The number of bytes written to the sensor's associated
      *   serial communication device.
      * 
      * \see SerialDevice::getNumWritten
      */
    inline size_t getDeviceNumWritten() const {
      return serialDevice.getNumWritten();
    };
    
    /** \brief Retrieve a package from the sensor's package buffer
      * 
      * This accessor does not modify the sensor's package buffer.
      *
      * \param[in] position The buffer position of the package to be
      *   retrieved.
      * \return A copy of the package at the specified buffer position.
      * 
      * \note If the package buffer is empty, NoSuchPackageError will
      *   be thrown. In order to avoid exceptions, evaluate the return
      *   value of Sensor::hasPackages.
      * 
      * \see dequeuePackage
      */
    SensorPackage getPackage(BufferPosition position = 
      buffer_position_newest) const;
      
    /** \brief Retrieve a reading from the sensor's package buffer
      * 
      * This accessor does not modify the sensor's package buffer.
      * 
      * \param[in] position The buffer position of the reading to be
      *   retrieved.
      * \param[in] compensated If true, attempt to generate the reading
      *   from temperature-compensated sensor values.
      * \return The reading which results from converting the package
      *   at the specified buffer position.
      * 
      * \note If the package buffer is empty, NoSuchPackageError will
      *   be thrown. In order to avoid exceptions, evaluate the return
      *   value of Sensor::hasPackages.
      * 
      * \see dequeueReading
      */
    SensorReading getReading(BufferPosition position = 
      buffer_position_newest, bool compensated = false) const;
    
    /** \brief Set the sensor's signal-to-force factor
      * 
      * \param[in] signalToForceFactor The new factor relating the sensor's
      *   internal force value representation to SI-unit forces and torques.
      */
    inline void setSignalToForceFactor(double signalToForceFactor) {
      boost::lock_guard<boost::mutex> lock(mutex);
      this->signalToForceFactor = signalToForceFactor;
    };
      
    /** \brief Retrieve the sensor's signal-to-force factor
      * 
      * \return The factor relating the sensor's internal force value
      *   representation to SI-unit forces and torques.
      */
    inline double getSignalToForceFactor() {
      return signalToForceFactor;
    };
    
    /** \brief Set the sensor's zero-weight offset
      * 
      * \param[in] zeroWeightOffset The new zero-weight offset of the sensor.
      */
    inline void setZeroWeightOffset(const SensorReading& zeroWeightOffset) {
      boost::lock_guard<boost::mutex> lock(mutex);
      this->zeroWeightOffset = zeroWeightOffset;
    };
    
    /** \brief Retrieve the sensor's zero-weight offset
      * 
      * \return The zero-weight offset of the sensor.
      */
    inline SensorReading getZeroWeightOffset() const {
      boost::lock_guard<boost::mutex> lock(mutex);
      return zeroWeightOffset;
    };

    /** \brief Set the sensor's compensated zero-weight offset
      * 
      * \param[in] compensatedZeroWeightOffset The new temperature-compensated
      *   zero-weight offset of the sensor.
      */
    inline void setCompensatedZeroWeightOffset(const SensorReading&
        compensatedZeroWeightOffset) {
      boost::lock_guard<boost::mutex> lock(mutex);
      this->compensatedZeroWeightOffset = compensatedZeroWeightOffset;
    };
    
    /** \brief Retrieve the sensor's compensated zero-weight offset
      * 
      * \return The temperature-compensated zero-weight offset of the sensor.
      */
    inline SensorReading getCompensatedZeroWeightOffset() const {
      boost::lock_guard<boost::mutex> lock(mutex);
      return compensatedZeroWeightOffset;
    };

    /** \brief Set the sensor's package buffer size
      * 
      * \param[in] packageBufferSize The new size of the sensor's package
      *   buffer.
      */
    void setPackageBufferSize(size_t packageBufferSize);
    
    /** \brief Retrieve the sensor's package buffer size
      * 
      * \return The size of the sensor's package buffer.
      */
    size_t getPackageBufferSize() const;
    
    /** \brief Retrieve the sensor's number of buffered packages
      * 
      * \return The number of packages in the sensor's package buffer.
      */
    inline size_t getNumPackages() const {
      boost::lock_guard<boost::mutex> lock(mutex);
      return packageBuffer.size();
    };
    
    /** \brief Retrieve the sensor's number of received packages
      * 
      * \return The number of packages received from the sensor.
      */
    inline size_t getNumReceivedPackages() const {
      return numReceivedPackages;
    };
    
    /** \brief Retrieve the sensor's number of dropped packages
      * 
      * \return The number of packages dropped from the sensor's package
      *   buffer.
      */
    inline size_t getNumDroppedPackages() const {
      return numDroppedPackages;
    };
    
    /** \brief Retrieve the sensor's number of calibration readings
      * 
      * \return The number of readings in the statistics accumulator
      *   which is used to determine the sensor's zero-weight offset.
      */
    inline size_t getNumCalibrationReadings() const {
      return boost::accumulators::count(zeroWeightAccumulator);
    };
    
    /** \brief Query if the sensor is connected
      * 
      * \return True, if the sensor is connected via its associated serial
      *   communication device.
      * 
      * \see SerialDevice::isOpen
      */
    bool isConnected() const;
    
    /** \brief Query if the sensor has any queued packages
      * 
      * \return True, if the sensor has at least one package in its package
      *   buffer.
      */
    inline bool hasPackages() const {
      boost::lock_guard<boost::mutex> lock(mutex);
      return !packageBuffer.empty();
    };
    
    /** \brief Query if the sensor is calibrating
      * 
      * \return True, if the sensor's zero-weight offset calibration is
      *   in progress.
      */
    inline bool isCalibrating() const {
      boost::lock_guard<boost::mutex> lock(mutex);
      return numCalibrationReadings;
    };
    
    /** \brief Connect the sensor
      * 
      * This method opens the sensor's serial communication device and
      * initializes its state machine.
      * 
      * \param[in] deviceFilename The filename of the character device which
      *   will be associated with the sensor's serial communication device.
      * \param[in] deviceBaudRate The baud rate at which the sensor is expected
      *   to communicate.
      * 
      * \see SerialDevice::open and StateMachine::restart
      */
    void connect(const std::string& deviceFilename, unsigned int
      deviceBaudRate = 115200);
    
    /** \brief Disconnect the sensor
      * 
      * This method closes the sensor's serial communication device.
      * 
      * \param[in] wait If true, block until the serial communication
      *   device has been closed.
      * 
      * \see SerialDevice::close
      */
    void disconnect(bool wait = false);
    
    /** \brief Configure the sensor
      * 
      * This method writes a sensor configuration to the open serial
      * communication device.
      * 
      * \param[in] config The sensor configuration to be written.
      */
    void configure(const SensorConfig& config);
    
    /** \brief Calibrate the sensor
      * 
      * This method initiates the zero-weight offset calibration of the
      * sensor and returns immediately.
      * 
      * \param[in] numReadings The number of readings to acquire for the
      *   statistics accumulator which is used to determine the sensor's
      *   zero-weight offset.
      * 
      * \see isCalibrating
      */
    void calibrate(size_t numReadings);
    
    /** \brief Dequeue a package from the sensor's package buffer
      * 
      * This method modifies the sensor's package buffer.
      *
      * \param[in] position The buffer position of the package to be
      *   dequeued.
      * \return The package which has been dequeued from the package buffer
      *   at the specified position.
      * 
      * \note If the package buffer is empty, NoSuchPackageError will
      *   be thrown. In order to avoid exceptions, evaluate the return
      *   value of Sensor::hasPackages.
      * 
      * \see getPackage
      */
    SensorPackage dequeuePackage(BufferPosition position = 
      buffer_position_newest);
    
    /** \brief Dequeue a reading from the sensor's package buffer
      * 
      * This method modifies the sensor's package buffer.
      *
      * \param[in] position The buffer position of the reading to be
      *   dequeued.
      * \param[in] compensated If true, attempt to generate the reading
      *   from temperature-compensated sensor values.
      * \return The reading which results from converting the package
      *   dequeued from the package buffer at the specified position.
      * 
      * \note If the package buffer is empty, NoSuchPackageError will
      *   be thrown. In order to avoid exceptions, evaluate the return
      *   value of Sensor::hasPackages.
      * 
      * \see getReading
      */
    SensorReading dequeueReading(BufferPosition position = 
      buffer_position_newest, bool compensated = false);
    
    /** \brief Clear the sensor's package buffer
      * 
      * This method explicitly removes all packages from the sensor's
      * package buffer.
      */
    void clearPackages();
    

    void setAlphaOffsetFilter(double alpha);
    void setForceVariance(const SensorReading& var);
    void setMahalanobisDistance(double distance) {
      mahalanobisDistance = distance;
    };
    double computeMahalanobisDistance(const SensorReading& mean, const SensorReading& sample);

  protected:
    /** \brief The sensor's associated serial communication device
      */
    SerialDevice serialDevice;
    
    /** \brief The sensor's underlying state machine
      */
    StateMachine stateMachine;
    
    /** \brief The sensor's package buffer
      */
    std::deque<SensorPackage> packageBuffer;
    
    /** \brief The size of the sensor's package buffer
      */
    size_t packageBufferSize;
    
    /** \brief The sensor's signal-to-force factor
      * 
      * The signal-to-force factor relates the sensor's internal force
      * value representation to SI-unit forces and torques in [N] and [Nm],
      * respectively.
      */
    double signalToForceFactor;
    
    /** \brief The sensor's zero-weight offset
      */
    SensorReading zeroWeightOffset;
    
    /** \brief The sensor's temperature-compensated zero-weight offset
      */
    SensorReading compensatedZeroWeightOffset;
    
    SensorReading variance;

    double alpha;
    double mahalanobisDistance;

    /** \brief The sensor's zero-weight accumulator
      * 
      * This mean accumulator determines the sensor's zero-weight offset
      * during calibration.
      */
    Accumulator zeroWeightAccumulator;
    
    /** \brief The sensor's temperature-compensated zero-weight accumulator
      * 
      * This mean accumulator determines the sensor's temperature-compensated
      * zero-weight offset during calibration.
      */
    Accumulator compensatedZeroWeightAccumulator;
    
    /** \brief The sensor's number of calibration readings
      * 
      * This member stores the number of readings requested for calibrating
      * the sensor's zero-weight offset.
      */ 
    size_t numCalibrationReadings;
    
    /** \brief The sensor's number of received packages
      */
    size_t numReceivedPackages;
    
    /** \brief The sensor's number of dropped packages
      */
    size_t numDroppedPackages;
    
    /** \brief The sensor's thread mutex
      */
    mutable boost::mutex mutex;
    
    /** \brief Handler for processing read data
      * 
      * This signal handler is invoked by the sensor's serial communication
      * device upon the completion of a read operation. It processes the data
      * using the state machine and queues the resulting packages.
      * 
      * \param[in] data The data received by the serial communication device.
      * \param[in] timestamp The timestamp of the data received by the serial
      *   communication device.
      * 
      * \see SerialDevice::connectOnReadComplete
      */
    void processReadData(const std::vector<unsigned char>& data,
      int64_t timestamp);
  };
};

#endif
