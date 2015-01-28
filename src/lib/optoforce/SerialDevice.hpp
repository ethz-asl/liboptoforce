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

/** \file SerialDevice.hpp
  * \brief Header file providing the SerialDevice class
  */

#ifndef OPTOFORCE_SERIALDEVICE_HPP
#define OPTOFORCE_SERIALDEVICE_HPP

#include <vector>
#include <deque>
#include <string>
#include <cstdint>

#if BOOST_PARAMETER_MAX_ARITY < 7
#define BOOST_PARAMETER_MAX_ARITY 7
#endif

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/signals2.hpp>

namespace optoforce {
  /** \brief Serial communication device of an OptoForce sensor
    * 
    * The serial communication device provides the underlying asynchronous
    * communication facilities of an OptoForce sensor.
    */
  
  class SerialDevice {
  public:
    /** \brief Type definition of the signal indicating completion of
     *    asynchronous read operations
      */
    typedef boost::signals2::signal
      <void (const std::vector<unsigned char>&, int64_t)> OnReadComplete;
    
    /** \brief Default constructor
      * 
      * \param[in] readBufferSize The size of the buffer for data read
      *   asynchronously by the device.
      */
    SerialDevice(size_t readBufferSize = 512);
    
    /** \brief Constructor for opening the device
      * 
      * \param[in] filename The filename of the character device which
      *   will be associated with this serial communication device.
      * \param[in] baudRate The baud rate at which the serial communication
      *   will read and write data.
      * \param[in] readBufferSize The size of the buffer for data read
      *   asynchronously by the device.
      * 
      * \see open
      */
    SerialDevice(const std::string& filename, unsigned int baudRate = 115200,
      size_t readBufferSize = 512);
    
    /** \brief Destructor
      * 
      * If the device is still open upon destruction, it will be closed.
      * 
      * \see close
      */
    ~SerialDevice();
    
    /** \brief Retrieve the filename of the device
      * 
      * \return The filename of the character device which is associated
      *   with this serial communication device.
      */
    inline const std::string& getFilename() const {
      return filename;
    };
    
    /** \brief Retrieve the number of bytes read by the device
      * 
      * \return The number of bytes read by the device since it has been
      *   opened.
      */
    inline size_t getNumRead() const {
      return numRead;
    };
    
    /** \brief Retrieve the number of bytes written by the device
      * 
      * \return The number of bytes written by the device since it has
      *   been opened.
      */
    inline size_t getNumWritten() const {
      return numWritten;
    };
    
    /** \brief Retrieve the most recent device error
      * 
      * \return The error which most recently occurred during device
      *   operation.
      */
    inline const boost::system::error_code& getError() const {
      return error;
    };
    
    /** \brief Query if the device is open
      * 
      * \return True, if the device is open.
      */
    bool isOpen() const;
    
    /** \brief Open the device
      * 
      * \param[in] filename The filename of the character device which will
      *   be associated with this serial communication device.
      * \param[in] baudRate The baud rate at which the serial communication
      *   will read and write data.
      */
    void open(const std::string& filename, unsigned int baudRate = 115200);
    
    /** \brief Close the device
      * 
      * \param[in] wait If true, block until the device has been closed.
      * 
      * \see doClose
      */
    void close(bool wait = false);
    
    /** \brief Connect a signal handler for being notified at completion of
      *   asynchronous read operations
      * 
      * \param[in] slot The signal slot associated with the handler which
      *   shall be invoked at completion of asynchronous read operations.
      * \return A connection object representing the connection between the
      *   signal and the slot.
      */
    boost::signals2::connection connectOnReadComplete(const
      OnReadComplete::slot_type& slot);
    
    /** \brief Asynchronously write a single byte
      * 
      * This method initiates an asynchronous write operation and returns
      * immediately.
      * 
      * \param[in] data A single byte representing the data which shall be
      *   written by the device.
      * 
      * \see doWrite
      */
    void write(unsigned char data);
    
    /** \brief Asynchronously write multiple bytes
      * 
      * This method initiates an asynchronous write operation and returns
      * immediately.
      * 
      * \param[in] data An array of bytes representing the data which shall
      *   be written by the device.
      * 
      * \see doWriteVector
      */
    void write(const std::vector<unsigned char>& data);
    
  protected:
    /** \brief The filename of the character device associated with this device
      */
    std::string filename;
    
    /** \brief The I/O service which runs the device's asynchronous read and
      *   write operations
      */
    boost::asio::io_service service;
    
    /** \brief The thread which runs the device's asynchronous read and
      *   write operations
      */
    boost::thread thread;
    
    /** \brief The underlying serial port operated by this device
      */
    boost::asio::serial_port port;

    /** \brief The buffer storing the data resulting from asynchronous read
      *   operations
      */
    std::vector<unsigned char> readBuffer;
    
    /** \brief The buffer storing the data consumed by asynchronous write
      *   operations
      */
    std::deque<unsigned char> writeBuffer;
    
    /** \brief The signal indicating completion of asynchronous read
      *   operations
      */
    OnReadComplete onReadComplete;
    
    /** \brief The condition signaling that the device has been closed
      */
    boost::condition_variable closeCondition;
    
    /** \brief The mutex of the condition signaling that the device has been
     *    closed
      */
    boost::mutex closeMutex;

    /** \brief The most recent device error
      */
    boost::system::error_code error;
    
    /** \brief The number of bytes read by the device
      */
    size_t numRead;
    
    /** \brief The number of bytes written by the device
      */
    size_t numWritten;
    
    /** \brief Start asynchronous read operation
      * 
      * This method initiates an asynchronous read operation and binds
      * SerialDevice::readComplete to indicate completion or failure of
      * the operation.
      */
    void readStart();
    
    /** \brief Indicate completion or failure of asynchronous read operations
      * 
      * The invocation of this method indicates that an asynchronous read
      * operation has either failed or completed, depending on the provided
      * error code.
      * 
      * \param[in] error The error which occurred for the device since the
      *   last call to SerialDevice::readStart.
      * \param[in] numRead The number of bytes read asynchronously by the
      *   device since the last call to SerialDevice::readStart.
      */
    void readComplete(const boost::system::error_code& error,
      size_t numRead);

    /** \brief Perform asynchronous write operations for single bytes of data
      * 
      * This method is called from the device's associated thread in order
      * to perform write operations involving single bytes of data.
      * 
      * \param[in] data A single byte of data to be written by the device.
      * 
      * \see write(unsigned char)
      */
    void doWrite(unsigned char data);
    
    /** \brief Perform asynchronous write operations for multiple bytes of data
      * 
      * This method is called from the device's associated thread in order
      * to perform write operations involving multiple bytes of data.
      * 
      * \param[in] data An array of bytes representing the data to be written
      *   by the device.
      * 
      * \see write(const std::vector<unsigned char>&)
      */
    void doWriteVector(std::vector<unsigned char> data);
    
    /** \brief Start asynchronous write operation
      * 
      * This method initiates an asynchronous write operation and binds
      * SerialDevice::writeComplete to indicate completion or failure of
      * the operation.
      */
    void writeStart();
    
    /** \brief Indicate completion or failure of asynchronous write operations
      * 
      * The invocation of this method indicates that an asynchronous write
      * operation has either failed or completed, depending on the provided
      * error code.
      * 
      * \param[in] error The error which occurred for the device since the
      *   last call to SerialDevice::writeStart.
      * \param[in] numWritten The number of bytes written asynchronously by
      *   the device since the last call to SerialDevice::writeStart.
      */
    void writeComplete(const boost::system::error_code& error,
      size_t numWritten);

    /** \brief Perform asynchronous closing of the device
      * 
      * This method is called from the device's associated thread in order
      * to close the device.
      * 
      * \param[in] error Any pending device error which occurred prior to
      *   closing the device, e.g., during a previous read or write operation.
      */
    void doClose(const boost::system::error_code& error);
  };
};

#endif
