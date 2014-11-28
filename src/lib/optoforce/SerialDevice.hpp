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

#ifndef OPTOFORCE_SERIALDEVICE_HPP
#define OPTOFORCE_SERIALDEVICE_HPP

#include <vector>
#include <deque>
#include <string>
#include <cstdint>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/signals2.hpp>

namespace optoforce {
  /** Serial communication device */
  class SerialDevice {
  public:
    typedef boost::signals2::signal
      <void (const std::vector<unsigned char>&, int64_t)> OnReadComplete;
    
    SerialDevice(size_t readBufferSize = 512);
    SerialDevice(const std::string& filename, unsigned int baudRate = 115200,
      size_t readBufferSize = 512);
    ~SerialDevice();
    
    // Return the filename of the serial device
    inline const std::string& getFilename() const {
      return filename;
    };
    
    // Return the number of bytes read from socket
    inline size_t getNumRead() const {
      return numRead;
    };
    
    // Return the number of bytes written to socket
    inline size_t getNumWritten() const {
      return numWritten;
    };
    
    // Return true if the serial port is open
    bool isOpen() const;
    
    // Return true if the socket is still active
    inline bool isActive() const {
      return active;
    };
    
    // Return the most recent error
    inline const boost::system::error_code& getError() const {
      return error;
    };

    void open(const std::string& filename, unsigned int baudRate = 115200);
    void close(bool wait = false);
    
    boost::signals2::connection connectOnReadComplete(const
      OnReadComplete::slot_type& slot);
    
    // Write data
    void write(unsigned char data);
    void write(const std::vector<unsigned char>& data);
  protected:
    // Remains true while this object is still operating
    bool active;
    
    // Name of the serial device
    std::string filename;
    
    // The I/O service that runs this connection
    boost::asio::io_service service;
    
    // The thread that runs this connection
    boost::thread thread;
    
    // The serial port this instance is connected to
    boost::asio::serial_port port;

    // Buffer for data read from the socket
    std::vector<unsigned char> readBuffer;
    
    // Buffer for data to be written to socket
    std::deque<unsigned char> writeBuffer;
    
    // Read complete signal
    OnReadComplete onReadComplete;
    
    // Condition signaling socket closed
    boost::condition_variable closeCondition;
    // Mutex for signaling socket closed
    boost::mutex closeMutex;

    // Most recent error
    boost::system::error_code error;
    
    // Number of bytes read from socket
    size_t numRead;
    // Number of bytes written to socket
    size_t numWritten;
    
    void doClearReadData();
    void readStart();
    void readComplete(const boost::system::error_code& error,
      size_t numRead);

    void doWrite(unsigned char data);
    void doWriteVector(std::vector<unsigned char> data);
    void writeStart();
    void writeComplete(const boost::system::error_code& error,
        size_t numWritten);

    void doClose(const boost::system::error_code& error);
  };
};

#endif // OPTOFORCE_SERIALDEVICE_HPP
