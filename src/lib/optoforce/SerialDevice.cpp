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

#include "SerialDevice.hpp"

#include <boost/bind.hpp>
#include <boost/chrono.hpp>

namespace optoforce {
  
/*****************************************************************************/
/* Constructors                                                              */
/*****************************************************************************/
  
SerialDevice::SerialDevice(size_t readBufferSize) :
  port(service),
  readBuffer(readBufferSize),
  numRead(0),
  numWritten(0) {
}
    
SerialDevice::SerialDevice(const std::string& filename, unsigned int
    baudRate, size_t readBufferSize) :
  port(service),
  readBuffer(readBufferSize),
  numRead(0),
  numWritten(0) {
  open(filename, baudRate);
}

/*****************************************************************************/
/* Destructor                                                                */
/*****************************************************************************/
  
SerialDevice::~SerialDevice() {
  if (port.is_open())
    close(true);
}
  
/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/
  
bool SerialDevice::isOpen() const {
  return port.is_open();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/
  
void SerialDevice::open(const std::string& filename, unsigned int baudRate) {
  this->filename = filename;
  port.open(filename);
  
  boost::asio::serial_port_base::baud_rate baudRateOption(baudRate);
  port.set_option(baudRateOption);
  
  numRead = 0;
  numWritten = 0;
  error.clear();
  
  readStart();
  
  service.reset();
  thread = boost::thread(
    boost::bind(&boost::asio::io_service::run, &service));
}

void SerialDevice::close(bool wait) {
  if (wait) {
    boost::unique_lock<boost::mutex> lock(closeMutex);
    service.post(boost::bind(&SerialDevice::doClose, this,
      boost::system::error_code()));
    closeCondition.wait(lock);
  }
  else
    service.post(boost::bind(&SerialDevice::doClose, this,
      boost::system::error_code()));
}
  
boost::signals2::connection SerialDevice::connectOnReadComplete(const
    OnReadComplete::slot_type& slot) {
  return onReadComplete.connect(slot);
}

void SerialDevice::write(unsigned char data) {
  service.post(boost::bind(&SerialDevice::doWrite, this, data));
}

void SerialDevice::write(const std::vector<unsigned char>& data) {
  service.post(boost::bind(&SerialDevice::doWriteVector, this, data));
}

void  SerialDevice::readStart() { 
  port.async_read_some(
    boost::asio::buffer(readBuffer),
    boost::bind(&SerialDevice::readComplete, this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
}

void SerialDevice::readComplete(const boost::system::error_code& error,
    size_t numRead) {
  if (!error) {
    boost::chrono::high_resolution_clock::time_point time =
      boost::chrono::high_resolution_clock::now();
    int64_t timestamp =
      boost::chrono::duration_cast<boost::chrono::nanoseconds>(
      time.time_since_epoch()).count();
    
    std::vector<unsigned char> readData(numRead);
    std::copy(readBuffer.begin(), readBuffer.begin()+numRead,
      readData.begin());
    this->numRead += numRead;

    onReadComplete(readData, timestamp);
    
    readStart(); 
  }
  else
    doClose(error);
}

void  SerialDevice::doWrite(unsigned char data) { 
  bool writeInProgress = !writeBuffer.empty();
  
  writeBuffer.push_back(data); 
  
  if (!writeInProgress)
    writeStart();
}

void  SerialDevice::doWriteVector(std::vector<unsigned char> data) { 
  bool writeInProgress = !writeBuffer.empty();

  writeBuffer.insert(writeBuffer.end(), data.begin(), data.end());
  
  if (!writeInProgress)
    writeStart();
}

void  SerialDevice::writeStart() { 
  boost::asio::async_write(port,
    boost::asio::buffer(&writeBuffer.front(), 1),
    boost::bind(&SerialDevice::writeComplete, this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
}

void  SerialDevice::writeComplete(const boost::system::error_code& error,
    size_t numWritten) {
  if (!error) {
    writeBuffer.erase(writeBuffer.begin(), writeBuffer.begin()+numWritten);
    this->numWritten += numWritten;
    
    if (!writeBuffer.empty())
      writeStart();
  }
  else
    doClose(error);
}

void  SerialDevice::doClose(const boost::system::error_code& error) {
  if (error == boost::asio::error::operation_aborted)
    return;
  
  port.close();
  this->error = error;    
  
  boost::lock_guard<boost::mutex> lock(closeMutex);
  closeCondition.notify_all();
}

}
