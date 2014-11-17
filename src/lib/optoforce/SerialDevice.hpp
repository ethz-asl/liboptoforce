#ifndef SERIALDEVICE_HPP
#define SERIALDEVICE_HPP

#include <deque>
#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#ifdef POSIX
#include <termios.h>
#endif

class SerialDevice {
public:
  SerialDevice(boost::asio::io_service& io_service, unsigned int baud, const
    std::string& device);

  void write(const char msg);

  char * return_read_data();
  int return_read_data_size(); //Returns size in bytes

  inline bool is_open() {
    return serialPort.is_open();
  };

  void close();

  bool active();
private:
  // maximum amount of data to read in one operation
  static const unsigned int max_read_length = 512;

  void read_start(void);
  void read_complete(const boost::system::error_code& error, size_t
    bytes_transferred);

  void do_write(const char msg);
  void write_start(void);
  void write_complete(const boost::system::error_code& error);

  void do_close(const boost::system::error_code& error);

  // remains true while this object is still operating
  bool active_;
  
  // the main IO service that runs this connection
  boost::asio::io_service& io_service_;
  
  // the serial port this instance is connected to
  boost::asio::serial_port serialPort;
  
  // data read from the socket
  char read_msg_[max_read_length];
  
  // data read from the socket
  char read_msg_complete_[max_read_length];
  
  unsigned int read_msg_size_; 
  
  // buffered write data
  std::deque<char> write_msgs_;
};

#endif // SERIALDEVICE_HPP
