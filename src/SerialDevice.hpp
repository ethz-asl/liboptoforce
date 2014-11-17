#ifndef SERIALDEVICE_H
#define SERIALDEVICE_H



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

using namespace std;


class SerialDevice
{
public:
	SerialDevice(boost::asio::io_service& io_service, unsigned int baud, const string& device);

	void write(const char msg);

	char * return_read_data();
	int return_read_data_size(); //Returns size in bytes

	bool is_open() {return serialPort.is_open();}

	void close();

	bool active();

private:

	static const unsigned int max_read_length = 512; // maximum amount of data to read in one operation

	void read_start(void);
	void read_complete(const boost::system::error_code& error, size_t bytes_transferred);

	void do_write(const char msg);
	void write_start(void);
	void write_complete(const boost::system::error_code& error);

	void do_close(const boost::system::error_code& error);

	bool active_; // remains true while this object is still operating
	boost::asio::io_service& io_service_; // the main IO service that runs this connection
	boost::asio::serial_port serialPort; // the serial port this instance is connected to
	char read_msg_[max_read_length]; // data read from the socket
	char read_msg_complete_[max_read_length]; // data read from the socket
	unsigned int  read_msg_size_;   deque<char> write_msgs_; // buffered write data
};


#endif // SERIALDEVICE_H
