

#include"SerialDevice.hpp"

using namespace std;

SerialDevice::SerialDevice(boost::asio::io_service& io_service, unsigned int baud, const string& device)
: active_(true),
  io_service_(io_service),
  serialPort(io_service, device)
{
	cout << "Opening the serial port" << endl;
	if (!serialPort.is_open())
	{
		cerr << "SerialDevice::Failed to open serial port\n";
		return;
	}
	boost::asio::serial_port_base::baud_rate baud_option(baud);
	serialPort.set_option(baud_option); // set the baud rate after the port has been opened
	read_start();
}


void SerialDevice::write(const char msg) // pass the write data to the do_write function via the io service in the other thread
{

	io_service_.post(boost::bind(&SerialDevice::do_write, this, msg));
}

void SerialDevice::close() // call the do_close function via the io service in the other thread
{
	io_service_.post(boost::bind(&SerialDevice::do_close, this, boost::system::error_code()));
}

bool SerialDevice::active() // return true if the socket is still active
{
	return active_;
}


void  SerialDevice::read_start(void)
{ // Start an asynchronous read and call read_complete when it completes or fails
	try {
		serialPort.async_read_some(boost::asio::buffer(read_msg_, max_read_length),
			//Bind read_complete:
			boost::bind(&SerialDevice::read_complete,
					this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));
	}
	catch(...) {
	std::cerr <<"SerialDevice: Error at async_read_some " << std::endl;
	}


//	 size_t length = sock.read_some(boost::asio::buffer(data, cMaxLength), error);
//	                        if (error == boost::asio::error::eof)
//	                        {
//	                                // done, stop echoing, wait for another connection
//	                                break;
//	                        }

}

void  SerialDevice::read_complete(const boost::system::error_code& error, size_t bytes_transferred)
{ // the asynchronous read operation has now completed or failed and returned an error
	if (!error)
	{ // read completed, so process the data

		//Add Null-Char to allow for strlen
	  //Dangerous. Null-Char is used in the new protocol (>API 1.1)
		if(bytes_transferred<max_read_length){
			read_msg_[bytes_transferred+1] = '\0';
		}
		else {
			read_msg_[max_read_length] = '\0';
		}

		//Copy the data:
		read_msg_size_ = (unsigned int) bytes_transferred;
		memcpy(read_msg_complete_, read_msg_, read_msg_size_);

		memset(&read_msg_[0], 0, sizeof(read_msg_)); //Clear the read buffer
		read_start(); // start waiting for another asynchronous read again
	}
	else
		do_close(error);
}

char * SerialDevice::return_read_data() {
	return read_msg_complete_;
}

int SerialDevice::return_read_data_size(){
	return read_msg_size_;
}

void  SerialDevice::do_write(const char msg)
{ // callback to handle write call from outside this class
	bool write_in_progress = !write_msgs_.empty(); // is there anything currently being written?
	write_msgs_.push_back(msg); // store in write buffer
	if (!write_in_progress) // if nothing is currently being written, then start
		write_start();
}

void  SerialDevice::write_start(void)
{ // Start an asynchronous write and call write_complete when it completes or fails
	boost::asio::async_write(serialPort,
			boost::asio::buffer(&write_msgs_.front(), 1),
			boost::bind(&SerialDevice::write_complete,
					this,
					boost::asio::placeholders::error));
}

void  SerialDevice::write_complete(const boost::system::error_code& error)
{ // the asynchronous read operation has now completed or failed and returned an error
	if (!error)
	{ // write completed, so send next write data
		write_msgs_.pop_front(); // remove the completed data
		if (!write_msgs_.empty()) // if there is anthing left to be written
			write_start(); // then start sending the next item in the buffer
	}
	else
		do_close(error);
}

void  SerialDevice::do_close(const boost::system::error_code& error)
{
	if (error == boost::asio::error::operation_aborted) // if this call is the result of a timer cancel()
		return; // ignore it because the connection cancelled the timer
	if (error)
		cerr << "Error: " << error.message() << endl; // show the error message
	else
		cout << "SerialDevice:: Error: Connection did not succeed - closing the serial port.\n";
	serialPort.close();
	active_ = false;
}
