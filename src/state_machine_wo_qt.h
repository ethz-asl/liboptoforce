#ifndef STATE_MACHINE_WO_QT_H
#define STATE_MACHINE_WO_QT_H

/* Adapted the original file from Optoforce:
 * - Removed the QT stuff
 * - Added functionality to configure the sampling frequency and the filter
 */

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include"SerialDevice.hpp"

#include "helper.h"



enum version {
    _66, _67
};

/* The fsm:
 * --------
 *
 *
 *
 *   ,--------------------,       byte == 55        ,---------------------,
 *   |  first check byte  | ----------------------> |  second check byte  |----,
 *   `--------------------`                         `---------------------`    |
 *       ^               |                                  |   |              |
 *       |  byte != 55   |        byte != 66 or 67          |   | byte == 66   |
 *       `---------------' <--------------------------------`   |              |
 *                                                              |              | byte == 67
 *                                                              |              |
 *                   .66 version processing <-------------------`              |
 *                      * x, y, z coors                                        |
 *                                                .67 version processing <-----`
 *                                                   * x, y, z coords
 *                                                   * temperature
 *                                                   * checksum
 *
 */


class state_machine { // : public QObject {

  //    Q_OBJECT

private:

    /* input serial port */
  //  QextSerialPort sp;

    /* current state */
    int state;

    /* current character in processing */
    uint8_t c;

    /* first valid input -> signal first_valid_read () */
    bool first_valid;

    /* OMD version */
    version vs;


    /*Serial stuff, mh */
    boost::asio::io_service io_service_;
    unsigned int baudrate_;  //Baudrate in [Bits per second];
    std::string device_;
    SerialDevice * serialDevice_;
    
    //signals:

    /* correct processing in fns */
    //void first_valid_read ();

public:

    /* constructor */
    state_machine ();

    /* try open a port with portname */
    bool open (std::string device, int baudrate);//const QString& portname);

    /* close the port */
    void close ();

    bool is_open ();

    /* read and processing all data from input */
    void read(std::deque<sp::display_package>& packs, const sp::display_package& offset);
    // void read (void);

    /* Added by markho@ethz.ch to allow for configuration of the sensor
    // possible values: frequency_hz_sampling: 1000, 333, 100, 10
    //  frequency_hz_filter:  0 (no filter - default), 150, 50, 15
    // Raw == 1: Raw data streaming (default), Raw == 0: Force (not working?) 
    */
    int writeConfiguration(int frequency_hz_sampling, int frequency_hz_filter, int flag_raw);


};

#endif // STATE_MACHINE_WO_QT_H
