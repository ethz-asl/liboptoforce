#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

/* 2013/mh: Adapted the original file from Optoforce :
 * - Removed the QT stuff
 * - Added functionality to configure the sampling frequency and the filter
 *
 * 2014/mh: Adapted to new version of the optoforce API (API 1.1)
 *  - Added enums for state_machine-type

 */

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include <optoforce/SerialDevice.hpp>
#include <optoforce/DisplayPackage.hpp>

enum version {
  _66, _67, _68
};

/* The fsm (v < 1.1):
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

class StateMachine {
  private:
  enum fsm_state {
    /* Check states */
    first_check, second_check,

    /* .66 Version states */
    _66_Start, _66_S1H, _66_S1L, _66_S2H, _66_S2L, _66_S3H, _66_S3L, _66_S4H,
    _66_S4L,

    /* .67 Version states */
    _67_Start, _67_Config, _67_I1H, _67_I1L, _67_I2H, _67_I2L, _67_I3H,
    _67_I3L, _67_I4H, _67_I4L, _67_TH, _67_TL, _67_Checksum,
    
    /* .68 Version states */
    _68_Start, _68_Config, _68_FXH, _68_FXL, _68_FYH, _68_FYL, _68_FZH,
    _68_FZL, _68_S1H, _68_S1L, _68_S2H, _68_S2L, _68_S3H, _68_S3L, _68_S4H,
    _68_S4L, _68_S1TH, _68_S1TL, _68_S2TH, _68_S2TL, _68_S3TH, _68_S3TL,
    _68_S4TH, _68_S4TL, _68_TH, _68_TL, _68_ChecksumH, _68_ChecksumL,
    
    /* .94 Version states */
    _94_Start, _94_Config, _94_I1H, _94_I1L, _94_I2H, _94_I2L, _94_I3H,
    _94_I3L, _94_I4H, _94_I4L, _94_S1TH, _94_S1TL, _94_I5H, _94_I5L, _94_I6H,
    _94_I6L, _94_I7H, _94_I7L, _94_I8H, _94_I8L, _94_S2TL, _94_S2TH, _94_I9H,
    _94_I9L, _94_I10H, _94_I10L, _94_I11H, _94_I11L, _94_I12H, _94_I12L,
    _94_S3TH, _94_S3TL, _94_I13H, _94_I13L, _94_I14H, _94_I14L, _94_I15H,
    _94_I15L, _94_I16H, _94_I16L, _94_S4TH, _94_S4TL, _94_Checksum
  };
  
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
public:
    /* constructor */
    StateMachine ();

    /* try open a port with portname */
    bool open (std::string device, int baudrate);

    /* close the port */
    void close ();

    bool is_open ();

    /* read and processing all data from input */
    void read(std::deque<sp::DisplayPackage>& packs, const sp::DisplayPackage&
      offset);

    /* Added by markho@ethz.ch to allow for configuration of the sensor
    // possible values: frequency_hz_sampling: 1000, 333, 100, 10
    //  frequency_hz_filter:  0 (no filter - default), 150, 50, 15
    // Raw == 1: Raw data streaming (default), Raw == 0: Force (not working?) 
    */
    int write_configuration(int frequency_hz_sampling, int frequency_hz_filter,
      int flag_raw);
};

#endif // STATEMACHINE_HPP
