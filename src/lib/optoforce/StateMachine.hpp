#ifndef OPTOFORCE_STATEMACHINE_HPP
#define OPTOFORCE_STATEMACHINE_HPP

/* 2013/mh: Adapted the original file from Optoforce :
 * - Removed the QT stuff
 * - Added functionality to configure the sampling frequency and the filter
 *
 * 2014/mh: Adapted to new version of the optoforce API (API 1.1)
 * - Added enums for state_machine-type
 */

#include <deque>
#include <cstdint>

#include <optoforce/SerialDevice.hpp>
#include <optoforce/SensorPackage.hpp>

namespace optoforce {
  /** The finite state machine (API version < 1.1):
    *
    \verbatim
    ,--------------------,       byte == 55        ,---------------------,
    |  first check byte  | ----------------------> |  second check byte  |----,
    `--------------------`                         `---------------------`    |
        ^               |                                  |   |              |
        |  byte != 55   |        byte != 66 or 67          |   | byte == 66   |
        `---------------' <--------------------------------`   |              |
                                                               |              | byte == 67
                                                               |              |
                    .66 version processing <-------------------`              |
                       * x, y, z coors                                        |
                                                 .67 version processing <-----`
                                                     * x, y, z coords
                                                     * temperature
                                                     * checksum
    \endverbatim
    *
    */  
  class StateMachine {
  public:
    /* States */
    enum State {
      /* Check states (all versions) */
      state_XX_CheckH, state_XX_CheckL,

      /* .66 version states */
      state_66_Start, state_66_S1H, state_66_S1L, state_66_S2H, state_66_S2L,
      state_66_S3H, state_66_S3L, state_66_S4H, state_66_S4L,

      /* .67 version states */
      state_67_Start, state_67_Config, state_67_I1H, state_67_I1L,
      state_67_I2H, state_67_I2L, state_67_I3H, state_67_I3L, state_67_I4H,
      state_67_I4L, state_67_TH, state_67_TL, state_67_Checksum,
      
      /* .68 version states */
      state_68_Start, state_68_Config, state_68_FXH, state_68_FXL,
      state_68_FYH, state_68_FYL, state_68_FZH, state_68_FZL, state_68_S1H, 
      state_68_S1L, state_68_S2H, state_68_S2L, state_68_S3H, state_68_S3L,
      state_68_S4H, state_68_S4L, state_68_S1TH, state_68_S1TL, state_68_S2TH,
      state_68_S2TL, state_68_S3TH, state_68_S3TL, state_68_S4TH,
      state_68_S4TL, state_68_TH, state_68_TL, state_68_ChecksumH,
      state_68_ChecksumL,
      
      /* .94 version states */
      state_94_Start, state_94_Config, state_94_I1H, state_94_I1L,
      state_94_I2H, state_94_I2L, state_94_I3H, state_94_I3L, state_94_I4H, 
      state_94_I4L, state_94_S1TH, state_94_S1TL, state_94_I5H, state_94_I5L,
      state_94_I6H, state_94_I6L, state_94_I7H, state_94_I7L, state_94_I8H,
      state_94_I8L, state_94_S2TL, state_94_S2TH, state_94_I9H, state_94_I9L,
      state_94_I10H, state_94_I10L, state_94_I11H, state_94_I11L,
      state_94_I12H, state_94_I12L, state_94_S3TH, state_94_S3TL,
      state_94_I13H, state_94_I13L, state_94_I14H, state_94_I14L,
      state_94_I15H, state_94_I15L, state_94_I16H, state_94_I16L,
      state_94_S4TH, state_94_S4TL, state_94_Checksum
    };
    
    StateMachine();
    StateMachine(const StateMachine& src);
    ~StateMachine();

    inline void setCurrentState(State state) {
      currentState = state;
    };
    
    inline State getCurrentState() const {
      return currentState;
    };

    StateMachine& operator=(const StateMachine& src);
    
    /* Process data */
    size_t process(const std::vector<unsigned char>& data, int64_t
      timestamp, std::deque<SensorPackage>& packages);
    
    /* Restart state machine */
    void restart();    
  protected:
    /* Current state */
    State currentState;
    
    /* Current package */
    SensorPackage currentPackage;
    
    /* Current value */
    unsigned short currentValue;
    
    /* Current checksum byte */
    unsigned short currentChecksumByte;
    /* Current checksum word */
    unsigned short currentChecksumWord;
  };
};

#endif // OPTOFORCE_STATEMACHINE_HPP
