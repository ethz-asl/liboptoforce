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

/** \file StateMachine.hpp
  * \brief Header file providing the StateMachine class
  */

#ifndef OPTOFORCE_STATEMACHINE_HPP
#define OPTOFORCE_STATEMACHINE_HPP

#include <deque>
#include <cstdint>

#include <optoforce/SerialDevice.hpp>
#include <optoforce/SensorPackage.hpp>

namespace optoforce {
  /** \brief Finite state machine implementation for OptoForce sensors
    * 
    * This is the schematics of the OptoForce finite state machine
    * (API version < 1.1):
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
    /** \brief Enumerable representing the states of the state machine
      */
    enum State {
      /** Check states (all sensor versions)
        */
      state_XX_CheckH, state_XX_CheckL,

      /** .66 version states
        */
      state_66_Start, state_66_S1H, state_66_S1L, state_66_S2H, state_66_S2L,
      state_66_S3H, state_66_S3L, state_66_S4H, state_66_S4L,

      /** .67 version states
        */
      state_67_Start, state_67_Config, state_67_I1H, state_67_I1L,
      state_67_I2H, state_67_I2L, state_67_I3H, state_67_I3L, state_67_I4H,
      state_67_I4L, state_67_TH, state_67_TL, state_67_Checksum,
      
      /** .68 version states
        */
      state_68_Start, state_68_Config, state_68_FXH, state_68_FXL,
      state_68_FYH, state_68_FYL, state_68_FZH, state_68_FZL, state_68_S1H, 
      state_68_S1L, state_68_S2H, state_68_S2L, state_68_S3H, state_68_S3L,
      state_68_S4H, state_68_S4L, state_68_S1TH, state_68_S1TL, state_68_S2TH,
      state_68_S2TL, state_68_S3TH, state_68_S3TL, state_68_S4TH,
      state_68_S4TL, state_68_TH, state_68_TL, state_68_ChecksumH,
      state_68_ChecksumL,
      
      /** .94 version states
        */
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
    
    /** \brief Default constructor
      */
    StateMachine();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source state machine which is being copied to
      *   this state machine.
      */
    StateMachine(const StateMachine& src);
    
    /** \brief Destructor
      */
    ~StateMachine();

    /** \brief Set the state machine's current state
      * 
      * \param[in] state The new current state of the state machine.
      */
    inline void setCurrentState(State state) {
      currentState = state;
    };
    
    /** \brief Retrieve the state machine's current state
      * 
      * \return The current state of the state machine.
      */
    inline State getCurrentState() const {
      return currentState;
    };

    /** \brief Assignment operator
      * 
      * \param[in] src The source state machine which is being copied to
      *   this state machine.
      * \return A non-const reference to this state machine.
      */
    StateMachine& operator=(const StateMachine& src);
    
    /** \brief Process input data
      * 
      * The finite state machine processes input data character by character
      * and thus sequentially advances its state whilst outputting new sensor
      * packages now and then.
      * 
      * \param[in] data The input data to be processed by the state machine.
      * \param[in] timestamp The timestamp of the input data to be processed
      *   by the state machine. This timestamp is inherited by any sensor
      *   package which is being generated whilst processing the provided
      *   sequence of input characters.
      * \param[in,out] packages A (not necessarily empty) queue to append
      *   all sensor packages which have been generated by processing the
      *   provided sequence of input characters to.
      * \return The number of sensor packages generated by processing the
      *   provided sequence of input characters.
      */
    size_t process(const std::vector<unsigned char>& data, int64_t
      timestamp, std::deque<SensorPackage>& packages);
    
    /** \brief Restart the state machine
      * 
      * The state machine is restarted by setting its current state to the
      * start state.
      * 
      * \see setCurrentState
      */
    void restart();
    
  protected:
    /** \brief The state machine's current state
      */
    State currentState;
    
    /** \brief The state machine's current package
      * 
      * The current package represents the sensor package which is being
      * generated currently.
      */
    SensorPackage currentPackage;
    
    /** \brief The state machine's current value
      */
    unsigned short currentValue;
    
    /** \brief The state machine's current checksum (byte representation)
      */
    unsigned short currentChecksumByte;
    
    /** \brief The state machine's current checksum (word representation)
      */
    unsigned short currentChecksumWord;
  };
};

#endif
