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

#ifndef OPTOFORCE_SENSORCONFIG_HPP
#define OPTOFORCE_SENSORCONFIG_HPP

#include <exception>

#include <boost/exception/exception.hpp>

namespace optoforce {
  /* Configuration class */
  class SensorConfig {
  public:
    /* Configuration value error */
    struct ValueError : virtual std::exception, virtual boost::exception {};  
    
    /* State for checking correct functioning of sensor */
    enum State {
      state_no_sensor = 0,
      state_overload_x = 1,
      state_overload_y = 2,
      state_overload_z = 3,
      state_sensor_failure = 4,
      state_sensor_ok = 5,
      state_connection_failure = 6
    };

    /* Sending data speed */
    enum Speed {
      speed_1000hz = 0,
      speed_333hz = 1,
      speed_100hz = 2,
      speed_30hz = 3
    };

    /* Filter mode */
    enum Filter {
      filter_none = 0,
      filter_150hz = 1,
      filter_50hz = 2,
      filter_15hz = 3
    };

    /* Sending mode */
    enum Mode {
      mode_raw = 0,
      mode_force = 1
    };
    
    SensorConfig(State state = state_no_sensor, Speed speed = speed_1000hz,
      Filter filter = filter_none, Mode mode = mode_raw);
    SensorConfig(const SensorConfig& src);
    ~SensorConfig();

    inline void setState(State state) {
      this->state = state;
    };
    
    inline State getState() const {
      return state;
    };
    
    inline void setSpeed(Speed speed) {
      this->speed = speed;
    };
    
    inline Speed getSpeed() const {
      return speed;
    };
    
    void setSpeedHz(unsigned int speedHz);
    unsigned int getSpeedHz() const;
    
    inline void setFilter(Filter filter) {
      this->filter = filter;
    };
    
    inline Filter getFilter() const {
      return filter;
    };
    
    void setFilterHz(unsigned int filterHz);
    unsigned int getFilterHz() const;
    
    inline void setMode(Mode mode) {
      this->mode = mode;
    };
    
    inline Mode getMode() const {
      return mode;
    };
    
    SensorConfig& operator=(const SensorConfig& src);
    
    inline void fromByte(unsigned char byte) {
      int i = byte;
      *this = *(SensorConfig*)(&i);
    };
    
    inline unsigned char toByte() const {
      return *(unsigned char*)this;
    };
    
    void reset();
  protected:
    Mode mode: 1;
    Filter filter: 2;
    Speed speed: 2;
    State state: 3;
  };
};

#endif // OPTOFORCE_SENSORCONFIG_HPP
