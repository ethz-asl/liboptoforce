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

#include "SensorConfig.hpp"

#include <boost/throw_exception.hpp>

namespace optoforce {
  
/*****************************************************************************/
/* Constructors                                                              */
/*****************************************************************************/

SensorConfig::SensorConfig(State state, Speed speed, Filter filter,
    Mode mode) :
  state(state),
  speed(speed),
  filter(filter),
  mode(mode) {
}

SensorConfig::SensorConfig(const SensorConfig& src) :
  state(src.state),
  speed(src.speed),
  filter(src.filter),
  mode(src.mode) {
}

/*****************************************************************************/
/* Destructor                                                                */
/*****************************************************************************/

SensorConfig::~SensorConfig() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void SensorConfig::setSpeedHz(unsigned int speedHz) {
  if (speedHz == 1000)
    speed = speed_1000hz;
  else if (speedHz == 333)
    speed = speed_333hz;
  else if (speedHz == 100)
    speed = speed_100hz;
  else if (speedHz == 30)
    speed = speed_30hz;
  else
    BOOST_THROW_EXCEPTION(ValueError());
}

unsigned int SensorConfig::getSpeedHz() const {
  switch(speed) {
    case speed_1000hz:
      return 1000;
    case speed_333hz:
      return 333;
    case speed_100hz:
      return 100;
    case speed_30hz:
      return 30;
    default:
      return 100;
  }
}

void SensorConfig::setFilterHz(unsigned int filterHz) {
  if (filterHz == 0)
    filter = filter_none;
  else if (filterHz == 150)
    filter = filter_150hz;
  else if (filterHz == 50)
    filter = filter_50hz;
  else if (filterHz == 15)
    filter = filter_15hz;
  else
    BOOST_THROW_EXCEPTION(ValueError());
}

unsigned int SensorConfig::getFilterHz() const {
  switch(filter) {
    case filter_none:
      return 0;
    case filter_150hz:
      return 150;
    case filter_50hz:
      return 50;
    case filter_15hz:
      return 15;
    default:
      return 0;
  }
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

SensorConfig& SensorConfig::operator=(const SensorConfig& src) {
  state = src.state;
  speed = src.speed;
  filter = src.filter;
  mode = src.mode;
    
  return *this;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void SensorConfig::reset() {
  state = state_no_sensor;
  speed = speed_1000hz;
  filter = filter_none;
  mode = mode_raw;
}

}
