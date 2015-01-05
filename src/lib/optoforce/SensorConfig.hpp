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

/** \file SensorConfig.hpp
  * \brief Header file providing the SensorConfig class
  */

#ifndef OPTOFORCE_SENSORCONFIG_HPP
#define OPTOFORCE_SENSORCONFIG_HPP

#include <exception>

#include <boost/exception/exception.hpp>

namespace optoforce {
  
  /** \brief Implementation of the OptoForce sensor configuration
    * 
    * This class provides the configuration parameters of an OptoForce
    * sensor.
    */
  
  class SensorConfig {
  public:
    /** \brief Enumerable representing the state of the sensor
      */
    enum State {
      /** Sensor not present
        */
      state_no_sensor = 0,
      /** Overload of the sensor in X-direction
        */
      state_overload_x = 1,
      /** Overload of the sensor in Y-direction
        */
      state_overload_y = 2,
      /** Overload of the sensor in Z-direction
        */
      state_overload_z = 3,
      /** Sensor failure
        */
      state_sensor_failure = 4,
      /** Sensor okay
        */
      state_sensor_ok = 5,
      /** Sensor connection failure
        */
      state_connection_failure = 6
    };

    /** \brief Enumerable representing the sampling speed of the sensor
      */
    enum Speed {
      /** 1000 packages/s
        */
      speed_1000hz = 0,
      /** 333 packages/s
        */
      speed_333hz = 1,
      /** 100 packages/s
        */
      speed_100hz = 2,
      /** 30 packages/s
        */
      speed_30hz = 3
    };

    /** \brief Enumerable representing the filtering frequency of the sensor
      */
    enum Filter {
      /** Filtering disabled
        */
      filter_none = 0,
      /** Filtering at 150 Hz
        */
      filter_150hz = 1,
      /** Filtering at 50 Hz
        */
      filter_50hz = 2,
      /** Filtering at 15 Hz
        */
      filter_15hz = 3
    };

    /** \brief Enumerable representing the sensor mode
      */
    enum Mode {
      /** Packages contain raw values
        */
      mode_raw = 0,
      /** Packages contain calculated force values
        */
      mode_force = 1
    };
    
    /** \brief Error indicating unexpected configuration values
      */
    struct ValueError : virtual std::exception, virtual boost::exception {};  
    
    /** \brief Default constructor
      * 
      * \param[in] state The sensor state.
      * \param[in] speed The sensor's sampling speed.
      * \param[in] filter The sensor's filtering frequency.
      * \param[in] mode The sensor mode.
      */
    SensorConfig(State state = state_no_sensor, Speed speed = speed_1000hz,
      Filter filter = filter_none, Mode mode = mode_raw);
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source sensor configuration which is being copied
      *   to this sensor configuration.
      */
    SensorConfig(const SensorConfig& src);
    
    /** \brief Destructor
      */
    ~SensorConfig();

    /** \brief Set the sensor state
      * 
      * \param[in] state The new value of the configuration parameter
      *   representing the sensor state.
      */
    inline void setState(State state) {
      this->state = state;
    };
    
    /** \brief Retrieve the sensor state
      * 
      * \return The value of the configuration parameter representing
      *   the sensor state.
      */
    inline State getState() const {
      return state;
    };
    
    /** \brief Set the sensor's sampling speed
      * 
      * \param[in] speed The new value of the configuration parameter
      *   representing the sensor's sampling speed.
      */
    inline void setSpeed(Speed speed) {
      this->speed = speed;
    };
    
    /** \brief Retrieve the sensor's sampling speed
      * 
      * \return The value of the configuration parameter representing the
      *   sensor's sampling speed.
      */
    inline Speed getSpeed() const {
      return speed;
    };
    
    /** \brief Set the sensor's sampling speed in [Hz]
      * 
      * \param[in] speedHz The sampling speed in [Hz] which determines the
      *   new value of the configuration parameter representing the sensor's
      *   sampling speed.
      * 
      * \note If the sampling speed is not supported, ValueError will be
      *   thrown.
      * 
      * \see setSpeed
      */
    void setSpeedHz(unsigned int speedHz);
    
    /** \brief Retrieve the sensor's sampling speed in [Hz]
      * 
      * \return The sampling speed in [Hz] as determined by the value of the
      *   configuration parameter representing the sensor's sampling speed.
      * 
      * \see getSpeed
      */
    unsigned int getSpeedHz() const;
    
    /** \brief Set the sensor's filtering frequency
      * 
      * \param[in] filter The new value of the configuration parameter
      *   representing the sensor's filtering frequency.
      */
    inline void setFilter(Filter filter) {
      this->filter = filter;
    };
    
    /** \brief Retrieve the sensor's filtering frequency
      * 
      * \return The value of the configuration parameter representing the
      *   sensor's filtering frequency.
      */
    inline Filter getFilter() const {
      return filter;
    };
    
    /** \brief Set the sensor's filtering frequency in [Hz]
      * 
      * \param[in] filterHz The filtering frequency in [Hz] which determines
      *   the new value of the configuration parameter representing the
      *   sensor's filtering frequency.
      * 
      * \note If the filtering frequency is not supported, ValueError will be
      *   thrown.
      * 
      * \see setFilter
      */
    void setFilterHz(unsigned int filterHz);
    
    /** \brief Retrieve the sensor's filtering frequency in [Hz]
      * 
      * \return The filtering frequency in [Hz] as determined by the value
      *   of the configuration parameter representing the sensor's filtering
      *   frequency.
      * 
      * \see getFilter
      */
    unsigned int getFilterHz() const;
    
    /** \brief Set the sensor mode
      * 
      * \param[in] mode The new value of the configuration parameter
      *   representing the sensor mode.
      */
    inline void setMode(Mode mode) {
      this->mode = mode;
    };
    
    /** \brief Retrieve the sensor mode
      * 
      * \return The value of the configuration parameter representing
      *   the sensor mode.
      */
    inline Mode getMode() const {
      return mode;
    };
    
    /** \brief Assignment operator
      * 
      * \param[in] src The source sensor configuration which is being
      *   copied to this sensor configuration.
      * \return A non-const reference to this sensor configuration.
      */
    SensorConfig& operator=(const SensorConfig& src);
    
    /** \brief Convert byte to sensor configuration
      * 
      * \param[in] byte The byte representation which is being converted to
      *   this sensor configuration.
      */
    inline void fromByte(unsigned char byte) {
      int i = byte;
      *this = *(SensorConfig*)(&i);
    };
    
    /** \brief Convert sensor configuration to byte
      * 
      * \return The byte representation which results from converting this
      *   sensor configuration.
      */
    inline unsigned char toByte() const {
      return *(unsigned char*)this;
    };
    
    /** \brief Reset the sensor configuration
      * 
      * This method re-initializes all configuration parameters to their
      * default values.
      */
    void reset();
    
  protected:
    /** \brief The sensor mode
      */
    Mode mode: 1;
    
    /** \brief The sensor's filtering frequency
      */
    Filter filter: 2;
    
    /** \brief The sensor's sampling speed
      */
    Speed speed: 2;
    
    /** \brief The sensor state
      */
    State state: 3;
  };
};

#endif
