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

#include "Sensor.hpp"

#include <boost/bind.hpp>

namespace optoforce {

/*****************************************************************************/
/* Constructors                                                              */
/*****************************************************************************/
  
Sensor::Sensor(size_t packageBufferSize, double signalToForceFactor) :
  packageBufferSize(packageBufferSize),
  signalToForceFactor(signalToForceFactor),
  numReceivedPackages(0),
  numDroppedPackages(0),
  numCalibrationReadings(0) {
  serialDevice.connectOnReadComplete(
    boost::bind(&Sensor::processReadData, this, _1, _2));
}

/*****************************************************************************/
/* Destructor                                                                */
/*****************************************************************************/
  
Sensor::~Sensor() {
  if (isConnected())
    disconnect(true);
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/
  
SensorPackage Sensor::getPackage(BufferPosition position) const {
  boost::lock_guard<boost::mutex> lock(mutex);
  
  if (!packageBuffer.empty()) {
    if (position == buffer_position_newest)
      return packageBuffer.back();
    else
      return packageBuffer.front();
  }
  else
    BOOST_THROW_EXCEPTION(NoSuchPackageError());
}

SensorReading Sensor::getReading(BufferPosition position, bool compensated)
    const {
  SensorPackage package = getPackage(position);
  
  SensorReading reading;
  if (compensated) {
    package.toCompensatedReading(reading, signalToForceFactor);
    reading -= compensatedZeroWeightOffset;
  }
  else {
    package.toReading(reading, signalToForceFactor);
    reading -= zeroWeightOffset;
  }
  
  return reading;
}
  
void Sensor::setPackageBufferSize(size_t packageBufferSize) {
  boost::lock_guard<boost::mutex> lock(mutex);
  
  this->packageBufferSize = packageBufferSize;

  while (packageBuffer.size() > this->packageBufferSize) {
    packageBuffer.pop_front();
    ++numDroppedPackages;
  }
}

bool Sensor::isConnected() const {
  return serialDevice.isOpen();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/
  
void Sensor::connect(const std::string& deviceFilename, unsigned int
    deviceBaudRate) {
  serialDevice.open(deviceFilename, deviceBaudRate);
  
  packageBuffer.clear();
  stateMachine.restart();
  
  numReceivedPackages = 0;
  numDroppedPackages = 0;
}

void Sensor::disconnect(bool wait) {
  serialDevice.close(wait);
}

void Sensor::configure(const SensorConfig& config) {
  if (hasPackages()) {
    if (getPackage().version == SensorPackage::version_170) {
      unsigned char speed, filter;
      switch (config.getSpeed()) {
        case SensorConfig::speed_1000hz:
          speed = 1;
          break;
        case SensorConfig::speed_333hz:
          speed = 3;
          break;
        case SensorConfig::speed_100hz:
          speed = 10;
          break;
        case SensorConfig::speed_30hz:
          speed = 33;
          break;
      }
      switch (config.getFilter()) {
        case SensorConfig::filter_none:
          filter = 0;
          break;
        case SensorConfig::filter_150hz:
          filter = 2;
          break;
        case SensorConfig::filter_50hz:
          filter = 3;
          break;
        case SensorConfig::filter_15hz:
          filter = 4;
          break;
      }
      printf("[DEBUG] writing config speed=%d filter=%d\n", speed, filter);
      unsigned short sum = 170 + 50 + 3 + (unsigned short)speed + (unsigned short)filter;
      serialDevice.write({170, 0, 50, 3, speed, filter, 0, (sum & 0xFF00) >> 8, sum & 0x00FF});
    } else {
      serialDevice.write(config.toByte());
    }
  }
}

void Sensor::calibrate(size_t numReadings) {
  boost::lock_guard<boost::mutex> lock(mutex);

  zeroWeightOffset.setForce(0.0, 0.0, 0.0);
  zeroWeightOffset.clearTorque();
  zeroWeightAccumulator = Accumulator();
  
  compensatedZeroWeightOffset.setForce(0.0, 0.0, 0.0);
  compensatedZeroWeightOffset.clearTorque();
  compensatedZeroWeightAccumulator = Accumulator();
  
  numCalibrationReadings = numReadings;
}

SensorPackage Sensor::dequeuePackage(BufferPosition position) {
  boost::lock_guard<boost::mutex> lock(mutex);
  
  if (!packageBuffer.empty()) {
    SensorPackage package;
    
    if (position == buffer_position_newest) {
      package = packageBuffer.back();
      packageBuffer.pop_back();
    }
    else {
      package = packageBuffer.front();
      packageBuffer.pop_front();
    }
    
    return package;
  }
  else
    BOOST_THROW_EXCEPTION(NoSuchPackageError());
}

SensorReading Sensor::dequeueReading(BufferPosition position, bool
    compensated) {
  SensorPackage package = dequeuePackage(position);
  
  SensorReading reading;
  if (compensated) {
    package.toCompensatedReading(reading, signalToForceFactor);
    reading -= compensatedZeroWeightOffset;
  }
  else {
    package.toReading(reading, signalToForceFactor);
    reading -= zeroWeightOffset;
  }
  
  return reading;
}

void Sensor::clearPackages() {
  boost::lock_guard<boost::mutex> lock(mutex);
  packageBuffer.clear();
}

void Sensor::requestConfig() {
  if (hasPackages()) {
      if (getPackage().version == SensorPackage::version_170) {
          printf("[DEBUG] writing config request\n");
          serialDevice.write({171, 0, 50, 6, 0, 227});
      }
  }
}

void Sensor::processReadData(const std::vector<unsigned char>& data,
    int64_t timestamp) {

  printf("[DEBUG] processReadData([");
  for (int i = 0; i < data.size(); ++i) printf("0x%02X%s", data[i], i == data.size()-1 ? "])\n" : ", ");

  if (!data.empty()) {
    boost::lock_guard<boost::mutex> lock(mutex);
    
    size_t numReceivedPackages = stateMachine.process(data, timestamp,
      packageBuffer);
    this->numReceivedPackages += numReceivedPackages;
    
    if (numCalibrationReadings) {
      size_t numCalibrationReadings = boost::accumulators::count(
        zeroWeightAccumulator);
      SensorReading reading;
      
      for (std::deque<SensorPackage>::const_iterator it =
          packageBuffer.end()-numReceivedPackages;
          (numReceivedPackages > 0) &&
          (numCalibrationReadings < this->numCalibrationReadings);
          ++it, --numReceivedPackages, ++numCalibrationReadings) {
        if (it->hasCompensatedSignals()) {
          it->toCompensatedReading(reading, signalToForceFactor);
          compensatedZeroWeightAccumulator(reading);
        }

        it->toReading(reading, signalToForceFactor);
        zeroWeightAccumulator(reading);
      }

      if (this->numCalibrationReadings == numCalibrationReadings) {
        zeroWeightOffset = boost::accumulators::mean(
          zeroWeightAccumulator);
        compensatedZeroWeightOffset = boost::accumulators::mean(
          compensatedZeroWeightAccumulator);
        
        this->numCalibrationReadings = 0;
      }
    }

    while (packageBuffer.size() > packageBufferSize) {
      packageBuffer.pop_front();
      ++numDroppedPackages;
    }
  }
}

}
