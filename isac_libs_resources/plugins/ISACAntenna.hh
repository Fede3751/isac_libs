/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef ODOMETER_HH_
#define ODOMETER_HH_

#include <ignition/sensors/Sensor.hh>
#include <ignition/sensors/SensorTypes.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/Entity.hh>

namespace custom
{
  /// \brief Example sensor that publishes the total distance traveled by a
  /// robot, with noise.
  class ISACAntenna : public ignition::sensors::Sensor
  {
    public:

      ISACAntenna();

      // Topics
      std::string tx_topic_name = "";
      std::string rx_topic_name = "";

      ignition::transport::Node::Publisher  rxPub;
      ignition::transport::Node::Publisher  txPub;

      // Model info
      std::string                           modelName;
      ignition::gazebo::Entity              modelEntity;
      ignition::gazebo::Entity              sensorEntity;

      // Position (updated externally)
      ignition::math::Vector3d position;

      // PHY / channel parameters - Default values if not specified
      double txPower_dBm      = 20.0; // transmission power
      double pathLossExponent = 3.5;  // log-distance exponent
      double fadingStd        = 6;    // fading std dev
      double bandwidth_Hz     = 1e6;  // channel bandwidth
      double noiseFigure_dB   = 5.0;  // receiver noise figure


      virtual bool                      Load(const sdf::Sensor &_sdf) override;
      virtual bool                      Update(const std::chrono::steady_clock::duration &_now) override;
      void                              NewPosition(const ignition::math::Vector3d &_pos);
      const ignition::math::Vector3d   &Position() const;
      void                              TxCallback(const ignition::msgs::StringMsg &msg);    
    
    private:
      std::unique_ptr<ignition::transport::Node> node;

      ignition::math::Vector3d prevPos{
        std::nan(""),
        std::nan(""),
        std::nan("")
      };

    
  };
}

#endif