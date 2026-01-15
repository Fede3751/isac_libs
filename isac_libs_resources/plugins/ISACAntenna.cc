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

#include <math.h>

#include <ignition/msgs/stringmsg.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/sensors/Noise.hh>
#include <ignition/sensors/Util.hh>

#include "ISACAntenna.hh"

using namespace custom;

ISACAntenna::ISACAntenna() {
  this->node = std::make_unique<ignition::transport::Node>();
}


//////////////////////////////////////////////////
bool ISACAntenna::Load(const sdf::Sensor &_sdf) {
  auto type = ignition::sensors::customType(_sdf);
  if ("ISACAntenna" != type) {
    ignerr << "Trying to load [ISACAntenna] sensor, but got type [" << type
           << "] instead." << std::endl;
    return false;
  }

  // Load common sensor params
  ignition::sensors::Sensor::Load(_sdf);

  if (!_sdf.Element()->HasElement("ignition:ISACAntenna")) {
    igndbg << "No custom configuration for [" << this->Topic() << "]" << std::endl;
    return true;
  }

  // Load custom sensor params
  auto customElem = _sdf.Element()->GetElement("ignition:ISACAntenna");

  // TX power
  if (customElem->HasElement("tx_power_dBm"))
      this->txPower_dBm = customElem->Get<double>("tx_power_dBm");
  // Path loss exponent
  if (customElem->HasElement("path_loss_exponent"))
      this->pathLossExponent = customElem->Get<double>("path_loss_exponent");
  // Fading standard deviation
  if (customElem->HasElement("fading_std"))
      this->fadingStd = customElem->Get<double>("fading_std");
  // Bandwidth
  if (customElem->HasElement("bandwidth_Hz"))
      this->bandwidth_Hz = customElem->Get<double>("bandwidth_Hz");
  // Noise figure
  if (customElem->HasElement("noise_figure_dB"))
      this->noiseFigure_dB = customElem->Get<double>("noise_figure_dB");

  return true;
}

//////////////////////////////////////////////////
bool ISACAntenna::Update(const std::chrono::steady_clock::duration &_now)
{
  return true;
}

//////////////////////////////////////////////////
void ISACAntenna::NewPosition(const ignition::math::Vector3d &_pos)
{
  this->position = _pos;
}

//////////////////////////////////////////////////
const ignition::math::Vector3d &ISACAntenna::Position() const
{
  return this->position;
}


void ISACAntenna::TxCallback(const ignition::msgs::StringMsg &msg) {

}  