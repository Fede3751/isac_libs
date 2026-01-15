#include <cstdlib>
#include <cmath>
#include <random>
#include <algorithm>


#include <ignition/gazebo/components/CustomSensor.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Sensor.hh>
#include <sdf/Box.hh>
#include <ignition/sensors/SensorFactory.hh>

#include <ignition/sensors/Sensor.hh>
#include <ignition/sensors/SensorTypes.hh>
#include <ignition/transport/Node.hh>
#include <ignition/sensors/Util.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>





#include "ISACAntenna.hh"
#include "ISACAntennaSystem.hh"


using namespace custom;


ISACAntennaSystem::ISACAntennaSystem() : rng(std::random_device{}()) {

  this->node            =   std::make_unique<ignition::transport::Node>();
  this->obstacles       =   std::vector<Obstacle>();
  this->txMessageBuffer =   std::vector<TxMessage>();
  this->topicSensorMap  =   std::unordered_map<std::string, ISACAntenna *>();
  this->entitySensorMap =   std::unordered_map <
                                ignition::gazebo::Entity,
                                std::shared_ptr<ISACAntenna>
                            >();

}


void ISACAntennaSystem::PreUpdate(const ignition::gazebo::UpdateInfo &,
                                 ignition::gazebo::EntityComponentManager &_ecm) {

      /* The PreUpdate function is the heart of the SystemPlugin, two main
         behaviors are defined here.
         - The first one handles newly spawned sensors, and is used to populate 
           the structures used by the SystemPlugin with the new entries.
           It is used both for sensors and for collision items such as walls.
         - The second is used to consume the TxBuffer, which allows for messages
           to be forwarded to rx interfaces of the antennas in the simulation.
           tx-rx forwarding is handled here because of the need to access entities
           from the entity component manager.
      */
      
      this->HandleNewEntities(_ecm);
      this->ProcessTxBuffer(_ecm);
      
}


void ISACAntennaSystem::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                  const ignition::gazebo::EntityComponentManager &_ecm) {
  if (!_info.paused) {
    for (auto &[entity, sensor] : this->entitySensorMap) {
      auto pos = ignition::gazebo::worldPose(entity, _ecm).Pos();
      //this->antennas[entity].position = pos;
      sensor->NewPosition(pos);
      sensor->Update(_info.simTime);
    }
  }
  this->RemoveSensorEntities(_ecm);
}


void ISACAntennaSystem::RemoveSensorEntities(
    const ignition::gazebo::EntityComponentManager &_ecm) {
  
  _ecm.EachRemoved<ignition::gazebo::components::CustomSensor>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::CustomSensor *)->bool {

        this->entitySensorMap.erase(_entity);
        return true;
      });

}


double ISACAntennaSystem::randGaussian(double mean, double stddev) {
    std::normal_distribution<double> dist(mean, stddev);
    return dist(rng);
}


void ISACAntennaSystem::TxCallback(const ignition::msgs::StringMsg &_msg,
                                  const ignition::transport::MessageInfo &_info) {
    // Copy the message so we can modify it
    ignition::msgs::StringMsg msg = ignition::msgs::StringMsg(_msg);

    auto it = this->topicSensorMap[_info.Topic()];
    if (!it) return;
    ISACAntenna* sensorPtr = it;


    TxMessage entry;
    entry.sender = sensorPtr;
    entry.msg = _msg;

    std::lock_guard<std::mutex> lock(this->txBufferMutex);
    this->txMessageBuffer.push_back(std::move(entry));

    // Add metadata
    //msg.add_data(sensorPtr->modelName);

}


void ISACAntennaSystem::HandleNewEntities(ignition::gazebo::EntityComponentManager &_ecm) {

    _ecm.EachNew<ignition::gazebo::components::CustomSensor,
               ignition::gazebo::components::ParentEntity>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::CustomSensor *_custom,
        const ignition::gazebo::components::ParentEntity *_parent)->bool {
          
            sdf::Sensor data = _custom->Data();
            ignition::sensors::SensorFactory sensorFactory;
            auto sensor = sensorFactory.CreateSensor<custom::ISACAntenna>(data);
            if (!sensor)
            {
                ignerr << "Failed to create ISAC Antenna sensor" << std::endl;
                return false;
            }

            sensor->modelEntity = _ecm.Component<ignition::gazebo::components::ParentEntity>(_parent->Data())->Data();
            sensor->sensorEntity = _entity;

            // Set parent name
            auto parentName = _ecm.Component<ignition::gazebo::components::Name>(
                _parent->Data())->Data();
            sensor->SetParent(parentName);

            // Add sensor topic component to ECM
            _ecm.CreateComponent(_entity,
                ignition::gazebo::components::SensorTopic(sensor->Topic()));

            auto modelName = this->ExtractModelNameFromEntity(_entity, _ecm);

            // Advertise tx and rx publishers
            sensor->modelName = modelName;
            sensor->tx_topic_name = "/" + modelName + "/tx_data";
            sensor->rx_topic_name = "/" + modelName + "/rx_data";
            
            sensor->txPub = this->node->Advertise<ignition::msgs::StringMsg>(sensor->tx_topic_name);
            sensor->rxPub = this->node->Advertise<ignition::msgs::StringMsg>(sensor->rx_topic_name);

            // Store the sensor in entity map
            ISACAntenna* sensorPtr = sensor.get(); // raw pointer for lookup
            this->topicSensorMap[sensorPtr->tx_topic_name] = sensorPtr;
            // Store topic sensor mapping for the member callback
            this->entitySensorMap[_entity] = std::move(sensor);

            // Subscribe using member function callback
            this->node->Subscribe<ISACAntennaSystem, ignition::msgs::StringMsg>(
                sensorPtr->tx_topic_name,
                &ISACAntennaSystem::TxCallback,
                this
            );

            return true;
    });


    _ecm.EachNew<
        ignition::gazebo::components::Collision,
        ignition::gazebo::components::ParentEntity,
        ignition::gazebo::components::Geometry>(
      [&](const ignition::gazebo::Entity &collisionEntity,
          const ignition::gazebo::components::Collision *,
          const ignition::gazebo::components::ParentEntity *collisionParent,
          const ignition::gazebo::components::Geometry *geom)
    {


    ignition::gazebo::Entity linkEntity = collisionParent->Data();

    auto linkParent = _ecm.Component<ignition::gazebo::components::ParentEntity>(linkEntity);
    if (!linkParent)
        return true;

    ignition::gazebo::Entity modelEntity = linkParent->Data();

    ignition::math::AxisAlignedBox localAabb;
    const auto &shape = geom->Data();

    switch (shape.Type()) {
        case sdf::GeometryType::BOX:
        {
            auto box = shape.BoxShape();
            ignition::math::Vector3d half = box->Size() * 0.5;
            localAabb.Min() = -half;
            localAabb.Max() = half;
            break;
        }
        case sdf::GeometryType::SPHERE:
        {
            double r = shape.SphereShape()->Radius();
            localAabb.Min() = ignition::math::Vector3d(-r, -r, -r);
            localAabb.Max() = ignition::math::Vector3d( r,  r,  r);
            break;
        }
        case sdf::GeometryType::CYLINDER:
        {
            double r = shape.CylinderShape()->Radius();
            double h = shape.CylinderShape()->Length();
            localAabb.Min() = ignition::math::Vector3d(-r, -r, -h/2.0);
            localAabb.Max() = ignition::math::Vector3d( r,  r,  h/2.0);
            break;
        }
        case sdf::GeometryType::PLANE:
            return true;
        case sdf::GeometryType::MESH:
        {
            auto meshShape = shape.MeshShape();
            if (meshShape)
            {
                auto mesh = ignition::common::MeshManager::Instance()->Load(meshShape->Uri());
                if (mesh)
                {
                    ignition::math::Vector3d center, min, max;
                    mesh->AABB(center, min, max);
                    min *= meshShape->Scale();
                    max *= meshShape->Scale();
                    localAabb.Min() = min;
                    localAabb.Max() = max;
                }
            }
            break;
        }
        default:
            ignwarn << "Unsupported collision geometry, using 1x1x1 box" << std::endl;
            localAabb.Min() = ignition::math::Vector3d(-0.5, -0.5, -0.5);
            localAabb.Max() = ignition::math::Vector3d( 0.5,  0.5,  0.5);
            break;
    }

    Obstacle obs;
    obs.collisionEntity = collisionEntity;
    obs.modelEntity = modelEntity;
    obs.local_aabb = localAabb;
    obs.attenuation_dB_per_m = 5.0;

    this->obstacles.push_back(obs);

        return true;
    });


}

bool ISACAntennaSystem::RayIntersectsAABB(
    const ignition::math::Vector3d &rayOrigin,
    const ignition::math::Vector3d &rayDir,
    const ignition::math::Vector3d &aabb_min,
    const ignition::math::Vector3d &aabb_max,
    double &tEnter, double &tExit) {
    constexpr double EPS = 1e-8;

    tEnter = -std::numeric_limits<double>::infinity();
    tExit  =  std::numeric_limits<double>::infinity();

    const ignition::math::Vector3d min = aabb_min;
    const ignition::math::Vector3d max = aabb_max;

    for (int i = 0; i < 3; ++i)
    {
        if (std::abs(rayDir[i]) < EPS)
        {
            // Ray is parallel to slab
            if (rayOrigin[i] < min[i] || rayOrigin[i] > max[i])
                return false;
        }
        else
        {
            double invD = 1.0 / rayDir[i];
            double t1 = (min[i] - rayOrigin[i]) * invD;
            double t2 = (max[i] - rayOrigin[i]) * invD;

            if (t1 > t2)
                std::swap(t1, t2);

            tEnter = std::max(tEnter, t1);
            tExit  = std::min(tExit,  t2);

            if (tEnter > tExit)
                return false;
        }
    }

    // We only care about intersections in front of the ray
    if (tExit < 0)
        return false;

    // Clamp entry to zero if ray starts inside the box
    tEnter = std::max(tEnter, 0.0);

    return true;
}


void ISACAntennaSystem::ProcessTxBuffer(ignition::gazebo::EntityComponentManager &_ecm) {

    std::lock_guard<std::mutex> lock(this->txBufferMutex);
    
    for (auto &txEntry : this->txMessageBuffer) {

      auto src = txEntry.sender;
      ignition::gazebo::Entity txModelEntity = src->modelEntity;

      for (auto &[dst_topic, dst] : this->topicSensorMap)
      {
          if (dst == src)
              continue;

          ignition::gazebo::Entity rxModelEntity = dst->modelEntity;

          ignition::math::Vector3d txPos = src->position;
          ignition::math::Vector3d rxPos = dst->position;


          double obstacleTotalLoss_dB = 0;
          ignition::math::Vector3d rayDir = (rxPos - txPos).Normalized();
          double maxDist = (rxPos - txPos).Length();

          for (auto &obs : this->obstacles) {
            
              // Skip obstacles belonging to sender or receiver
              if (obs.modelEntity == txModelEntity || obs.modelEntity == rxModelEntity)
                  continue;

              double tEnter, tExit;
          
                auto worldPose = _ecm.Component<ignition::gazebo::components::Pose>(obs.modelEntity);
                auto localPose = _ecm.Component<ignition::gazebo::components::Pose>(obs.collisionEntity);
                
              
                if (!worldPose) {
                  continue;
                }

                ignition::math::Vector3d pos = worldPose->Data().Pos() + localPose->Data().Pos();
        
                auto aabb_min = obs.local_aabb.Min() + pos;
                auto aabb_max = obs.local_aabb.Max() + pos;



              if (RayIntersectsAABB(txPos, rayDir, aabb_min, aabb_max, tEnter, tExit)) {
                  
                  if (tEnter <= maxDist) {

                      double thickness =
                          std::min(tExit, maxDist) - std::max(tEnter, 0.0);

                      obstacleTotalLoss_dB +=
                          thickness * obs.attenuation_dB_per_m;
                  }
              }
          }


        ignition::msgs::StringMsg forwardedMsg = ignition::msgs::StringMsg();

        auto packetPayload = txEntry.msg.data();
        auto packetTelemetryData = this->computeReceivedSignal(src, dst, obstacleTotalLoss_dB);


        std::ostringstream combined;
        combined << "{"
                << "\"telemetry\": " << packetTelemetryData << ","
                << "\"payload\": " << std::quoted(packetPayload)
                << "}";


        forwardedMsg.set_data(combined.str());
          

        if (packetTelemetryData != "")
                dst->rxPub.Publish(forwardedMsg);

      }
      
    }

    this->txMessageBuffer.clear();
}






std::string ISACAntennaSystem::computeReceivedSignal(const ISACAntenna* src, const ISACAntenna* dst,
                           double obstacleTotalLoss_dB) {


    // Constants
    constexpr double kBoltzmann = 1.38e-23;  // J/K
    constexpr double T = 290.0;              // Temperature in Kelvin
    constexpr double c = 3e8;


    // Compute distance
    double distance = (dst->position - src->position).Length();

    // Compute path loss (log-distance model)
    double pathLoss_dB = 10.0 * dst->pathLossExponent * log10(distance);

    // Add fading
    double fading_dB = this->randGaussian(0.0, src->fadingStd);

    // Compute received power in dBm
    double rxPower_dBm = src->txPower_dBm - pathLoss_dB - obstacleTotalLoss_dB + fading_dB;

    // Compute thermal noise in dBm
    double noisePower_W = kBoltzmann * T * src->bandwidth_Hz;
    double noisePower_dBm = 10.0 * log10(noisePower_W) + 30.0 + dst->noiseFigure_dB;

    // Compute SNR
    double snr_dB = rxPower_dBm - noisePower_dBm;
    double snr_linear = pow(10.0, snr_dB / 10.0);

    // Compute achievable rate (Shannon capacity)
    double achievableRate_bps = src->bandwidth_Hz * log2(1.0 + snr_linear);

    double tof_s = distance/c;

    // Decide if packet is received
    // We use a simple threshold-based reception
    bool packetReceived = (snr_dB > 0.0);


    // We excluded cases where tof is 0, these are instances where the entities
    // are not yet initiated, and hence discarded
    if (packetReceived && tof_s > 0)
        return this->EncodeISACJSON(

            src->modelName,
            tof_s,

            src->txPower_dBm,
            rxPower_dBm,
            pathLoss_dB,
            fading_dB,

            noisePower_dBm,
            snr_dB,
            snr_linear,

            dst->bandwidth_Hz,
            achievableRate_bps,

            packetReceived
        );
    else
        return "";
}



std::string ISACAntennaSystem::ExtractModelNameFromEntity(
    const ignition::gazebo::Entity &_entity,
    ignition::gazebo::EntityComponentManager &_ecm) {
    
    auto sensorScopedName = ignition::gazebo::removeParentScope(
          ignition::gazebo::scopedName(_entity, _ecm, "::", false),
          "::"
        );

    // Remove leading '/'
    if (!sensorScopedName.empty() && sensorScopedName.front() == '/')
      sensorScopedName.erase(0, 1);

    // Extract model name (everything before first "::")
    std::string modelName = sensorScopedName;
    std::size_t pos = modelName.find("::");
    if (pos != std::string::npos)
      modelName = modelName.substr(0, pos);

    // Handle spaces → underscores (Simple Person → Simple_Person)
    std::replace(modelName.begin(), modelName.end(), ' ', '_');

    return modelName;

}



IGNITION_ADD_PLUGIN(ISACAntennaSystem, ignition::gazebo::System,
  ISACAntennaSystem::ISystemPreUpdate,
  ISACAntennaSystem::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(ISACAntennaSystem, "custom::ISACAntennaSystem")
