#ifndef ISACANTENNASYSTEM_HH_
#define ISACANTENNASYSTEM_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/sensors/Sensor.hh>
#include <ignition/transport/Node.hh>
#include <ignition/math/Vector3.hh>

#include <unordered_map>
#include <memory>
#include <string>
#include <mutex>

#include "ISACAntenna.hh"

namespace custom {


struct TxMessage {
    custom::ISACAntenna*        sender;
    ignition::msgs::StringMsg   msg;
};

struct Obstacle {
  ignition::gazebo::Entity          collisionEntity;
  ignition::gazebo::Entity          modelEntity;
  ignition::math::AxisAlignedBox    local_aabb;
  double                            attenuation_dB_per_m{5.0};
};


class ISACAntennaSystem:
public ignition::gazebo::System,
public ignition::gazebo::ISystemPreUpdate,
public ignition::gazebo::ISystemPostUpdate {

  public:

    ISACAntennaSystem();

    std::unordered_map<std::string, ISACAntenna *> topicSensorMap;
    std::unordered_map <
      ignition::gazebo::Entity,
      std::shared_ptr<ISACAntenna>
    >                                             entitySensorMap;

    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                    ignition::gazebo::EntityComponentManager &_ecm) final;
    void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                    const ignition::gazebo::EntityComponentManager &_ecm) final;
    // Static utility function used to encode ISAC sensing data to a unique JSON message
    static std::string EncodeISACJSON(
        const std::string &txId,
        double tof_s,

        double txPower_dBm,
        double rxPower_dBm,
        double pathLoss_dB,
        double fading_dB,

        double noisePower_dBm,
        double snr_dB,
        double snr_linear,

        double bandwidth_Hz,
        double achievableRate_bps,

        bool packetReceived ) {
        std::ostringstream ss;
        ss << "{"
          << "\"tx_id\":\"" << txId << "\","
          << "\"tof_s\":" << tof_s << ","

          << "\"tx_power_dBm\":" << txPower_dBm << ","
          << "\"rx_power_dBm\":" << rxPower_dBm << ","
          << "\"path_loss_dB\":" << pathLoss_dB << ","
          << "\"fading_dB\":" << fading_dB << ","

          << "\"noise_power_dBm\":" << noisePower_dBm << ","
          << "\"snr_dB\":" << snr_dB << ","
          << "\"snr_linear\":" << snr_linear << ","

          << "\"bandwidth_Hz\":" << bandwidth_Hz << ","
          << "\"achievable_rate_bps\":" << achievableRate_bps << ","

          << "\"packet_received\":" << (packetReceived ? "true" : "false")
          << "}";

        return ss.str();
    }


  private:

    std::unique_ptr<ignition::transport::Node>  node;
    std::vector<Obstacle>                       obstacles;
    std::vector<TxMessage>                      txMessageBuffer;
    std::mutex                                  txBufferMutex;
    std::unordered_map<
        ignition::gazebo::Entity,
        std::string
    >                                           modelEntities;
  
    void        RemoveSensorEntities(const ignition::gazebo::EntityComponentManager &_ecm);
    void        OnTx(ignition::gazebo::Entity sender, const ignition::msgs::StringMsg &msg);
    void        TxCallback(
                    const ignition::msgs::StringMsg &_msg,
                    const ignition::transport::MessageInfo &_info
    );
    void        HandleNewEntities(ignition::gazebo::EntityComponentManager &_ecm);
    void        ProcessTxBuffer(ignition::gazebo::EntityComponentManager &_ecm);
    std::string ExtractModelNameFromEntity(
                    const ignition::gazebo::Entity &_entity,
                    ignition::gazebo::EntityComponentManager &_ecm
    );
    bool        RayIntersectsAABB(
                    const ignition::math::Vector3d &rayOrigin,
                    const ignition::math::Vector3d &rayDir,
                    const ignition::math::Vector3d &aabb_min,
                    const ignition::math::Vector3d &aabb_max,
                    double &tEnter,
                    double &tExit
    );
    double      randGaussian(double mean, double stddev);
    std::string computeReceivedSignal(
                    const ISACAntenna* src,
                    const ISACAntenna* dst,
                    double obstacleTotalLoss_dB
    );


    std::mt19937 rng;

};

}

#endif // ISACANTENNASYSTEM_HH_
