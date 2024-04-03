#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
// #include "laser_geometry/laser_geometry.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"


using namespace std::placeholders;
using namespace std::chrono_literals;
using namespace sensor_msgs::msg;
using namespace message_filters;

class SensorSyncNode : public rclcpp::Node
{
public:
  SensorSyncNode() : Node("sensor_data_synchronizer_node")
  {
    // QoS Config
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 1; // queue_size=1, always keep latest message
    custom_qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT; // use "best effort strategy"
    custom_qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos_profile.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL; // make sure to receive the message that before Subscriber start

    // subscription
    Subscriber<LaserScan> laser_sub(this, "/scan", custom_qos_profile);
    Subscriber<Imu> imu_sub(this, "/bno055/imu", custom_qos_profile);

    // TimeSynchronizer
    typedef sync_policies::ApproximateTime<LaserScan,Imu> approx_policy;
    Synchronizer<approx_policy>syncApprox(approx_policy(10),laser_sub, imu_sub);
    syncApprox.registerCallback(std::bind(&SensorSyncNode::sync_callback, this, _1, _2));
    // syncApprox.registerCallback(sync_callback);
  }

    void sync_callback(const LaserScan::ConstSharedPtr laser_msg, const Imu::ConstSharedPtr imu_msg)
    // void sync_callback(const LaserScan::SharedPtr laser_msg, const Imu::SharedPtr imu_msg)
  {
    (void) laser_msg;
    (void) imu_msg;
    RCLCPP_INFO(this->get_logger(), "Received synchronized messages");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(rclcpp::Node::make_shared("sensor_data_synchronizer_node"));
  rclcpp::shutdown();
  return 0;
}
