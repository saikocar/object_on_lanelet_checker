#include <rclcpp/rclcpp.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/geometry/Point.h>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

#include <unordered_set>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <lanelet2_extension/utility/message_conversion.hpp>

#include <iterator>  // std::begin, std::end

using std::placeholders::_1;
using lanelet::LaneletMapPtr;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;

class ObjectOnLaneletChecker : public rclcpp::Node, public std::enable_shared_from_this<ObjectOnLaneletChecker> {
public:
  ObjectOnLaneletChecker()
    : Node("object_on_lanelet_checker")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    map_sub_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
      "/map/vector_map",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&ObjectOnLaneletChecker::mapCallback, this, std::placeholders::_1));

    route_sub_ = this->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
      "/planning/mission_planning/route",
      rclcpp::QoS(1),
      [this](const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg) {
        this->routeCallback(msg);
      });

    object_sub_ = this->create_subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
      "/camera/rois_depth",
      rclcpp::QoS(10),
      [this](const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr msg) {
        this->objectCallback(msg);
      });

    //object_pub_ = create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>("/objects_on_route", 10);
    object_pub_ = create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>("/objects_on_route", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/objects_on_route_marker", 10);
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<DetectedObjectsWithFeature>::SharedPtr object_sub_;
  //rclcpp::Publisher<DetectedObjectsWithFeature>::SharedPtr object_pub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr object_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  lanelet::LaneletMapPtr lanelet_map_;
  std::unordered_set<lanelet::Id> route_lanelet_ids_;

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg) {
    lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_);
    RCLCPP_INFO(get_logger(), "Lanelet map loaded with %lu lanelets", lanelet_map_->laneletLayer.size());
  }

  void routeCallback(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg) {
    route_lanelet_ids_.clear();
    for (const auto& segment : msg->segments) {
      const auto& id = segment.preferred_primitive.id;
      route_lanelet_ids_.insert(id);
    }
    RCLCPP_INFO(get_logger(), "Route updated with %lu lanelets", route_lanelet_ids_.size());
  }

  void objectCallback(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr msg) {
    // if (!lanelet_map_ || route_lanelet_ids_.empty()) {
    //   RCLCPP_WARN(this->get_logger(), "!lanelet_map_ || route_lanelet_ids_.empty()");
    //   return;}

    if (!lanelet_map_ ) {
      //RCLCPP_WARN(this->get_logger(), "!lanelet_map_ ");
      return;}

    if (route_lanelet_ids_.empty()) {
      RCLCPP_WARN(this->get_logger(), "route_lanelet_ids_.empty()");
      return;}

    //DetectedObjectsWithFeature objects_on_route;
    autoware_auto_perception_msgs::msg::DetectedObjects objects_on_route;
    objects_on_route.header = msg->header;  // フレームは元のまま

    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;

    for (const auto& obj_with_feature : msg->feature_objects) {
      const auto & obj = obj_with_feature.object;

      // 元フレームのposeを取得（transformはするが判定用のみmap座標へ）
      geometry_msgs::msg::PoseStamped input_pose;
      input_pose.header = msg->header;
      input_pose.pose = obj.kinematics.pose_with_covariance.pose;

      geometry_msgs::msg::PoseStamped map_pose;
      try {
        map_pose = tf_buffer_->transform(input_pose, "map", tf2::durationFromSec(0.2));
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        continue;
      }

      lanelet::BasicPoint2d point_2d(map_pose.pose.position.x, map_pose.pose.position.y);

      bool on_route = false;
      for (const auto& id : route_lanelet_ids_) {
        auto it = lanelet_map_->laneletLayer.find(id);
        if (it != lanelet_map_->laneletLayer.end() &&
            lanelet::geometry::within(point_2d, it->polygon2d())) {
          on_route = true;
          break;
        }
      }

      if (!on_route){
        RCLCPP_INFO(this->get_logger(), "object_on_route nothing!");
        continue;}

      //objects_on_route.feature_objects.push_back(obj_with_feature);
      objects_on_route.objects.push_back(obj);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = msg->header.frame_id;  // 元のフレームIDをそのまま使う
      marker.header.stamp = msg->header.stamp;
      marker.ns = "objects_on_route";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = obj.kinematics.pose_with_covariance.pose;  // 元の座標系のposeを使う
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
      markers.markers.push_back(marker);
    }

    object_pub_->publish(objects_on_route);
    marker_pub_->publish(markers);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectOnLaneletChecker>());
  rclcpp::shutdown();
  return 0;
}

