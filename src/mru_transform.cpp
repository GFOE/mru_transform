/** 
 * @file mru_transform.cpp 
 * 
 * Uses the concept of a "Sensor" which is the collection of three inputs:
 * position, orientation and velocity.  Multiple sensors can be defined,
 * but it seems like the most common use-case is for a single "sensor".
 * 
 * Private Parameters:
 *  - map_frame: 
 *  - base_frame: 
 *  - odom_frame:
 *  - odom_topic:
 *  - sensors: Used to specify the sensor object type - not sure if/how this is used.
 * 
 * Subscribes:
 *  - 
 *  -
 * 
 * Publishes:
 *  - nav_msgs::Odometry on 'odom'
 *  - sensor_msgs::NavSatFix on 'nav/position'
 *  - sensor_msgs::Imu on 'nav/orientation'
 *  - geometry_msgs::TwistWithCovarianceStamped on 'nav/velocity'
 *  - std_msgs::String on 'nav/active_sensor'
 *
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include "project11/utils.h"

#include "mru_transform/LatLongToEarth.h"
#include "mru_transform/LatLongToMap.h"
#include "mru_transform/EarthToLatLong.h"
#include "mru_transform/MapToLatLong.h"

namespace p11 = project11;

/**
 * @brief Class that creates and broadcasts two transforms.  
 * 
 * Class gets created the first time update() is called. Reference position is initial ben pos fix
 * 
 * 1. earth to ben/map transform
 *   * earth-centered to datum transform. This is my best guess based on name conventions
 *   * datum given when creating class ben's initial position
 *   * transform published on 2sec timer (Is this a fixed timer?  Should parameterize, and this seems awful slow.)
 * 2.  ben/map to ben/odom transform
 *   * transform is all zeros (trans + rot) except rotation.w = 1
 *   *transform published on 2sec timer
 * 
 * Services:
 * This class also advertises 2 services:
 * 1. wgs84_to_map: transforms lat/lon/alt into x,y,z from reference datum
 * 2. map_to_wds84: transforms x,y,z into lat/lon/alt
 */
class MapFrame
{
public:
  MapFrame(p11::LatLongDegrees const &datum, std::string const &map_frame, std::string const &odom_frame, std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster, ros::Duration broadcast_interval):m_mapFrame(p11::ENUFrame(datum)), m_broadcaster(broadcaster), m_broadcast_interval(broadcast_interval)
  {
    m_earth_to_map_transform.header.frame_id = "earth";
    m_earth_to_map_transform.child_frame_id = map_frame;
    
    gz4d::GeoPointECEF originECEF(datum);
    m_earth_to_map_transform.transform.translation.x = originECEF[0];
    m_earth_to_map_transform.transform.translation.y = originECEF[1];
    m_earth_to_map_transform.transform.translation.z = originECEF[2];
    
    tf2::Quaternion longQuat;
    longQuat.setRPY(0.0,0.0,(datum.longitude()+90.0)*M_PI/180.0);
    tf2::Quaternion latQuat;
    latQuat.setRPY((90-datum.latitude())*M_PI/180.0,0.0,0.0);
    tf2::Quaternion earth_to_map_rotation = longQuat*latQuat;
    
    m_earth_to_map_transform.transform.rotation = tf2::toMsg(earth_to_map_rotation);
    
    broadcaster->sendTransform(m_earth_to_map_transform);
    
    m_map_to_odom_transform.header.frame_id = map_frame;
    m_map_to_odom_transform.child_frame_id = odom_frame;
    m_map_to_odom_transform.transform.rotation.w = 1.0; // make null quaternion unit length
    
    broadcaster->sendTransform(m_map_to_odom_transform);
    
    ros::NodeHandle nh;
    m_wgs84_to_map_service = nh.advertiseService("wgs84_to_map", &MapFrame::ll2map, this);
    m_map_to_wgs84_service = nh.advertiseService("map_to_wgs84", &MapFrame::map2ll, this);
    
    m_timer = nh.createTimer(broadcast_interval, &MapFrame::timerCallback, this);
  }

  p11::Point toLocal(p11::LatLongDegrees const &p) const
  {
    return m_mapFrame.toLocal(p);
  }
  
private:
  p11::ENUFrame m_mapFrame;
  
  geometry_msgs::TransformStamped m_earth_to_map_transform;
  geometry_msgs::TransformStamped m_map_to_odom_transform;
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_broadcaster;
  ros::Duration m_broadcast_interval;
  ros::Timer m_timer;

  ros::ServiceServer m_wgs84_to_map_service;
  ros::ServiceServer m_map_to_wgs84_service;

  void timerCallback(const ros::TimerEvent &timerEvent)
  {
    // set a bit to the future to prevent blocking transforms
    // remember to take this into account if updating datum
    ros::Time a_bit_later = timerEvent.current_real+m_broadcast_interval;
    
    // make sure we don't go back in time when sending transform
    if(a_bit_later > m_earth_to_map_transform.header.stamp)
    {
      m_earth_to_map_transform.header.stamp = a_bit_later;
      m_broadcaster->sendTransform(m_earth_to_map_transform);
      m_map_to_odom_transform.header.stamp = a_bit_later;
      m_broadcaster->sendTransform(m_map_to_odom_transform);
    }
  }

  bool ll2map(mru_transform::LatLongToMap::Request &req, mru_transform::LatLongToMap::Response &res)
  {
      p11::LatLongDegrees p_ll;
      p11::fromMsg(req.wgs84.position, p_ll); 
      p11::ECEF p_ecef(p_ll);
      
      p11::Point position = m_mapFrame.toLocal(p_ecef);
      
      res.map.header.frame_id = m_earth_to_map_transform.child_frame_id;
      res.map.header.stamp = req.wgs84.header.stamp;
      p11::toMsg(position, res.map.point);
      return true;
  }

  bool map2ll(mru_transform::MapToLatLong::Request &req, mru_transform::MapToLatLong::Response &res)
  {
      p11::Point position;
      p11::fromMsg(req.map.point, position);
      p11::LatLongDegrees latlon = m_mapFrame.toLatLong(position);
      p11::toMsg(latlon, res.wgs84.position);
      
      res.wgs84.header.frame_id = "wgs84";
      res.wgs84.header.stamp = req.map.header.stamp;
      return true;
  }

};

// Forward declare so Sensor can call it.
// TODO: Should really use a header file.
/** 
 * @brief Who knows?
 * 
 * publishes /ben/nav/active_sensor (show sensor info hasn't timed out)

creates MapFrame if it doesn't exist

datum given is initial position reported by Time Synchronizer (ben's initial position)
publishes ben/map to ben/base_link_north_up transform

transform only contains translation from ben/map origin (ben's initial position) to current position
rotation is all zeros except rotation.w = 1
publishes ben/base_link_north_up to ben/base_link_level transform

simple rotation transform
tf_echo shows this is the same as ben/base_link_north_up to ben/base_link transform
publishes ben/base_link_north_up to ben/base_link transform

simple rotation transform
tf_echo shows this is the same as ben/base_link_north_up to ben/base_link_level transform
publishes /ben/nav/position
publishes /ben/nav/orientation
publishes /ben/nav/velocity

publishes odometry message (/ben/odom)

pose.position in map frame (x,y,z)
pose.orientation same as /ben/nav/orientation.orientation
twist.twist.angular same as /ben/nav/orientation.angular_velocity
twist.twist.linear is a transform
transform is a rotation equal to inverse of ben's orientation applied to the /ben/nav/velocity.twisit.twist.linear (line #326 mir_transform.cpp)
*/


void update();

/**
 * @brief Each instance subscribes to position, velocity and orientation.

Uses Time Synchronizer (￼message_filters - ROS Wiki) to synchronize incoming position, orientation and velocity messages based on timestamp.
 *
 * position, orientation and velocity messages/topics can be configure through launch file (see: project11/test_posmv.launch)
 *
 * time Synchronizer callback calls update()
 * 
 * gets created early in the main(). It only requires names of sensor topics that can be set in launch file. If none are provided it defaults to topics: postion, orientation and velocity
 */
class Sensor
{
public:
  /** @brief Constructor used of the "sensors" private parameter is provided **/
  Sensor(XmlRpc::XmlRpcValue const &sensor_param)
  {
    std::string position_topic, orientation_topic, velocity_topic, name;
    position_topic = std::string(sensor_param["topics"]["position"]);
    orientation_topic = std::string(sensor_param["topics"]["orientation"]);
    velocity_topic = std::string(sensor_param["topics"]["velocity"]);
    name = std::string(sensor_param["name"]);
    Initialize(position_topic, orientation_topic, velocity_topic, name);
  }

  /** @brief Default constructor using message topics of 'position', 'orientation' and 'velocity. **/
  Sensor()
  {
    Initialize("position", "orientation", "velocity");
  }

  /**
   * @brief Setup function called by constructors
   * 
   * @param position_topic ROS topic for position messages
   * @param orientation_topic ROS topic for orientation messages
   * @param velocity_topic ROS topic for velocity messages
   * @param name Name of this sensor object.
   */
  void Initialize(const std::string &position_topic, const std::string &orientation_topic , const std::string &velocity_topic, std::string name="default")
  {
    ros::NodeHandle nh;

    // These subscriptions use message_filters::Subscriber filters
    // which are then typedef'ed for the particular message type.
    // The typedef seems unnecessary and confusing.
    // Using the message_filters::Subscriber wraps the ROS subscriber
    // and allows the callback output to be used as input to the Synchronizer
    m_position_sub = std::shared_ptr<PositionSub>(new PositionSub(nh, position_topic, 1));
    m_orientation_sub =  std::shared_ptr<OrientationSub>(new OrientationSub(nh, orientation_topic, 1));
    m_velocity_sub = std::shared_ptr<VelocitySub>(new VelocitySub(nh, velocity_topic, 1));

    m_name = name;

    // Instantiates a SyncType object
    // which is a globally typedef'ed message_filters::TimeSynchronizer.
    // This structure makes it unnecessarily unclear.
    m_sync = std::shared_ptr<SyncType>(new SyncType(*m_position_sub, *m_orientation_sub, *m_velocity_sub, 10));
    // Register the synchonizer's callback so that when we get three
    m_sync->registerCallback(boost::bind(&Sensor::callback, this, _1, _2, _3));
    // Register the synchonizer's drop callback so we can warn the user
    m_sync->registerDropCallback(boost::bind(&Sensor::dropCallback, this, _1, _2, _3));
  }

  /** @brief Get the name string of Sensor object.
   * 
   * @returns m_name Name string
   */
  const std::string &getName() const {return m_name;}

  sensor_msgs::NavSatFix::ConstPtr lastPositionMessage() const {return m_last_position;}
  sensor_msgs::Imu::ConstPtr lastOrientationMessage() const {return m_last_orientation;}
  geometry_msgs::TwistWithCovarianceStamped::ConstPtr lastVelocityMessage() const {return m_last_velocity;}

  typedef std::shared_ptr<Sensor> Ptr;
  
private:
  /** 
   * @brief When three messages with the same timestamp, store them and call update() 
   * 
   */
  void callback(const sensor_msgs::NavSatFix::ConstPtr & position, const sensor_msgs::Imu::ConstPtr & orientation, const geometry_msgs::TwistWithCovarianceStamped::ConstPtr & velocity)
  {
    m_last_position = position;
    m_last_orientation = orientation;
    m_last_velocity = velocity;
    update();
  }

  /** @brief Called if we get three messages that are not synced - logs warning */
  void dropCallback(const sensor_msgs::NavSatFix::ConstPtr & position, const sensor_msgs::Imu::ConstPtr & orientation, const geometry_msgs::TwistWithCovarianceStamped::ConstPtr & velocity)
  {
    ROS_WARN("mru_transform: dropped synchronized position, orientation and velocity messages - check your timestamps!");
      
  }
  
  typedef message_filters::Subscriber<sensor_msgs::NavSatFix> PositionSub;
  std::shared_ptr<PositionSub> m_position_sub;

  typedef message_filters::Subscriber<sensor_msgs::Imu> OrientationSub;
  std::shared_ptr<OrientationSub> m_orientation_sub;

  typedef message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> VelocitySub;
  std::shared_ptr<VelocitySub> m_velocity_sub;
  
  typedef message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, sensor_msgs::Imu, geometry_msgs::TwistWithCovarianceStamped> SyncType;
  std::shared_ptr<SyncType> m_sync;
  
  sensor_msgs::NavSatFix::ConstPtr m_last_position;
  sensor_msgs::Imu::ConstPtr m_last_orientation;
  geometry_msgs::TwistWithCovarianceStamped::ConstPtr m_last_velocity;
  std::string m_name;
};


// TODO: This declaration of a bunch of global variables is bad form
// and makes the code difficult to maintain.  Should include a header and document these variables as attributes of a properly scoped object.

/** @brief List of sensors, in order of priority */
std::vector<Sensor::Ptr> sensors;

sensor_msgs::NavSatFix::ConstPtr last_sent_position;
sensor_msgs::Imu::ConstPtr last_sent_orientation;
geometry_msgs::TwistWithCovarianceStamped::ConstPtr last_sent_velocity;

// replublish selected nav messages
ros::Publisher position_pub;
ros::Publisher orientation_pub;
ros::Publisher velocity_pub;
ros::Publisher active_sensor_pub;

ros::Duration sensor_timeout(1.0);

std::string base_frame = "base_link";
std::string map_frame = "map";
std::string odom_frame = "odom";
std::string odom_topic = "odom";

std::shared_ptr<MapFrame> mapFrame;
std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
ros::Publisher odom_pub;

/** 
 * @brief Called by Sensor-syncronizer-callback when three messages with the same timestamp are received.
 *
 * 
 */
void update()
{
  ros::Time now = ros::Time::now();
  
  sensor_msgs::NavSatFix::ConstPtr position;
  sensor_msgs::Imu::ConstPtr orientation;
  geometry_msgs::TwistWithCovarianceStamped::ConstPtr velocity;

  // loop through the sensors until we get an unexpired message of each type
  // TODO: Use of s->lastPositionMessage() condition
  // without explicit initialization of
  // the underlying private variable, or the function output, is prone to
  // portability problems.
  bool good_sensor = false;
  for(auto s: sensors)
  {
    if(!position && s->lastPositionMessage() &&
       now - s->lastPositionMessage()->header.stamp < sensor_timeout &&
       s->lastPositionMessage()->status.status >= 0)
    {
      position = s->lastPositionMessage();
    }
    else if (!s->lastPositionMessage())
    {
      ROS_WARN_STREAM("mru_transform: Sensor=<" <<
		      s->getName() <<
		      "> Cannot use the last position input. "
		      "Last position message does not exist.");
    }
    else if (s->lastPositionMessage() &&
	     now - s->lastPositionMessage()->header.stamp >= sensor_timeout)
    {
      auto dur = now - s->lastPositionMessage()->header.stamp;
      double dt = dur.toSec();
      ROS_WARN_STREAM("mru_transform: Sensor=<" <<
		      s->getName() <<
		      "> Cannot use the last position input. "
		      "Check the staleness or status of the position input. "
		      "Staleness = " << dt << " s.");
    }
    else if (s->lastPositionMessage()->status.status < 0)
    {
      ROS_WARN_STREAM("mru_transform: Sensor=<" <<
		      s->getName() <<
		      "> Cannot use the last position input. "
		      "Message has bad status.");
    }
    else
    {
      ROS_WARN_STREAM("mru_transform:  Sensor=<" <<
		      s->getName() <<
		      "> Cannot use the last position input. "
		      "Not sure why?");
    }
    if(!orientation && s->lastOrientationMessage() &&
       now - s->lastOrientationMessage()->header.stamp < sensor_timeout)
    {
      orientation = s->lastOrientationMessage();
    }
    else
    {
      ROS_WARN("mru_transform: Cannot use last orientation input. "
	       "Check staleness of the input");
    }
    if(!velocity && s->lastVelocityMessage() && now - s->lastVelocityMessage()->header.stamp < sensor_timeout)
    {
      velocity = s->lastVelocityMessage();
    }
    else
    {
      ROS_WARN("mru_transform: Cannot use last velocity input. "
	       "Check staleness of the input");
    }
    if(position && orientation && velocity)
    {
      std_msgs::String active;
      active.data = s->getName();
      active_sensor_pub.publish(active);
      good_sensor = true;
      break;
    }
    else
    {
      ROS_WARN_STREAM("mru_transform: Sensor named <" << s->getName() <<
		      "> will not be used and mru_transform will not "
		      "publish output if there is not another good Sensor");
		    
    }
  }

  if (!good_sensor){
    ROS_ERROR("mru_transform: There are no good Sensors. Will not publish nav outputs or tf transforms!");
  }

  nav_msgs::Odometry odom;
  odom.header.frame_id = odom_frame;
  
  if(position && (!last_sent_position || position->header.stamp > last_sent_position->header.stamp))
  {
    p11::LatLongDegrees p;
    p11::fromMsg(*position, p);
    if (std::isnan(p[2]))
      p[2] = 0.0;

    if(!mapFrame && position->status.status >= 0)
    {
      mapFrame = std::shared_ptr<MapFrame>(new MapFrame(p, map_frame, odom_frame, broadcaster, ros::Duration(2.0) ));
    }
    
    position_pub.publish(position);
    
    p11::Point position_map = mapFrame->toLocal(p);
    
    geometry_msgs::TransformStamped map_to_north_up_base_link;
    map_to_north_up_base_link.header.stamp = position->header.stamp;
    map_to_north_up_base_link.header.frame_id = map_frame;
    map_to_north_up_base_link.child_frame_id = base_frame+"_north_up";
    p11::toMsg(position_map, map_to_north_up_base_link.transform.translation);
    map_to_north_up_base_link.transform.rotation.w = 1.0;
    broadcaster->sendTransform(map_to_north_up_base_link);
    
    p11::toMsg(position_map, odom.pose.pose.position);
    odom.header.stamp = position->header.stamp;
    
    last_sent_position = position;
  }

  tf2::Quaternion orientation_quat;
  
  if(orientation && (!last_sent_orientation || orientation->header.stamp > last_sent_orientation->header.stamp))
  {
    orientation_pub.publish(orientation);

    tf2::fromMsg(orientation->orientation, orientation_quat);
    
    double roll,pitch,yaw;
    tf2::getEulerYPR(orientation_quat, yaw, pitch, roll);
       
    geometry_msgs::TransformStamped north_up_base_link_to_level_base_link;
    north_up_base_link_to_level_base_link.header.stamp = orientation->header.stamp;
    north_up_base_link_to_level_base_link.header.frame_id = base_frame+"_north_up";
    north_up_base_link_to_level_base_link.child_frame_id = base_frame+"_level";
    tf2::Quaternion heading_quat;
    heading_quat.setRPY(0.0,0.0,yaw);
    north_up_base_link_to_level_base_link.transform.rotation = tf2::toMsg(heading_quat);
    broadcaster->sendTransform(north_up_base_link_to_level_base_link);
    
    geometry_msgs::TransformStamped north_up_base_link_to_base_link;
    north_up_base_link_to_base_link.header.stamp = orientation->header.stamp;
    north_up_base_link_to_base_link.header.frame_id = base_frame+"_north_up";
    north_up_base_link_to_base_link.child_frame_id = base_frame;
    north_up_base_link_to_base_link.transform.rotation = orientation->orientation;
    broadcaster->sendTransform(north_up_base_link_to_base_link);

    odom.pose.pose.orientation = orientation->orientation;
    odom.twist.twist.angular = orientation->angular_velocity;
    
    last_sent_orientation = orientation;
  }
  
  if(velocity && (!last_sent_velocity || velocity->header.stamp > last_sent_velocity->header.stamp))
  {
    velocity_pub.publish(velocity);
    odom.child_frame_id = velocity->header.frame_id;
    geometry_msgs::TransformStamped odom_base_rotation;
    odom_base_rotation.transform.rotation = tf2::toMsg(orientation_quat.inverse());
    tf2::doTransform(velocity->twist.twist.linear, odom.twist.twist.linear, odom_base_rotation);
    odom_pub.publish(odom);
    last_sent_velocity = velocity;
  }
}

/** 
 * @brief Get params, set up pubs and then just spin.
 * 
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mru_transform");
    
  ros::NodeHandle nh_private("~");

  // TODO: Verify that the calls were successful and define the behavior if not.
  nh_private.getParam("map_frame", map_frame);
  nh_private.getParam("base_frame", base_frame);
  nh_private.getParam("odom_frame", odom_frame);
  // TODO: Don't use params to define topic names - use remap.
  nh_private.getParam("odom_topic", odom_topic);

  broadcaster = std::shared_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster);

  ros::NodeHandle nh;
  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 50);

  XmlRpc::XmlRpcValue sensors_param;
  if(nh_private.getParam("sensors", sensors_param))
  {
    if(sensors_param.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for(int i = 0; i < sensors_param.size(); i++)
        sensors.push_back(std::shared_ptr<Sensor>(new Sensor(sensors_param[i])));
    }
  }
  else
  {
    ROS_WARN("mru_transform: There is no local ROS parameter 'sensors', "
	     "so using the defaults.");
    sensors.push_back(std::shared_ptr<Sensor>(new Sensor()));
  }
      
  // publishers for the selected messages. This should allow subscribers to get the best available from a single set of topics
  position_pub = nh.advertise<sensor_msgs::NavSatFix>("nav/position",10);
  orientation_pub = nh.advertise<sensor_msgs::Imu>("nav/orientation",10);
  velocity_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("nav/velocity",10);
  active_sensor_pub = nh.advertise<std_msgs::String>("nav/active_sensor",10);
  
  ros::spin();
  return 0;
}
