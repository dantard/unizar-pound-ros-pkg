/** TOPIC / SERVICES DEFINITION
*
*  TOPIC(type, topic, source, dest, priority, period, time_to_live)
*  QOSTOPIC(type, topic, source, dest, priority, period, time_to_live, queue_length)
*  TFTOPIC(topic, source, dest, priority, period, time_to_live)
*
*  SERVICE(type, topic, source, priority, time_to_live)
*
*/

TOPIC(theora_image_transport::Packet, "/net/image/theora",                 0, "1", 51, 1,   500);
TOPIC(sensor_msgs::LaserScan,         "/scan/shaped",                      0, "1", 50, 200, 500);
TOPIC(geometry_msgs::PoseStamped,     "/mavros/local_position/pose",       0, "1", 50, 100, 500);
TOPIC(geometry_msgs::TwistStamped,    "/mavros/setpoint_velocity/cmd_vel", 0, "1", 50, 100, 500);
TOPIC(nav_msgs::Path,                 "/mavtest/local_plan",               0, "1", 50, 200, 500);
TOPIC(nav_msgs::Odometry,             "/rtabmap/odom",                     0, "1", 50, 200, 500);
TOPIC(mavros_msgs::State,             "/mavros/state",                     0, "1", 55, 500, 500);
TOPIC(mavros_msgs::BatteryStatus,     "/mavros/battery",                   0, "1", 56, 100, 500);
TOPIC(std_msgs::Int8,                 "/int8",                             0, "1", 50, 100, 500);
