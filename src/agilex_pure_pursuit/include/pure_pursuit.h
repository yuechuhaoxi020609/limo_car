#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int16.h>
#include "path.h"

class PurePursuit {
public:
    PurePursuit();
    ~PurePursuit() {}

    void run();
private:

    double normalizeAngle(double angle);
    void makePath(const nav_msgs::Path& path_msg);
    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);

    bool getPose();
    void detectOobstacle(const sensor_msgs::LaserScan& scan);
    bool checkPosition(const WayPoint& p);
    bool checkHeading(const WayPoint& p);
    double calculateAngVel(const WayPoint& p);
    void calculateVel(const WayPoint& closest_p, const WayPoint& next_p,
                      double& lin_vel, double& ang_vel);
    void track();

    void publishCmdVel(double lin_vel, double ang_vel);
    void publishNextWayPoint(const WayPoint& p);
    bool loadTargetPath(const std::string& path_file, nav_msgs::Path& path_msg);

private:
    enum TrackingState {
        IDLE,
        START,
        TRACK,
    };

    ros::Publisher cmd_vel_pub_;
    ros::Publisher next_waypoint_pub_;
    ros::Publisher target_path_pub_;
    ros::Subscriber scan_sub_;
    tf::TransformListener tf_listener_;

    bool obstacle_detected_ = false;
    double obstacle_check_x_range_;
    double obstacle_check_y_range_;
    double position_control_tolerance_;
    double angle_control_tolerance_;
    double look_ahead_dist_;
    double position_kp_;
    double angle_kp_;
    double max_lin_vel_;
    double max_ang_vel_;

    Eigen::Vector3d cur_pose_ = Eigen::Vector3d::Zero();
    Path path_;
    TrackingState state_ = IDLE;
    double obstacle_distance_;
};

#endif // PURE_PURSUIT_H
