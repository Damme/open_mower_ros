#include <ftc_local_planner/ftc_planner.h>

#include <cmath>

#include <pluginlib/class_list_macros.h>
#include "mbf_msgs/ExePathAction.h"

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::FTCPlanner, mbf_costmap_core::CostmapController)

#define RET_SUCCESS 0
#define RET_COLLISION 104
#define RET_BLOCKED 109

// ---- stuck recovery (uglyfix until the ROS2 migration) ---------------------
// The mower firmware latches wheel speed to 0 when bumped with both wheels
// driving forward; commanding exact zero (or reversing) releases the latch.
// FOLLOWING has no timeout, so a latched bump used to stall forever.
static const double kStuckTimeout = 3.0;   // s commanding forward without moving
static const double kStuckMinMove = 0.05;  // m of movement that counts as progress
static const double kReleaseTime  = 1.0;   // s of exact-zero cmd (releases the latch)
static const double kReverseSpeed = 0.15;  // m/s straight back, no steering
static const double kReverseTime  = 3.0;   // s -> ~0.3 m over just-driven ground
static const double kSkipAhead    = 0.75;   // m of path skipped, times the attempt no.
static const int    kMaxAttempts  = 4;     // then give up: is_crashed -> MBF aborts

namespace ftc_local_planner
{

    FTCPlanner::FTCPlanner()
    {
    }

    void FTCPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);

        progress_server = private_nh.advertiseService(
            "planner_get_progress", &FTCPlanner::getProgress, this);

        global_point_pub = private_nh.advertise<geometry_msgs::PoseStamped>("global_point", 1);
        global_plan_pub = private_nh.advertise<nav_msgs::Path>("global_plan", 1, true);
        obstacle_marker_pub = private_nh.advertise<visualization_msgs::Marker>("costmap_marker", 10);

        costmap = costmap_ros;
        costmap_map_ = costmap->getCostmap();
        tf_buffer = tf;

        // Parameter for dynamic reconfigure
        reconfig_server = new dynamic_reconfigure::Server<FTCPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<FTCPlannerConfig>::CallbackType cb = boost::bind(&FTCPlanner::reconfigureCB, this,
                                                                                     _1, _2);
        reconfig_server->setCallback(cb);

        current_state = PRE_ROTATE;

        // PID Debugging topic
        if (config.debug_pid)
        {
            pubPid = private_nh.advertise<ftc_local_planner::PID>("debug_pid", 1, true);
        }

        // Recovery behavior initialization
        failure_detector_.setBufferLength(std::round(config.oscillation_recovery_min_duration * 10));

        ROS_INFO("FTCLocalPlannerROS: Version 2 Init.");
    }

    void FTCPlanner::reconfigureCB(FTCPlannerConfig &c, uint32_t level)
    {
        if (c.restore_defaults)
        {
            reconfig_server->getConfigDefault(c);
            c.restore_defaults = false;
        }
        config = c;

        // just to be sure
        current_movement_speed = config.speed_slow;

        // set recovery behavior
        failure_detector_.setBufferLength(std::round(config.oscillation_recovery_min_duration * 10));
    }

    bool FTCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        current_state = PRE_ROTATE;
        state_entered_time = ros::Time::now();
        is_crashed = false;

        global_plan = plan;
        current_index = 0;
        current_progress = 0.0;

        last_time = ros::Time::now();
        current_movement_speed = config.speed_slow;

        lat_error = 0.0;
        lon_error = 0.0;
        angle_error = 0.0;
        i_lon_error = 0.0;
        i_lat_error = 0.0;
        i_angle_error = 0.0;
        last_cmd_vel_linear = 0.0;
        carrot_gated = false;
        recovery_phase = 0;
        recovery_attempts = 0;
        stuck_ref_valid = false;

        nav_msgs::Path path;

        if (global_plan.size() > 2)
        {
            // duplicate last point
            global_plan.push_back(global_plan.back());
            // give second from last point last oriantation as the point before that
            global_plan[global_plan.size() - 2].pose.orientation = global_plan[global_plan.size() - 3].pose.orientation;
            path.header = plan.front().header;
            path.poses = plan;
        }
        else
        {
            ROS_WARN_STREAM("FTCLocalPlannerROS: Global plan was too short. Need a minimum of 3 poses - Cancelling.");
            current_state = FINISHED;
            state_entered_time = ros::Time::now();
        }
        global_plan_pub.publish(path);

        ROS_INFO_STREAM("FTCLocalPlannerROS: Got new global plan with " << plan.size() << " points.");

        return true;
    }

    FTCPlanner::~FTCPlanner()
    {
        if (reconfig_server != nullptr)
        {
            delete reconfig_server;
            reconfig_server = nullptr;
        }
    }

    double FTCPlanner::distanceLookahead()
    {
        if (global_plan.size() < 2)
        {
            return 0;
        }
        Eigen::Quaternion<double> current_rot(current_control_point.linear());

        Eigen::Affine3d last_straight_point = current_control_point;
        for (uint32_t i = current_index + 1; i < global_plan.size(); i++)
        {
            tf2::fromMsg(global_plan[i].pose, last_straight_point);
            // check, if direction is the same. if so, we add the distance
            Eigen::Quaternion<double> rot2(last_straight_point.linear());
            if (abs(rot2.angularDistance(current_rot)) > config.speed_fast_threshold_angle * (M_PI / 180.0))
            {
                break;
            }
        }

        return (last_straight_point.translation() - current_control_point.translation()).norm();
    }

    uint32_t FTCPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
                                                 const geometry_msgs::TwistStamped &velocity,
                                                 geometry_msgs::TwistStamped &cmd_vel, std::string &message)
    {

        ros::Time now = ros::Time::now();
        double dt = now.toSec() - last_time.toSec();
        last_time = now;

        if (is_crashed)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return RET_COLLISION;
        }

        if (current_state == FINISHED)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return RET_SUCCESS;
        }

        // Stuck recovery (uglyfix): while active it owns this cycle's command
        // entirely -- carrot frozen, PID untouched, collision check skipped.
        if (recoveryActive(pose, cmd_vel))
        {
            return RET_SUCCESS;
        }

        // We're not crashed and not finished.
        // First, we update the control point if needed. This is needed since we need the local_control_point to calculate the next state.
        update_control_point(dt);
        // Then, update the planner state.
        auto new_planner_state = update_planner_state();
        if (new_planner_state != current_state)
        {
            ROS_INFO_STREAM("FTCLocalPlannerROS: Switching to state " << new_planner_state);
            state_entered_time = ros::Time::now();
            current_state = new_planner_state;
        }

        if (checkCollision(config.obstacle_lookahead))
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            is_crashed = true;
            return RET_BLOCKED;
        }

        // Finally, we calculate the velocity commands.
        calculate_velocity_commands(dt, cmd_vel);

        if (is_crashed)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return RET_COLLISION;
        }

        return RET_SUCCESS;
    }


    bool FTCPlanner::isGoalReached(double dist_tolerance, double angle_tolerance)
    {
        return current_state == FINISHED && !is_crashed;
    }

    bool FTCPlanner::cancel()
    {
        ROS_WARN_STREAM("FTCLocalPlannerROS: FTC planner was cancelled.");
        current_state = FINISHED;
        state_entered_time = ros::Time::now();
        return true;
    }

    FTCPlanner::PlannerState FTCPlanner::update_planner_state()
    {
        switch (current_state)
        {
        case PRE_ROTATE:
        {
            if (time_in_current_state() > config.goal_timeout)
            {
                ROS_ERROR_STREAM("FTCLocalPlannerROS: Error reaching goal. config.goal_timeout (" << config.goal_timeout << ") reached - Timeout in PRE_ROTATE phase.");
                is_crashed = true;
                return FINISHED;
            }
            if (abs(angle_error) * (180.0 / M_PI) < config.max_goal_angle_error)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: PRE_ROTATE finished. Starting following");
                return FOLLOWING;
            }
        }
        break;
        case FOLLOWING:
        {
            double distance = local_control_point.translation().norm();
            // check for crash
            if (distance > config.max_follow_distance)
            {
                ROS_ERROR_STREAM("FTCLocalPlannerROS: Robot is far away from global plan. distance (" << distance << ") > config.max_follow_distance (" << config.max_follow_distance << ") It probably has crashed.");
                is_crashed = true;
                return FINISHED;
            }

            // check if we're done following
            if (current_index == global_plan.size() - 2)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: switching planner to position mode");
                return WAITING_FOR_GOAL_APPROACH;
            }
        }
        break;
        case WAITING_FOR_GOAL_APPROACH:
        {
            double distance = local_control_point.translation().norm();
            if (time_in_current_state() > config.goal_timeout)
            {
                ROS_WARN_STREAM("FTCLocalPlannerROS: Could not reach goal position. config.goal_timeout (" << config.goal_timeout << ") reached - Attempting final rotation anyways.");
                return POST_ROTATE;
            }
            if (distance < config.max_goal_distance_error)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: Reached goal position.");
                return POST_ROTATE;
            }
        }
        break;
        case POST_ROTATE:
        {
            if (time_in_current_state() > config.goal_timeout)
            {
                ROS_WARN_STREAM("FTCLocalPlannerROS: Could not reach goal rotation. config.goal_timeout (" << config.goal_timeout << ") reached");
                return FINISHED;
            }
            if (abs(angle_error) * (180.0 / M_PI) < config.max_goal_angle_error)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: POST_ROTATE finished.");
                return FINISHED;
            }
        }
        break;
        case FINISHED:
        {
            // Nothing to do here
        }
        break;
        }

        return current_state;
    }

    void FTCPlanner::update_control_point(double dt)
    {

        switch (current_state)
        {
        case PRE_ROTATE:
            tf2::fromMsg(global_plan[0].pose, current_control_point);
            break;
        case FOLLOWING:
        {
            // Normal planner operation
            double straight_dist = distanceLookahead();
            double speed;
            if (straight_dist >= config.speed_fast_threshold)
            {
                speed = config.speed_fast;
            }
            else
            {
                speed = config.speed_slow;
            }

            if (speed > current_movement_speed)
            {
                // accelerate
                current_movement_speed += dt * config.acceleration;
                if (current_movement_speed > speed)
                    current_movement_speed = speed;
            }
            else if (speed < current_movement_speed)
            {
                // decelerate
                current_movement_speed -= dt * config.acceleration;
                if (current_movement_speed < speed)
                    current_movement_speed = speed;
            }

        double distance_to_move = dt * current_movement_speed;
        double angle_to_move = dt * config.speed_angular * (M_PI / 180.0);

        // --- Carrot leash (anti-runaway) -------------------------------------
        // local_control_point is the carrot expressed in base_link, so its
        // translation norm is how far AHEAD of the robot the carrot already is.
        // (It is updated at the end of the previous cycle.) If the robot has
        // fallen behind - bump, wheel slip, bogged down on a slope - we stop
        // advancing the carrot instead of letting the tracking error balloon.
        // This is the core fix for the post-stall catch-up surge.
        carrot_gated = false;
        if (config.carrot_max_lag > 0.0)
        {
            double along_track_lag = local_control_point.translation().x();   // lon error, +ve = carrot ahead
            double lateral_lag     = local_control_point.translation().y();
            double total_lag = std::hypot(along_track_lag, lateral_lag);
            if (total_lag > config.carrot_max_lag)
            {
                // Fully gate: hold the carrot until the robot catches up.
                distance_to_move = 0.0;
                angle_to_move = 0.0;
                carrot_gated = true;
            }
            else if (total_lag > 0.5 * config.carrot_max_lag)
            {
                // Soft zone: linearly taper carrot speed from full -> 0 as lag
                // grows from 50% -> 100% of the leash. Avoids a hard on/off that
                // itself causes jerk.
                double scale = (config.carrot_max_lag - total_lag) /
                               (0.5 * config.carrot_max_lag);
                if (scale < 0.0) scale = 0.0;
                if (scale > 1.0) scale = 1.0;
                distance_to_move *= scale;
                angle_to_move *= scale;
            }
        }

            Eigen::Affine3d nextPose, currentPose;
            while (angle_to_move > 0 && distance_to_move > 0 && current_index < global_plan.size() - 2)
            {

                tf2::fromMsg(global_plan[current_index].pose, currentPose);
                tf2::fromMsg(global_plan[current_index + 1].pose, nextPose);

                double pose_distance = (nextPose.translation() - currentPose.translation()).norm();

                Eigen::Quaternion<double> current_rot(currentPose.linear());
                Eigen::Quaternion<double> next_rot(nextPose.linear());

                double pose_distance_angular = current_rot.angularDistance(next_rot);

                if (pose_distance <= 0.0)
                {
                    ROS_WARN_STREAM("FTCLocalPlannerROS: Skipping duplicate point in global plan.");
                    current_index++;
                    continue;
                }

                double remaining_distance_to_next_pose = pose_distance * (1.0 - current_progress);
                double remaining_angular_distance_to_next_pose = pose_distance_angular * (1.0 - current_progress);

                if (remaining_distance_to_next_pose < distance_to_move &&
                    remaining_angular_distance_to_next_pose < angle_to_move)
                {
                    // we need to move further than the remaining distance_to_move. Skip to the next point and decrease distance_to_move.
                    current_progress = 0.0;
                    current_index++;
                    distance_to_move -= remaining_distance_to_next_pose;
                    angle_to_move -= remaining_angular_distance_to_next_pose;
                }
                else
                {
                    // we cannot reach the next point yet, so we update the percentage
                    double current_progress_distance =
                        (pose_distance * current_progress + distance_to_move) / pose_distance;
                    double current_progress_angle =
                        (pose_distance_angular * current_progress + angle_to_move) / pose_distance_angular;
                    current_progress = fmin(current_progress_angle, current_progress_distance);
                    if (current_progress > 1.0)
                    {
                        ROS_WARN_STREAM("FTCLocalPlannerROS: FTC PLANNER: Progress > 1.0");
                        //                    current_progress = 1.0;
                    }
                    distance_to_move = 0;
                    angle_to_move = 0;
                }
            }

            tf2::fromMsg(global_plan[current_index].pose, currentPose);
            tf2::fromMsg(global_plan[current_index + 1].pose, nextPose);
            // interpolate between points
            Eigen::Quaternion<double> rot1(currentPose.linear());
            Eigen::Quaternion<double> rot2(nextPose.linear());

            Eigen::Vector3d trans1 = currentPose.translation();
            Eigen::Vector3d trans2 = nextPose.translation();

            Eigen::Affine3d result;
            result.translation() = (1.0 - current_progress) * trans1 + current_progress * trans2;
            result.linear() = rot1.slerp(current_progress, rot2).toRotationMatrix();

            current_control_point = result;
        }
        break;
        case POST_ROTATE:
            tf2::fromMsg(global_plan[global_plan.size() - 1].pose, current_control_point);
            break;
        case WAITING_FOR_GOAL_APPROACH:
            break;
        case FINISHED:
            break;
        }

        {
            geometry_msgs::PoseStamped viz;
            viz.header = global_plan[current_index].header;
            viz.pose = tf2::toMsg(current_control_point);
            global_point_pub.publish(viz);
        }
        auto map_to_base = tf_buffer->lookupTransform("base_link", "map", ros::Time(), ros::Duration(1.0));
        tf2::doTransform(current_control_point, local_control_point, map_to_base);

        lat_error = local_control_point.translation().y();
        lon_error = local_control_point.translation().x();
        angle_error = local_control_point.rotation().eulerAngles(0, 1, 2).z();
    }

    void FTCPlanner::calculate_velocity_commands(double dt, geometry_msgs::TwistStamped &cmd_vel)
    {
        // check, if we're completely done
        if (current_state == FINISHED || is_crashed)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return;
        }

        // Anti-windup: when the carrot is gated (robot stuck/lagging) do NOT keep
        // charging the longitudinal integral, otherwise it saturates to ki_lon_max
        // and dumps that stored push the instant the robot breaks free.
        if (!carrot_gated)
        {
            i_lon_error += lon_error * dt;
        }
        i_lat_error += lat_error * dt;
        i_angle_error += angle_error * dt;

        if (i_lon_error > config.ki_lon_max)
        {
            i_lon_error = config.ki_lon_max;
        }
        else if (i_lon_error < -config.ki_lon_max)
        {
            i_lon_error = -config.ki_lon_max;
        }
        if (i_lat_error > config.ki_lat_max)
        {
            i_lat_error = config.ki_lat_max;
        }
        else if (i_lat_error < -config.ki_lat_max)
        {
            i_lat_error = -config.ki_lat_max;
        }
        if (i_angle_error > config.ki_ang_max)
        {
            i_angle_error = config.ki_ang_max;
        }
        else if (i_angle_error < -config.ki_ang_max)
        {
            i_angle_error = -config.ki_ang_max;
        }

        double d_lat = (lat_error - last_lat_error) / dt;
        double d_lon = (lon_error - last_lon_error) / dt;
        double d_angle = (angle_error - last_angle_error) / dt;

        last_lat_error = lat_error;
        last_lon_error = lon_error;
        last_angle_error = angle_error;
       
        // allow linear movement only if in following state

        if (current_state == FOLLOWING)
        {
            double lin_speed = lon_error * config.kp_lon + i_lon_error * config.ki_lon + d_lon * config.kd_lon;

            if (lin_speed < 0 && config.forward_only)
            {
                lin_speed = 0;
            }
            else
            {
                if (lin_speed > config.max_cmd_vel_speed)
                {
                    lin_speed = config.max_cmd_vel_speed;
                }
                else if (lin_speed < -config.max_cmd_vel_speed)
                {
                    lin_speed = -config.max_cmd_vel_speed;
                }

                if (lin_speed < 0)
                {
                    lat_error *= -1.0;
                }
            }

            // --- Output slew-rate (acceleration) limit ---------------------------
            // Cap how fast the COMMANDED velocity may change per cycle. This turns
            // any residual catch-up from a step into a ramp, which is the direct
            // antidote to pitching up / tipping over on upward slopes.
            if (config.max_cmd_vel_accel > 0.0 && dt > 0.0)
            {
                double max_delta = config.max_cmd_vel_accel * dt;
                double delta = lin_speed - last_cmd_vel_linear;
                if (delta > max_delta) lin_speed = last_cmd_vel_linear + max_delta;
                else if (delta < -max_delta) lin_speed = last_cmd_vel_linear - max_delta;
            }
            last_cmd_vel_linear = lin_speed;
            // ---------------------------------------------------------------------

            cmd_vel.twist.linear.x = lin_speed;
        }
        else
        {
            cmd_vel.twist.linear.x = 0.0;
            last_cmd_vel_linear = 0.0;   // reset ramp baseline when not following
        }

        if ((current_state == FOLLOWING) || (current_state == WAITING_FOR_GOAL_APPROACH))
        {
            // reduce angular error gain if there is a large lateral error
            double ang_gain_factor = 1.0;               
            if(config.lateral_priority_distance > 0.01) 
            {
                if(abs(lat_error) >= config.lateral_priority_distance)
                    ang_gain_factor = 0;
                else
                    ang_gain_factor = (config.lateral_priority_distance - abs(lat_error))/config.lateral_priority_distance;

                if(ang_gain_factor < 0.1)
                    ang_gain_factor = 0.1;
            }

            double ang_speed = (ang_gain_factor * (angle_error * config.kp_ang + i_angle_error * config.ki_ang + d_angle * config.kd_ang)) +
                               lat_error * config.kp_lat + i_lat_error * config.ki_lat + d_lat * config.kd_lat;

            if (ang_speed > config.max_cmd_vel_ang)
            {
                ang_speed = config.max_cmd_vel_ang;
            }
            else if (ang_speed < -config.max_cmd_vel_ang)
            {
                ang_speed = -config.max_cmd_vel_ang;
            }

            cmd_vel.twist.angular.z = ang_speed;
        }
        else
        {
            double ang_speed = angle_error * config.kp_ang + i_angle_error * config.ki_ang + d_angle * config.kd_ang;
            if (ang_speed > config.max_cmd_vel_ang)
            {
                ang_speed = config.max_cmd_vel_ang;
            }
            else if (ang_speed < -config.max_cmd_vel_ang)
            {
                ang_speed = -config.max_cmd_vel_ang;
            }

            cmd_vel.twist.angular.z = ang_speed;

            // check if robot oscillates
            bool is_oscillating = checkOscillation(cmd_vel);
            if (is_oscillating)
            {
                ang_speed = 0.0;//config.max_cmd_vel_ang;
                cmd_vel.twist.angular.z = ang_speed;
            }
        }

        if (config.debug_pid)
        {
            ftc_local_planner::PID debugPidMsg;
            debugPidMsg.stamp = ros::Time::now();

            // proportional
            debugPidMsg.kp_lat_set = lat_error * config.kp_lat;
            debugPidMsg.kp_lon_set = lon_error * config.kp_lon;
            debugPidMsg.kp_ang_set = angle_error * config.kp_ang;
    
            // integral
            debugPidMsg.ki_lat_set = i_lat_error * config.ki_lat;
            debugPidMsg.ki_lon_set = i_lon_error * config.ki_lon;
            debugPidMsg.ki_ang_set = i_angle_error * config.ki_ang;

            // diff
            debugPidMsg.kd_lat_set = d_lat * config.kd_lat;
            debugPidMsg.kd_lon_set = d_lon * config.kd_lon;
            debugPidMsg.kd_ang_set = d_angle * config.kd_ang;

            // errors
            debugPidMsg.lon_err = lon_error;
            debugPidMsg.lat_err = lat_error;
            debugPidMsg.ang_err = angle_error;

            // speeds
            debugPidMsg.ang_speed = cmd_vel.twist.angular.z;
            debugPidMsg.lin_speed = cmd_vel.twist.linear.x;
            debugPidMsg.lookahead_speed = lookahead_speed;

            pubPid.publish(debugPidMsg);
        }
    }

    bool FTCPlanner::getProgress(PlannerGetProgressRequest &req, PlannerGetProgressResponse &res)
    {
        res.index = current_index;
        return true;
    }

    // ---- stuck recovery (uglyfix) ------------------------------------------
    // Detect: in FOLLOWING, commanding forward, but the robot has not moved
    // kStuckMinMove within kStuckTimeout. Recover: (1) command exact zero for
    // kReleaseTime -- this is the "pause/unpause" trick, it releases the
    // firmware collision latch; (2) reverse straight for kReverseTime over
    // ground we just drove; (3) skip the carrot kSkipAhead * attempt past the
    // stall and resume. Repeated stalls near the same index escalate the skip;
    // after kMaxAttempts we declare a crash so MBF aborts and mower_logic's
    // normal failure handling repositions instead of looping here forever.
    bool FTCPlanner::recoveryActive(const geometry_msgs::PoseStamped &pose,
                                    geometry_msgs::TwistStamped &cmd_vel)
    {
        ros::Time now = ros::Time::now();
        Eigen::Vector2d p(pose.pose.position.x, pose.pose.position.y);

        if (recovery_phase == 0)
        {
            if (current_state != FOLLOWING || is_crashed)
            {
                stuck_ref_valid = false;
                return false;
            }
            if (!stuck_ref_valid || (p - stuck_ref_pos).norm() > kStuckMinMove)
            {
                stuck_ref_pos = p;                 // progress made: restart the window
                stuck_ref_time = now;
                stuck_ref_valid = true;
                return false;
            }
            if ((now - stuck_ref_time).toSec() < kStuckTimeout ||
                std::fabs(last_cmd_vel_linear) < 0.02)
            {
                return false;                      // not stuck (or not even trying to move)
            }
            // Stuck. Same neighbourhood as the last recovery counts as a retry.
            if (recovery_attempts > 0 && current_index - recovery_last_index < 30)
                ++recovery_attempts;
            else
                recovery_attempts = 1;
            recovery_last_index = current_index;
            if (recovery_attempts > kMaxAttempts)
            {
                ROS_ERROR_STREAM("FTCLocalPlannerROS: stuck " << kMaxAttempts
                                 << "x near index " << current_index
                                 << " - giving up, reporting collision.");
                is_crashed = true;                 // next cycle returns RET_COLLISION
                cmd_vel.twist.linear.x = 0;
                cmd_vel.twist.angular.z = 0;
                return true;
            }
            ROS_WARN_STREAM("FTCLocalPlannerROS: no movement for " << kStuckTimeout
                            << "s at index " << current_index << " - recovery attempt "
                            << recovery_attempts << "/" << kMaxAttempts
                            << " (release latch, reverse, skip ahead).");
            recovery_phase = 1;
            recovery_phase_start = now;
        }

        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.angular.z = 0;
        double t = (now - recovery_phase_start).toSec();
        switch (recovery_phase)
        {
        case 1:                                    // exact zero: firmware latch releases
            if (t >= kReleaseTime) { recovery_phase = 2; recovery_phase_start = now; }
            break;
        case 2:                                    // straight back, no steering
            cmd_vel.twist.linear.x = -kReverseSpeed;
            if (t >= kReverseTime) { recovery_phase = 3; recovery_phase_start = now; }
            break;
        case 3:                                    // skip past the stall and resume
        {
            // Keep the carrot within the max_follow_distance crash check: we
            // just reversed kReverseSpeed*kReverseTime, the skip adds on top.
            double allowed = 0.7 * config.max_follow_distance - kReverseSpeed * kReverseTime;
            double skip = std::min(kSkipAhead * recovery_attempts, std::max(0.2, allowed));
            skipAhead(skip);
            i_lon_error = i_lat_error = i_angle_error = 0.0;   // clean PID restart
            last_cmd_vel_linear = 0.0;
            current_movement_speed = config.speed_slow;
            stuck_ref_pos = p;                     // fresh detection window for the retry
            stuck_ref_time = now;
            recovery_phase = 0;
            ROS_INFO_STREAM("FTCLocalPlannerROS: recovery done, skipped " << skip
                            << " m of path, resuming at index " << current_index << ".");
            return false;                          // normal control resumes this cycle
        }
        }
        return true;
    }

    // Advance the carrot `dist` metres along the plan (never past the goal).
    void FTCPlanner::skipAhead(double dist)
    {
        current_progress = 0.0;
        while (dist > 0.0 && current_index < global_plan.size() - 2)
        {
            Eigen::Affine3d a, b;
            tf2::fromMsg(global_plan[current_index].pose, a);
            tf2::fromMsg(global_plan[current_index + 1].pose, b);
            dist -= (b.translation() - a.translation()).norm();
            current_index++;
        }
    }

    bool FTCPlanner::checkCollision(int max_points)
    {
        unsigned int x;
        unsigned int y;

        std::vector<geometry_msgs::Point> footprint;
        visualization_msgs::Marker obstacle_marker;

        if (!config.check_obstacles)
        {
            return false;
        }
        // maximal costs
        unsigned char previous_cost = 255;
        // ensure look ahead not out of plan
        if (global_plan.size() < max_points)
        {
            max_points = global_plan.size();
        }

        // calculate cost of footprint at robots actual pose
        if (config.obstacle_footprint)
        {
        costmap->getOrientedFootprint(footprint);
        for (int i = 0; i < footprint.size(); i++)
        {
            // check cost of each point of footprint
            if (costmap_map_->worldToMap(footprint[i].x, footprint[i].y, x, y))
            {
                unsigned char costs = costmap_map_->getCost(x, y);
                if (costs >= costmap_2d::LETHAL_OBSTACLE)
                {
                    ROS_WARN("FTCLocalPlannerROS: Possible collision of footprint at actual pose. Stop local planner.");
                    return true;
                }
            }
        }
        }

        for (int i = 0; i < max_points; i++)
        {
            geometry_msgs::PoseStamped x_pose;
            int index = current_index + i;
            if (index > global_plan.size())
            {
                index = global_plan.size();
            }
            x_pose = global_plan[index];

            if (costmap_map_->worldToMap(x_pose.pose.position.x, x_pose.pose.position.y, x, y))
            {
                unsigned char costs = costmap_map_->getCost(x, y);
                if (config.debug_obstacle)
                {
                    debugObstacle(obstacle_marker, x, y, costs, max_points);
                }
                // Near at obstacel
                if (costs > 0)
                {
                    // Possible collision
                    if (costs > 127 && costs > previous_cost)
                    {
                        ROS_WARN("FTCLocalPlannerROS: Possible collision. Stop local planner.");
                        return true;
                    }
                }
                previous_cost = costs;
            }
        }
        return false;
    }

    bool FTCPlanner::checkOscillation(geometry_msgs::TwistStamped &cmd_vel)
    {
        bool oscillating = false;
        // detect and resolve oscillations
        if (config.oscillation_recovery)
        {
            // oscillating = true;
            double max_vel_theta = config.max_cmd_vel_ang;
            double max_vel_current = config.max_cmd_vel_speed;

            failure_detector_.update(cmd_vel, config.max_cmd_vel_speed, config.max_cmd_vel_speed, max_vel_theta,
                                     config.oscillation_v_eps, config.oscillation_omega_eps);

            oscillating = failure_detector_.isOscillating();

            if (oscillating) // we are currently oscillating
            {
                if (!oscillation_detected_) // do we already know that robot oscillates?
                {
                    time_last_oscillation_ = ros::Time::now(); // save time when oscillation was detected
                    oscillation_detected_ = true;
                }
                // calculate duration of actual oscillation
                bool oscillation_duration_timeout = !((ros::Time::now() - time_last_oscillation_).toSec() < config.oscillation_recovery_min_duration); // check how long we oscillate
                if (oscillation_duration_timeout)
                {
                    if (!oscillation_warning_) // ensure to send warning just once instead of spamming around
                    {
                        ROS_WARN("FTCLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
                        oscillation_warning_ = true;
                    }
                    return true;
                }
                return false; // oscillating but timeout not reached
            }
            else
            {
                // not oscillating
                time_last_oscillation_ = ros::Time::now(); // save time when oscillation was detected
                oscillation_detected_ = false;
                oscillation_warning_ = false;
                return false;
            }
        }
        return false; // no check for oscillation
    }

    void FTCPlanner::debugObstacle(visualization_msgs::Marker &obstacle_points, double x, double y, unsigned char cost, int maxIDs)
    {
        if (obstacle_points.points.empty())
        {
            obstacle_points.header.frame_id = costmap->getGlobalFrameID();
            obstacle_points.header.stamp = ros::Time::now();
            obstacle_points.action = visualization_msgs::Marker::ADD;
            obstacle_points.pose.orientation.w = 1.0;
            obstacle_points.type = visualization_msgs::Marker::POINTS;
            obstacle_points.scale.x = 0.2;
            obstacle_points.scale.y = 0.2;
        }
        obstacle_points.id = obstacle_points.points.size() + 1;

        if (cost < 127)
        {
            obstacle_points.color.g = 1.0f;
        }

        if (cost >= 127 && cost < 255)
        {
            obstacle_points.color.r = 1.0f;
        }
        obstacle_points.color.a = 1.0;
        geometry_msgs::Point p;
        costmap_map_->mapToWorld(x, y, p.x, p.y);
        p.z = 0;

        obstacle_points.points.push_back(p);
        if (obstacle_points.points.size() >= maxIDs || cost > 0)
        {
            obstacle_marker_pub.publish(obstacle_points);
            obstacle_points.points.clear();
        }
    }
}