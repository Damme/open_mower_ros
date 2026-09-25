//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "SystemModel.hpp"

#include "ros/ros.h"

#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "xbot_msgs/AbsolutePose.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "xbot_positioning_core.h"
#include "xbot_msgs/WheelTick.h"
#include "xbot_positioning/KalmanState.h"
#include "xbot_positioning/GPSControlSrv.h"
#include "xbot_positioning/SetPoseSrv.h"

ros::Publisher odometry_pub;
ros::Publisher xbot_absolute_pose_pub;

// Debug Publishers
ros::Publisher kalman_state;
ros::Publisher dbg_expected_motion_vector;

// The kalman filters
xbot::positioning::xbot_positioning_core core{};

// True, if we don't want to do gyro calibration on launch
bool skip_gyro_calibration;

// True, if we have wheel ticks (i.e. last_ticks is valid)
bool has_ticks;
xbot_msgs::WheelTick last_ticks;
bool has_gps;
xbot_msgs::AbsolutePose last_gps;

// True, if last_imu is valid and gyro_offset is valid
bool has_gyro;
sensor_msgs::Imu last_imu;
ros::Time gyro_calibration_start;
double gyro_offset;
int gyro_offset_samples;

// Current speed calculated by wheel ticks
double vx = 0.0;

// --- On-the-fly calibration state ---------------------------------------------
// Both calibrations (gyro bias, wheel scale) follow the same rule: never adapt on
// a single sample, never adapt on unverified data. Each collects TWO independent
// estimates and only commits when they AGREE; both are hard-clamped so a bad patch
// can never run the value away; and the wheel scale additionally waits out a
// confirmation horizon so a DELAYED RTK-loss flag cannot poison it retroactively.

// Last time either wheel produced ticks (physical motion) and last time a tick
// message arrived at all (freshness -- a dead tick topic must not read as
// "stationary", and must not leave a stale vx integrating forever).
ros::Time last_wheel_motion_time(0.0);
ros::Time last_tick_time(0.0);
double tick_timeout;           // wheel ticks older than this => wheel data unusable

// Adaptive forward wheel scale (ticks->metres). 1.0 = nominal.
double vx_scale = 1.0;

// Gyro bias. gyro_offset_boot is the startup calibration and is the anchor the
// running offset is clamped around, so ZARU can never drift far from a value that
// was measured while genuinely stationary.
double gyro_offset_boot = 0.0;
bool gyro_boot_valid = false;
double gyro_offset_var_acc = 0.0;   // to validate the boot calibration itself

// ZARU (zero angular rate update) -- two sub-windows that must agree.
bool enable_zaru;
double zaru_rate;              // offset adaptation per accepted WINDOW (not per sample)
double zaru_max_rate;          // gyro must read below this for "stationary" to be believed
double zaru_agree_tol;         // max |meanA - meanB| to accept a window pair
int zaru_window_samples;       // samples per sub-window
double gyro_offset_band;       // max |gyro_offset - gyro_offset_boot|
std::vector<double> zaru_a, zaru_b;
double stationary_settle_time; // seconds of no wheel motion before trusting "stationary"

// Wheel-scale calibration -- a window accumulates over WHEEL DISTANCE (not time),
// tolerating per-sample jitter, and is judged once at the end from GPS geometry:
// straight-line displacement vs summed wheel distance, split into two halves that
// must agree. Only a HARD break (RTK loss / stop) drops a window mid-flight.
bool enable_wheel_cal;
double wheel_cal_rate;          // vx_scale adaptation per accepted WINDOW (EMA)
double wheel_cal_min_speed;     // min wheel speed [m/s] to be "moving"
double wheel_cal_min_dist;      // wheel distance [m] per window (split into two halves)
double wheel_cal_agree_tol;     // max |ratioA - ratioB| between the two halves
double wheel_cal_straight_tol;  // max (gA+gB)/gT - 1 : 3 GPS points collinear (wiggle)
double wheel_cal_max_hdg;       // max |heading change| over the window [rad] (curves)
double wheel_cal_max_asym;      // max sum|dL-dR| / distance over the window (slip/turn)
double wheel_cal_min_scale, wheel_cal_max_scale;   // hard clamp on vx_scale

// Raw wheel travel and left/right asymmetry accumulated between GPS samples.
double wheel_dist_accum = 0.0;
double wheel_asym_accum = 0.0;
double max_abs_yaw_since_gps = 0.0;   // (retained; no longer gates)

// GPS quality gating for calibration.
double cal_settle_time;        // good RTK required BEFORE a window may start
double cal_confirm_time;       // good RTK required AFTER it, before committing
ros::Time last_bad_gps_time(0.0);   // HARD problems only: non-fixed / outlier / gap / disabled

// Active window (distance-based).
bool win_active = false;
int win_half = 0;
ros::Time win_t0;
double win_p0x = 0, win_p0y = 0, win_pmx = 0, win_pmy = 0;
double win_da = 0, win_db = 0, win_asym = 0, win_hdg0 = 0;
unsigned cal_windows_seen = 0, cal_windows_accepted = 0;   // diagnostics

// Candidate awaiting the RTK confirmation horizon.
bool cal_pending = false;
double cal_pending_ratio = 1.0;
ros::Time cal_pending_t0, cal_pending_t1;

double max_imu_dt;             // reject absurd / negative IMU dt
double orientation_min_speed;  // min GPS speed to trust motion_vector as a heading

// A HARD GPS problem (not-fixed / outlier / gap / disabled -- NOT merely marginal
// accuracy, which the metre-scale displacement averages out). Drops any window in
// flight and vetoes a pending candidate, so a late RTK-loss flag still nullifies a
// window it overlaps.
static void noteBadGps(const ros::Time &t) {
    last_bad_gps_time = t;
    win_active = false;
    cal_pending = false;
}
// ------------------------------------------------------------------------------

// Min speed for motion vector to be fed into kalman filter.
// SUPERSEDED by orientation_min_speed: 0.01 m/s was low enough that the heading
// was being taken from the direction of a nearly-zero (i.e. pure noise) velocity
// vector. Kept only so an existing param set still loads.
double min_speed = 0.0;

// Max position accuracy to allow for GPS updates
double max_gps_accuracy;

// Seconds without an accepted GPS fix before the pose stops being flagged as
// GPS-backed (FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE) -- i.e. the RTK-loss reaction time.
double recent_gps_timeout;

// True, if we should publish debug topics (expected motion vector and kalman state)
bool publish_debug;

// Antenna offset (offset between point of rotation and antenna)
double antenna_offset_x, antenna_offset_y;

nav_msgs::Odometry odometry;
xbot_positioning::KalmanState state_msg;
xbot_msgs::AbsolutePose xb_absolute_pose_msg;

bool gps_enabled = true;
int gps_outlier_count = 0;
int valid_gps_samples = 0;

ros::Time last_gps_time(0.0);


void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
    if(!has_gyro) {
        if(!skip_gyro_calibration) {
            if (gyro_offset_samples == 0) {
                ROS_INFO_STREAM("Started gyro calibration");
                gyro_calibration_start = msg->header.stamp;
                gyro_offset = 0;
                gyro_offset_var_acc = 0;
            }
            gyro_offset += msg->angular_velocity.z;
            gyro_offset_var_acc += msg->angular_velocity.z * msg->angular_velocity.z;
            gyro_offset_samples++;
            if ((msg->header.stamp - gyro_calibration_start).toSec() < 5) {
                last_imu = *msg;
                return;
            }
            has_gyro = true;
            if (gyro_offset_samples > 0) {
                const double n = (double)gyro_offset_samples;
                gyro_offset /= n;
                // Validate the boot calibration itself: if the mower was moved or
                // bumped during it, the mean is not a bias and everything anchored
                // to it (the ZARU clamp) would be anchored to garbage.
                const double var = std::max(0.0, gyro_offset_var_acc / n - gyro_offset * gyro_offset);
                const double sd = std::sqrt(var);
                if (sd > zaru_max_rate) {
                    ROS_WARN_STREAM("Gyro boot calibration looks unreliable (stddev "
                                    << sd << " rad/s) -- was the mower moving? "
                                    << "Bias adaptation will be left unanchored.");
                    gyro_boot_valid = false;
                } else {
                    gyro_boot_valid = true;
                }
            } else {
                gyro_offset = 0;
            }
            gyro_offset_boot = gyro_offset;
            gyro_offset_samples = 0;
            ROS_INFO_STREAM("Calibrated gyro offset: " << gyro_offset);
        } else {
            ROS_WARN("Skipped gyro calibration");
            gyro_offset_boot = gyro_offset;
            gyro_boot_valid = (gyro_offset != 0.0);
            has_gyro = true;
            return;
        }
    }

    // Guard the timestep before it can reach the filter. A duplicate stamp gives
    // dt == 0 (-> inf in the models), an out-of-order one gives dt < 0 (-> a
    // backwards predict). One NaN here destroys the whole state.
    const double imu_dt = (msg->header.stamp - last_imu.header.stamp).toSec();
    if (imu_dt <= 0.0 || imu_dt > max_imu_dt) {
        ROS_WARN_STREAM_THROTTLE(5.0, "Skipping IMU sample with implausible dt: " << imu_dt);
        last_imu = *msg;
        return;
    }

    const double gyro_z = msg->angular_velocity.z;

    // Is the wheel data even usable? A dead/stalled tick topic must NOT be read as
    // "no motion" -- that is what let a rotating mower be declared stationary.
    const bool ticks_fresh =
        has_ticks && (msg->header.stamp - last_tick_time).toSec() < tick_timeout;

    // Stationary requires TWO independent witnesses that agree:
    //   wheels: no ticks for the settle time (and the tick feed is alive), and
    //   gyro:   it is itself reading near zero rate.
    // A diff-drive cannot rotate without ticks only if it is under its own power;
    // it can still be pushed, slide on a slope, pivot off an obstacle, or lose the
    // encoder feed. In all of those the old tick-only test declared "stationary",
    // forced yaw_rate to 0, and fed the REAL turn rate into the bias -- a phantom
    // constant yaw rate afterwards, i.e. the mower curving/looping in dead reckoning.
    const bool wheels_quiet = ticks_fresh &&
        (msg->header.stamp - last_wheel_motion_time).toSec() > stationary_settle_time;
    const bool gyro_quiet = std::fabs(gyro_z - gyro_offset) < zaru_max_rate;
    const bool stationary = wheels_quiet && gyro_quiet;

    if (enable_zaru && stationary) {
        // Collect two independent sub-windows and only commit if they agree. A
        // single noisy or half-moving sample can no longer move the bias.
        if ((int)zaru_a.size() < zaru_window_samples) {
            zaru_a.push_back(gyro_z);
        } else if ((int)zaru_b.size() < zaru_window_samples) {
            zaru_b.push_back(gyro_z);
        }
        if ((int)zaru_b.size() >= zaru_window_samples) {
            const double mA = std::accumulate(zaru_a.begin(), zaru_a.end(), 0.0) / zaru_a.size();
            const double mB = std::accumulate(zaru_b.begin(), zaru_b.end(), 0.0) / zaru_b.size();
            if (std::fabs(mA - mB) <= zaru_agree_tol) {
                double target = gyro_offset + zaru_rate * (0.5 * (mA + mB) - gyro_offset);
                if (gyro_boot_valid) {   // never wander far from the trusted boot bias
                    target = std::min(std::max(target, gyro_offset_boot - gyro_offset_band),
                                      gyro_offset_boot + gyro_offset_band);
                }
                gyro_offset = target;
            } else {
                ROS_WARN_STREAM_THROTTLE(10.0, "ZARU windows disagree ("
                                         << mA << " vs " << mB << "), discarded.");
            }
            zaru_a.clear();
            zaru_b.clear();
        }
    } else {
        zaru_a.clear();   // any motion (or doubt) invalidates a partial window
        zaru_b.clear();
    }

    // Only force zero rate when BOTH witnesses agree we are stopped.
    const double yaw_rate = stationary ? 0.0 : (gyro_z - gyro_offset);

    // Never integrate a stale wheel speed: vx is only written by onWheelTicks, so
    // if that topic dies it would otherwise hold its last value forever and the
    // filter would dead-reckon forward at that speed indefinitely.
    const double vx_use = ticks_fresh ? vx : 0.0;

    // Peak yaw rate between GPS samples -- the wheel calibration needs the mower to
    // have been straight for the WHOLE window, not just at the instant a GPS
    // message happened to land.
    max_abs_yaw_since_gps = std::max(max_abs_yaw_since_gps, std::fabs(yaw_rate));

    core.predict(vx_use * vx_scale, yaw_rate, imu_dt);
    auto x = core.updateSpeed(vx_use * vx_scale, yaw_rate, 0.01);

    ROS_INFO_STREAM_THROTTLE(10.0, "calib: gyro_offset=" << gyro_offset
                                   << " (boot " << gyro_offset_boot << ")"
                                   << " vx_scale=" << vx_scale
                                   << (stationary ? " [stationary]" : "")
                                   << (ticks_fresh ? "" : " [WHEEL TICKS STALE]"));

    odometry.header.stamp = ros::Time::now();
    odometry.header.seq++;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = x.x_pos();
    odometry.pose.pose.position.y = x.y_pos();
    odometry.pose.pose.position.z = 0;
    tf2::Quaternion q(0.0, 0.0, x.theta());
    odometry.pose.pose.orientation = tf2::toMsg(q);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header = odometry.header;
    odom_trans.child_frame_id = odometry.child_frame_id;
    odom_trans.transform.translation.x = odometry.pose.pose.position.x;
    odom_trans.transform.translation.y = odometry.pose.pose.position.y;
    odom_trans.transform.translation.z = odometry.pose.pose.position.z;
    odom_trans.transform.rotation = odometry.pose.pose.orientation;

    static tf2_ros::TransformBroadcaster transform_broadcaster;
    transform_broadcaster.sendTransform(odom_trans);

    if(publish_debug) {
        auto state = core.getState();
        state_msg.x = state.x();
        state_msg.y = state.y();
        state_msg.theta = state.theta();
        state_msg.vx = state.vx();
        state_msg.vr = state.vr();

        kalman_state.publish(state_msg);
    }

    odometry_pub.publish(odometry);

    xb_absolute_pose_msg.header = odometry.header;
    xb_absolute_pose_msg.sensor_stamp = 0;
    xb_absolute_pose_msg.received_stamp = 0;
    xb_absolute_pose_msg.source = xbot_msgs::AbsolutePose::SOURCE_SENSOR_FUSION;
    xb_absolute_pose_msg.flags = xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING;

    xb_absolute_pose_msg.orientation_valid = true;
    // TODO: send motion vector
    xb_absolute_pose_msg.motion_vector_valid = false;
    // TODO: set real value from kalman filter, not the one from the GPS.
    // A fix only counts as recent while it is being ACCEPTED (RTK fixed, accurate,
    // inlier). Rejected fixes never update last_gps_time, so on RTK loss the flag
    // drops after recent_gps_timeout and mower_logic reacts. This used to be a
    // hard-coded 10 s, during which the stale accuracy of the last good fix was
    // reported and the mower kept mowing on dead reckoning.
    const bool gps_recent = has_gps &&
        (ros::Time::now() - last_gps_time).toSec() < recent_gps_timeout;
    if(gps_recent) {
        xb_absolute_pose_msg.flags |= xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;
        xb_absolute_pose_msg.position_accuracy = last_gps.position_accuracy;
    } else {
        xb_absolute_pose_msg.position_accuracy = 999;
    }
    // TODO: set real value
    xb_absolute_pose_msg.orientation_accuracy = 0.01;
    xb_absolute_pose_msg.pose = odometry.pose;
    xb_absolute_pose_msg.vehicle_heading = x.theta();
    xb_absolute_pose_msg.motion_heading = x.theta();

    xbot_absolute_pose_pub.publish(xb_absolute_pose_msg);


    last_imu = *msg;
}

void onWheelTicks(const xbot_msgs::WheelTick::ConstPtr &msg) {
    if(!has_ticks) {
        last_ticks = *msg;
        last_tick_time = msg->stamp;
        last_wheel_motion_time = msg->stamp;   // never leave this at epoch
        has_ticks = true;
        return;
    }
    last_tick_time = msg->stamp;               // feed is alive (freshness for onImu)

    double dt = (msg->stamp - last_ticks.stamp).toSec();

    double d_wheel_l = (double) (msg->wheel_ticks_rl - last_ticks.wheel_ticks_rl) * (1/(double)msg->wheel_tick_factor);
    double d_wheel_r = (double) (msg->wheel_ticks_rr - last_ticks.wheel_ticks_rr) * (1/(double)msg->wheel_tick_factor);

    if(msg->wheel_direction_rl) {
        d_wheel_l *= -1.0;
    }
    if(msg->wheel_direction_rr) {
        d_wheel_r *= -1.0;
    }

    // Mark physical motion: any tick on either wheel means we're translating or
    // rotating in place. Absence of this for stationary_settle_time => wheels quiet.
    // (Checked per-wheel, so a pure rotation -- where dL = -dR and the average is
    // zero -- still counts as motion.)
    if (std::abs(d_wheel_l) > 1e-6 || std::abs(d_wheel_r) > 1e-6) {
        last_wheel_motion_time = msg->stamp;
    }

    // Guard dt before dividing: a duplicate stamp gives inf, an out-of-order one a
    // negative speed. Either would go straight into the filter.
    if (dt <= 1e-6 || dt > 1.0) {
        ROS_WARN_STREAM_THROTTLE(5.0, "Skipping wheel tick sample with implausible dt: " << dt);
        last_ticks = *msg;
        return;
    }

    double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
    vx = d_ticks / dt;

    // Raw (unscaled) travel and left/right asymmetry since the last GPS sample.
    // Asymmetry while the gyro says we are going straight is a slip signature: one
    // wheel is turning without the ground moving under it.
    wheel_dist_accum += std::abs(d_ticks);
    wheel_asym_accum += std::abs(d_wheel_l - d_wheel_r);

    last_ticks = *msg;
}

bool setGpsState(xbot_positioning::GPSControlSrvRequest &req, xbot_positioning::GPSControlSrvResponse &res) {
    gps_enabled = req.gps_enabled;
    return true;
}

bool setPose(xbot_positioning::SetPoseSrvRequest &req, xbot_positioning::SetPoseSrvResponse &res) {
    tf2::Quaternion q;
    tf2::fromMsg(req.robot_pose.orientation, q);


    tf2::Matrix3x3 m(q);
    double unused1, unused2, yaw;

    m.getRPY(unused1, unused2, yaw);
    core.setState(req.robot_pose.position.x, req.robot_pose.position.y, yaw,0,0);
    return true;
}

void onPose(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    // Every rejection path below MUST be recorded via noteBadGps(). The old code
    // returned silently, so the calibration never learned that the fix had been
    // bad -- it simply resumed on the next good-looking sample as if nothing had
    // happened. Recording them is what lets a window be thrown away retroactively
    // when an RTK-loss flag arrives late.
    const ros::Time t_msg = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    if(!gps_enabled) {
        ROS_INFO_STREAM_THROTTLE(1, "dropping GPS update, since gps_enabled = false.");
        noteBadGps(t_msg);
        return;
    }
    // TODO fuse with high covariance?
    if((msg->flags & (xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED)) == 0) {
        ROS_INFO_STREAM_THROTTLE(1, "Dropped GPS update, since it's not RTK Fixed");
        noteBadGps(t_msg);
        return;
    }

    if(msg->position_accuracy > max_gps_accuracy) {
        ROS_INFO_STREAM_THROTTLE(1, "Dropped GPS update, since it's not accurate enough. Accuracy was: " << msg->position_accuracy << ", limit is:" << max_gps_accuracy);
        noteBadGps(t_msg);
        return;
    }

    double time_since_last_gps = (ros::Time::now() - last_gps_time).toSec();
    if (time_since_last_gps > 5.0) {
        ROS_WARN_STREAM("Last GPS was " << time_since_last_gps << " seconds ago.");
        has_gps = false;
        valid_gps_samples = 0;
        gps_outlier_count = 0;
        last_gps = *msg;
        // we have GPS for next time
        last_gps_time = ros::Time::now();
        noteBadGps(t_msg);
        return;
    }

    tf2::Vector3 gps_pos(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    tf2::Vector3 last_gps_pos(last_gps.pose.pose.position.x,last_gps.pose.pose.position.y,last_gps.pose.pose.position.z);

    double distance_to_last_gps = (last_gps_pos - gps_pos).length();

    if (distance_to_last_gps < 5.0) {
        // inlier, we treat it normally
        // store the gps as last
        last_gps = *msg;
        last_gps_time = ros::Time::now();

        gps_outlier_count = 0;
        valid_gps_samples++;

        if (!has_gps && valid_gps_samples > 10) {
            ROS_INFO_STREAM("GPS data now valid");
            ROS_INFO_STREAM("First GPS data, moving kalman filter to " << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y);
            // we don't even have gps yet, set odometry to first estimate
            core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.001);

            has_gps = true;
        } else if (has_gps) {
            // gps was valid before, we apply the filter
            core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, 500.0);
            if(publish_debug) {
                auto m = core.om2.h(core.ekf.getState());
                geometry_msgs::Vector3 dbg;
                dbg.x = m.vx();
                dbg.y = m.vy();
                dbg_expected_motion_vector.publish(dbg);
            }
            const double gps_speed = std::sqrt(std::pow(msg->motion_vector.x, 2) +
                                               std::pow(msg->motion_vector.y, 2));
            // Heading from the motion vector is only meaningful when the vehicle is
            // actually moving: at a few cm/s the direction of a noisy velocity vector
            // is essentially random, and feeding that in corrupts theta directly.
            if(gps_speed >= orientation_min_speed) {
                core.updateOrientation2(msg->motion_vector.x, msg->motion_vector.y, 10000.0);
            }

            // ---- Wheel forward-scale calibration -----------------------------
            // NOT from the instantaneous speed ratio. |motion_vector| is the
            // magnitude of a noisy vector, and E[|v+noise|] > |v| ALWAYS -- so on a
            // marginal fix the GPS speed reads systematically HIGH, the ratio reads
            // high, and vx_scale ratchets UP. That is the "drives way outside in
            // dead reckoning" failure: the filter believes it has travelled farther
            // than it has. (Measured: at 0.4 m/s velocity noise the bias is +36%.)
            //
            // Instead: integrate over a window and compare straight-line GPS
            // DISPLACEMENT against summed wheel distance. Displacement over metres
            // swamps centimetre noise, so the bias vanishes. The window is split in
            // half, and the two halves must AGREE before anything is applied --
            // slip and noise show up as disagreement. The result is clamped, and it
            // is held back until the RTK has stayed good past a confirmation
            // horizon, so a late-arriving RTK-loss flag can still veto it.
            if (enable_wheel_cal) {
                const double d_w    = wheel_dist_accum;    // wheel travel since last GPS sample
                const double d_asym = wheel_asym_accum;
                wheel_dist_accum = 0.0;
                wheel_asym_accum = 0.0;
                max_abs_yaw_since_gps = 0.0;

                const double now_s = t_msg.toSec();
                const double px = msg->pose.pose.position.x;
                const double py = msg->pose.pose.position.y;
                const double hdg = core.getState().theta();     // filtered heading

                // Only a HARD RTK problem or a stop drops a window; marginal accuracy
                // does NOT (metre-scale displacement averages centimetre noise out, and
                // the two-half agreement rejects windows where it didn't).
                const bool rtk_settled = (now_s - last_bad_gps_time.toSec()) > cal_settle_time;
                const bool moving      = std::fabs(vx) > wheel_cal_min_speed;

                if (!rtk_settled || !moving) {
                    win_active = false;                        // hard break: abandon window
                } else if (!win_active) {
                    win_active = true; win_half = 0;
                    win_da = win_db = win_asym = 0.0;
                    win_p0x = px; win_p0y = py; win_hdg0 = hdg; win_t0 = t_msg;
                } else if (win_half == 0) {
                    win_da += d_w; win_asym += d_asym;
                    if (win_da >= 0.5 * wheel_cal_min_dist) {   // first half done -> record midpoint
                        win_half = 1; win_pmx = px; win_pmy = py;
                    }
                } else {
                    win_db += d_w; win_asym += d_asym;
                    if (win_da + win_db >= wheel_cal_min_dist) {
                        // GPS geometry: two half-chords and the full chord.
                        const double gA = std::hypot(win_pmx - win_p0x, win_pmy - win_p0y);
                        const double gB = std::hypot(px - win_pmx, py - win_pmy);
                        const double gT = std::hypot(px - win_p0x, py - win_p0y);
                        const double rA = gA / std::max(win_da, 1e-6);
                        const double rB = gB / std::max(win_db, 1e-6);
                        const double r  = gT / std::max(win_da + win_db, 1e-6);   // best estimate
                        double dh = std::atan2(std::sin(hdg - win_hdg0), std::cos(hdg - win_hdg0));

                        const bool turned   = std::fabs(dh) > wheel_cal_max_hdg;      // curve
                        const bool straight = (gA + gB) <= gT * (1.0 + wheel_cal_straight_tol);
                        const bool agree    = std::fabs(rA - rB) <= wheel_cal_agree_tol;
                        const bool no_slip  = win_asym <= wheel_cal_max_asym * std::max(win_da + win_db, 1e-6);
                        const bool sane     = r > wheel_cal_min_scale && r < wheel_cal_max_scale;
                        cal_windows_seen++;

                        if (!turned && straight && agree && no_slip && sane) {
                            cal_pending = true;
                            cal_pending_ratio = r;
                            cal_pending_t0 = win_t0;
                            cal_pending_t1 = t_msg;
                            ROS_INFO_STREAM("wheel cal: window ok r=" << r << " (rA=" << rA
                                            << " rB=" << rB << "), holding for confirmation");
                        } else {
                            ROS_INFO_STREAM_THROTTLE(5.0, "wheel cal: window rejected -- "
                                << (turned ? "turning " : "") << (!straight ? "not-straight " : "")
                                << (!agree ? "halves-disagree " : "") << (!no_slip ? "slip " : "")
                                << (!sane ? "out-of-range " : "")
                                << "(r=" << r << " dh=" << dh << ")");
                        }
                        win_active = false;
                    }
                }

                // Commit after the confirmation horizon, unless a hard RTK problem
                // landed in or after the window (a late RTK-loss flag still catches it).
                if (cal_pending && (now_s - cal_pending_t1.toSec()) > cal_confirm_time) {
                    if ((cal_pending_t0.toSec() - last_bad_gps_time.toSec()) > cal_settle_time) {
                        const double before = vx_scale;
                        vx_scale += wheel_cal_rate * (cal_pending_ratio - vx_scale);
                        vx_scale = std::min(std::max(vx_scale, wheel_cal_min_scale),
                                            wheel_cal_max_scale);
                        cal_windows_accepted++;
                        ROS_INFO_STREAM("wheel cal: ACCEPTED ratio " << cal_pending_ratio
                                        << ", vx_scale " << before << " -> " << vx_scale
                                        << " (" << cal_windows_accepted << " accepted / "
                                        << cal_windows_seen << " seen)");
                    } else {
                        ROS_WARN_STREAM("wheel cal: RTK degraded in confirmation horizon, discarded.");
                    }
                    cal_pending = false;
                }

                ROS_INFO_STREAM_THROTTLE(15.0, "wheel cal status: vx_scale=" << vx_scale
                    << " active=" << win_active << " da=" << win_da << " db=" << win_db
                    << " seen=" << cal_windows_seen << " accepted=" << cal_windows_accepted
                    << (rtk_settled ? "" : " [rtk-unsettled]") << (moving ? "" : " [stopped]"));
            }
        }
    } else {
        ROS_WARN_STREAM("GPS outlier found. Distance was: " << distance_to_last_gps);
        noteBadGps(t_msg);                     // an outlier invalidates any window in flight
        gps_outlier_count++;
        // ~10 sec
        if (gps_outlier_count > 10) {
            ROS_ERROR_STREAM("too many outliers, assuming that the current gps value is valid.");
            // store the gps as last
            last_gps = *msg;
            last_gps_time = ros::Time::now();
            has_gps = false;

            valid_gps_samples = 0;
            gps_outlier_count = 0;
        }
    }



}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbot_positioning");

    has_gps = false;
    gps_enabled = true;
    vx = 0.0;
    has_gyro = false;
    has_ticks = false;
    gyro_offset = 0;
    gyro_offset_samples = 0;

    valid_gps_samples = 0;
    gps_outlier_count = 0;

    antenna_offset_x = antenna_offset_y = 0;

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ros::ServiceServer gps_service = n.advertiseService("xbot_positioning/set_gps_state", setGpsState);
    ros::ServiceServer pose_service = n.advertiseService("xbot_positioning/set_robot_pose", setPose);

    paramNh.param("skip_gyro_calibration", skip_gyro_calibration, false);
    paramNh.param("gyro_offset", gyro_offset, 0.0);
    paramNh.param("min_speed", min_speed, 0.01);
    paramNh.param("max_gps_accuracy", max_gps_accuracy, 0.1);
    paramNh.param("recent_gps_timeout", recent_gps_timeout, 1.5);
    paramNh.param("debug", publish_debug, false);
    paramNh.param("antenna_offset_x", antenna_offset_x, 0.0);
    paramNh.param("antenna_offset_y", antenna_offset_y, 0.0);
    // Heading from the GPS motion vector is only trustworthy above a real speed.
    // NOTE: this is deliberately far above the old min_speed (0.01 m/s), which
    // allowed heading updates from what was effectively velocity noise.
    paramNh.param("orientation_min_speed", orientation_min_speed, 0.15);

    // Timestep sanity.
    paramNh.param("max_imu_dt", max_imu_dt, 0.5);
    paramNh.param("tick_timeout", tick_timeout, 0.3);

    // ZARU (gyro bias): stationarity must be confirmed by the gyro as well as the
    // wheels, two sub-windows must agree, and the result is clamped to a band
    // around the boot calibration.
    paramNh.param("enable_zaru", enable_zaru, true);
    paramNh.param("stationary_settle_time", stationary_settle_time, 0.5);
    paramNh.param("zaru_rate", zaru_rate, 0.05);              // per accepted window
    paramNh.param("zaru_max_rate", zaru_max_rate, 0.02);      // rad/s
    paramNh.param("zaru_agree_tol", zaru_agree_tol, 0.004);   // rad/s
    paramNh.param("zaru_window_samples", zaru_window_samples, 50);
    paramNh.param("gyro_offset_band", gyro_offset_band, 0.05);// rad/s around boot

    // Wheel scale: windowed displacement ratio, two halves must agree, clamped,
    // and only committed after the RTK has stayed good through a confirmation
    // horizon (so a delayed RTK-loss flag can still veto it).
    paramNh.param("enable_wheel_cal", enable_wheel_cal, true);
    paramNh.param("wheel_cal_rate", wheel_cal_rate, 0.10);       // per accepted window (EMA)
    paramNh.param("wheel_cal_min_speed", wheel_cal_min_speed, 0.15);
    paramNh.param("wheel_cal_min_dist", wheel_cal_min_dist, 3.0);   // m of wheel travel per window
    paramNh.param("wheel_cal_agree_tol", wheel_cal_agree_tol, 0.05);
    paramNh.param("wheel_cal_straight_tol", wheel_cal_straight_tol, 0.03);  // collinearity of 3 pts
    paramNh.param("wheel_cal_max_hdg", wheel_cal_max_hdg, 0.10);    // rad heading change over window
    paramNh.param("wheel_cal_max_asym", wheel_cal_max_asym, 0.20);
    paramNh.param("wheel_cal_min_scale", wheel_cal_min_scale, 0.85);
    paramNh.param("wheel_cal_max_scale", wheel_cal_max_scale, 1.15);
    paramNh.param("cal_settle_time", cal_settle_time, 2.0);
    paramNh.param("cal_confirm_time", cal_confirm_time, 2.0);

    core.setAntennaOffset(antenna_offset_x, antenna_offset_y);

    ROS_INFO_STREAM("Antenna offset: " << antenna_offset_x << ", " << antenna_offset_y);

    if(gyro_offset != 0.0 && skip_gyro_calibration) {
        ROS_WARN_STREAM("Using gyro offset of: " << gyro_offset);
    }

    odometry_pub = paramNh.advertise<nav_msgs::Odometry>("odom_out", 50);
    xbot_absolute_pose_pub = paramNh.advertise<xbot_msgs::AbsolutePose>("xb_pose_out", 50);
    if(publish_debug) {
        dbg_expected_motion_vector = paramNh.advertise<geometry_msgs::Vector3>("debug_expected_motion_vector", 50);
        kalman_state = paramNh.advertise<xbot_positioning::KalmanState>("kalman_state", 50);
    }

    ros::Subscriber imu_sub = paramNh.subscribe("imu_in", 10, onImu);
    ros::Subscriber pose_sub = paramNh.subscribe("xb_pose_in", 10, onPose);
    ros::Subscriber wheel_tick_sub = paramNh.subscribe("wheel_ticks_in", 10, onWheelTicks);

    ros::spin();
    return 0;
}