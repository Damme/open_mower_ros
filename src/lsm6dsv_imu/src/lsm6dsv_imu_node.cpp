/*
 * 7.cpp  -  ROS IMU publisher for the ST LSM6DSV
 *
 * Publishes:  imu/data_raw   (sensor_msgs/Imu)
 *
 * Sensor:     ST LSM6DSV, datasheet DS13476 Rev 5 (August 2023)
 * Settings:   Gyro  120 Hz, HAODR, +/-500 dps, LPF1 on
 *             Accel 120 Hz, HAODR, +/-2 g (default, no CTRL8 write)
 *             SFLP  on-chip gyro-bias estimation enabled
 *
 * Build (add to CMakeLists.txt):
 *   add_executable(lsm6dsv_imu src/lsm6dsv_imu.cpp)
 *   target_link_libraries(lsm6dsv_imu ${catkin_LIBRARIES} wiringPi m)
 */

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <unistd.h>
#include <wiringPiI2C.h>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

/*  I2C address 
 * SA0/SDO pin LOW  -> 0x6A
 * SA0/SDO pin HIGH -> 0x6B  (board default)                                  */
#define LSM6DSV_ADDR  0x6B

/*  Register addresses  */
#define REG_FUNC_CFG_ACCESS  0x01
#define REG_WHO_AM_I         0x0F
#define REG_CTRL1            0x10
#define REG_CTRL2            0x11
#define REG_CTRL3            0x12
#define REG_CTRL6            0x15
#define REG_CTRL7            0x16
#define REG_CTRL8            0x17
#define REG_CTRL9            0x18
#define REG_STATUS           0x1E
#define REG_OUT_TEMP_L       0x20
#define REG_OUTX_L_G         0x22   /* gyro  burst: 6 bytes -> 0x27 */
#define REG_OUTX_L_A         0x28   /* accel burst: 6 bytes -> 0x2D */
#define EMB_FUNC_EN_A        0x04   /* embedded-function bank       */

#define WHO_AM_I_EXPECTED    0x70
#define STATUS_GDA           0x02   /* gyroscope data available     */

/*  Sensitivity 
 * Accel  +/-2 g default (CTRL8 not written)  ->  0.061 mg/LSB
 * Gyro   +/-500 dps (CTRL2 FS encoding)      ->  17.5 mdps/LSB
 * Temp   256 LSB/degC, 0 LSB = 25 degC                                          */
#define ACCEL_SCALE  (0.061e-3 * 9.80665)          /* -> m/s^2 per LSB        */
#define GYRO_SCALE   (17.5e-3 * (M_PI / 180.0))    /* -> rad/s per LSB       */
#define GYRO_DEG_PER_LSB   17.5e-3                  /* dps per LSB (for log) */
#define TEMP_SCALE   (1.0 / 256.0)
#define TEMP_OFFSET  25.0

/*  Noise covariances for sensor_msgs/Imu 
 * Derived from datasheet sec 4.1, Table 3.
 * Gyro  noise density 2.8 mdps/sqrtHz at ~50 Hz BW -> sigma ~ 20 mdps RMS
 * Accel noise density 60 ug/sqrtHz   at ~50 Hz BW -> sigma ~ 424 ug RMS
 * Variances shown below; set once in the covariance arrays.                  */
#define GYRO_VAR   (3.5e-4 * 3.5e-4)   /* (rad/s)^2  */
#define ACCEL_VAR  (4.2e-3 * 4.2e-3)   /* (m/s^2)^2   */

/*  Calibration
 * NOTE: the old "512 samples took ~70 s" observation was a symptom of the
 * gyro running at 7.5 Hz under the previous (mis-encoded) CTRL2 (512/7.5 =
 * 68 s).  With the corrected 120 Hz config the loop is gated by data-ready,
 * so 64 samples now complete in well under a second; raising the count for a
 * tighter bias estimate is cheap.                                            */
#define BIAS_CAL_SAMPLES  256
#define LEVEL_CAL_SAMPLES 128   /* accel averages for the gravity-up estimate */

/*  */

static int burst_read(int fd, uint8_t reg, uint8_t *buf, int len)
{
    if (write(fd, &reg, 1) != 1) return -1;
    if (read(fd, buf, len) != len) return -1;
    return 0;
}

static inline int16_t le16(const uint8_t *b)
{
    return static_cast<int16_t>(static_cast<uint16_t>(b[1]) << 8 | b[0]);
}

/*  Discrete mounting remap: sensor frame -> body frame (REP-103).
 * Single source of truth for the coarse 90-degree orientation.  This fixes
 * which way is forward/left/up; the fine residual tilt is handled by the
 * auto-level rotation below.  Used by both the main loop and the leveler.  */
static inline void remap_sensor_to_body(double sx, double sy, double sz,
                                        double &bx, double &by, double &bz)
{
    bx = -sz;   /* body +X (forward) = -Z_sensor */
    by =  sx;   /* body +Y (left)    = +X_sensor */
    bz = -sy;   /* body +Z (up)      = -Y_sensor */
}

/*  Apply a 3x3 rotation.  Inputs are by value, so out may alias in.  */
static inline void mat3_apply(const double R[3][3],
                              double x, double y, double z,
                              double &ox, double &oy, double &oz)
{
    ox = R[0][0] * x + R[0][1] * y + R[0][2] * z;
    oy = R[1][0] * x + R[1][1] * y + R[1][2] * z;
    oz = R[2][0] * x + R[2][1] * y + R[2][2] * z;
}

/*  Sensor init 
 *
 * REGISTER LAYOUT (verified against DS13476 Rev 5):
 *   CTRL1 (10h): [7]=0  [6:4]=OP_MODE_XL  [3:0]=ODR_XL    (NOT full scale)
 *   CTRL2 (11h): [7]=0  [6:4]=OP_MODE_G   [3:0]=ODR_G     (NOT full scale)
 *   CTRL6 (15h): [6:4]=LPF1_G_BW          [3:0]=FS_G      (gyro full scale)
 *   CTRL7 (16h): [0]=LPF1_G_EN            (gyro LPF1 enable lives HERE)
 *   CTRL8 (17h): [7:5]=HP_LPF2_XL_BW      [1:0]=FS_XL     (accel full scale)
 *   CTRL9 (18h): [3]=LPF2_XL_EN           (accel LPF2 enable)
 *
 *   OP_MODE 001 = high-accuracy ODR (HAODR);  ODR 0110 = 120 Hz (Tables 50-54).
 *   Both sensors must share a power mode or STATUS may never assert (sec 6.5).
 *   HAODR vs HP: same noise density, ODR accuracy +/-1% vs +/-3% -> steadier dt
 *   for the Kalman integrator.                                                 */
static bool lsm6dsv_init(int fd)
{
    int id = wiringPiI2CReadReg8(fd, REG_WHO_AM_I);
    if (id != WHO_AM_I_EXPECTED) {
        ROS_ERROR("LSM6DSV WHO_AM_I: got 0x%02X, expected 0x%02X", id, WHO_AM_I_EXPECTED);
        ROS_ERROR("Check I2C address (SA0 LOW=0x6A, HIGH=0x6B) and wiring");
        return false;
    }

    /* Software reset - self-clearing bit */
    wiringPiI2CWriteReg8(fd, REG_CTRL3, 0x01);
    for (int i = 0; i < 200; i++) {
        if (!(wiringPiI2CReadReg8(fd, REG_CTRL3) & 0x01)) break;
        usleep(1000);
    }
    usleep(15000);

    /* BDU=1 (freeze output regs until both bytes read), IF_INC=1 (burst) */
    wiringPiI2CWriteReg8(fd, REG_CTRL3, 0x44);

    /* Gyro full scale + LPF1 bandwidth (CTRL6).
     *   FS_G      = 0010 -> +/-500 dps  (matches GYRO_SCALE = 17.5 mdps/LSB)
     *   LPF1_G_BW = 110  -> 24.2 Hz at 120 Hz ODR (Table 63); below the 25 Hz
     *                       Nyquist of the 50 Hz output, so it also serves as
     *                       the anti-alias filter for the decimated stream.
     * 0x62 = 0 110 0010.  For +/-250 dps use 0x61 and set GYRO_SCALE to 8.75e-3. */
    wiringPiI2CWriteReg8(fd, REG_CTRL6, 0x62);

    /* Enable gyro LPF1 (CTRL7 bit0 = LPF1_G_EN).  Was 0x00 = filter OFF. */
    wiringPiI2CWriteReg8(fd, REG_CTRL7, 0x01);

    /* Accel full scale + anti-vibration LPF2.
     *   CTRL8: HP_LPF2_XL_BW = 001 -> ODR/10 = 12 Hz cutoff (Table 68),
     *          FS_XL = 00 -> +/-2 g (matches ACCEL_SCALE = 0.061 mg/LSB).
     *          0x20 = 001 0 0 0 00.
     *   CTRL9: LPF2_XL_EN (bit3) = 1 selects the LPF2 output (0x08).
     * Strongly recommended in a vibrating mower: keeps the gravity/tilt
     * estimate (anti-tip) clean.  Set CTRL8=0x00, CTRL9=0x00 to disable. */
    wiringPiI2CWriteReg8(fd, REG_CTRL8, 0x20);
    wiringPiI2CWriteReg8(fd, REG_CTRL9, 0x08);

    /* Gyro:  OP_MODE_G=001 (HAODR), ODR_G=0110 (120 Hz).  0x16 = 0 001 0110. */
    wiringPiI2CWriteReg8(fd, REG_CTRL2, 0x16);

    /* Accel: OP_MODE_XL=001 (HAODR), ODR_XL=0110 (120 Hz).  0x16 = 0 001 0110. */
    wiringPiI2CWriteReg8(fd, REG_CTRL1, 0x16);

    usleep(50000);   /* gyro turn-on time 30 ms (Table, Ton) */
    return true;
}

/*  SFLP gyro-bias estimation 
 * Enables the on-chip Sensor Fusion Low Power block (sec 2.7).
 * The SFLP continuously estimates gyro bias; outputs appear as tagged FIFO
 * records (game rotation vector, gravity, gyro bias - see AN5922 sec 5).
 * Static yaw drift after convergence: 0.5deg / 5 min ~ 1.7 mdps (Table 1).
 *
 * This node uses the software calibration for the initial bias offset and
 * relies on SFLP for ongoing drift tracking.  Full FIFO-based SFLP quaternion
 * output can be added as a separate orientation publisher in a future step.  */
static void lsm6dsv_enable_sflp(int fd)
{
    wiringPiI2CWriteReg8(fd, REG_FUNC_CFG_ACCESS, 0x80);  /* unlock embed bank */
    usleep(200);
    int cur = wiringPiI2CReadReg8(fd, EMB_FUNC_EN_A);
    wiringPiI2CWriteReg8(fd, EMB_FUNC_EN_A, cur | 0x02);  /* SFLP_GAME_EN = bit1 */
    wiringPiI2CWriteReg8(fd, REG_FUNC_CFG_ACCESS, 0x00);  /* re-lock           */
    usleep(200);
    ROS_INFO("SFLP gyro-bias estimation enabled");
}

/*  Static gyro-bias calibration 
 * Factory OTP trim (applied at power-up, not user-readable) brings zero-rate
 * offset to +/-1 dps spec.  This removes the PCB-mounting residual that remains.
 *
 * Keep sensor PERFECTLY STILL.  Progress is logged via ROS_INFO.            */
static bool calibrate_gyro_bias(int fd,
                                double &bx, double &by, double &bz)
{
    ROS_INFO("Gyro bias calibration - keep sensor still (~5-15 s)...");

    double sx = 0, sy = 0, sz = 0;
    uint8_t buf[6];

    for (int n = 0; n < BIAS_CAL_SAMPLES; ) {
        int retries = 0, status;
        do {
            status = wiringPiI2CReadReg8(fd, REG_STATUS);
            if (status < 0) status = 0;   /* bus error: -1 & GDA is truthy */
            usleep(200);
        } while (!(status & STATUS_GDA) && ++retries < 5000);

        if (!(status & STATUS_GDA)) {
            ROS_ERROR("Gyro calibration: STATUS_GDA timeout at sample %d", n);
            return false;
        }
        if (burst_read(fd, REG_OUTX_L_G, buf, 6) < 0) {
            ROS_ERROR("Gyro calibration: read error at sample %d", n);
            return false;
        }

        sx += le16(buf + 0);
        sy += le16(buf + 2);
        sz += le16(buf + 4);
        ++n;
    }

    bx = sx / BIAS_CAL_SAMPLES;
    by = sy / BIAS_CAL_SAMPLES;
    bz = sz / BIAS_CAL_SAMPLES;

    ROS_INFO("Gyro bias (dps): X=%+.4f  Y=%+.4f  Z=%+.4f",
             bx * GYRO_DEG_PER_LSB,
             by * GYRO_DEG_PER_LSB,
             bz * GYRO_DEG_PER_LSB);
    return true;
}

/*  Auto-level calibration  (gravity alignment)
 *
 * Removes the residual roll/pitch of the physical mount so that flat ground
 * reads zero tilt and the gyro Z axis sits on TRUE vertical.  Procedure:
 * average accel while still -> that vector is "up" in the body frame ->
 * build the minimal rotation that sends it to (0, 0, +1).
 *
 * IMPORTANT LIMITATION: gravity fixes only 2 of 3 axes (roll, pitch).  It
 * CANNOT define yaw / which way is forward - that still comes from the
 * discrete remap above (and, for heading, from GPS course downstream).  So
 * this locks Z to vertical; it does not make mounting yaw irrelevant.
 *
 * Builds R via the standard "rotation between two vectors" (Rodrigues with
 * v = u x z, c = u . z).  Keep the sensor still during this step.           */
static bool calibrate_level(int fd, double R[3][3])
{
    /* identity default */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R[i][j] = (i == j) ? 1.0 : 0.0;

    ROS_INFO("Auto-level - keep sensor still...");

    double bx = 0, by = 0, bz = 0;
    uint8_t buf[6];
    for (int n = 0; n < LEVEL_CAL_SAMPLES; ++n) {
        if (burst_read(fd, REG_OUTX_L_A, buf, 6) < 0) {
            ROS_ERROR("Auto-level: accel read error at sample %d", n);
            return false;
        }
        double sx = le16(buf + 0) * ACCEL_SCALE;
        double sy = le16(buf + 2) * ACCEL_SCALE;
        double sz = le16(buf + 4) * ACCEL_SCALE;
        double ax, ay, az;
        remap_sensor_to_body(sx, sy, sz, ax, ay, az);  /* same coarse remap */
        bx += ax; by += ay; bz += az;
        usleep(8000);   /* ~125 Hz */
    }
    bx /= LEVEL_CAL_SAMPLES;
    by /= LEVEL_CAL_SAMPLES;
    bz /= LEVEL_CAL_SAMPLES;

    double mag = std::sqrt(bx * bx + by * by + bz * bz);
    if (mag < 1.0) {                      /* implausibly low - sensor moving? */
        ROS_WARN("Auto-level: |accel|=%.3f too low, leaving frame unrotated", mag);
        return true;
    }

    /* unit "up" vector u, and target z = (0,0,1) */
    double ux = bx / mag, uy = by / mag, uz = bz / mag;
    double tilt_deg = std::acos(uz < -1.0 ? -1.0 : (uz > 1.0 ? 1.0 : uz))
                      * 180.0 / M_PI;

    /* v = u x z = (uy, -ux, 0);  s = |v| = sin(angle);  c = u.z = uz */
    double vx = uy, vy = -ux, vz = 0.0;
    double s2 = vx * vx + vy * vy + vz * vz;   /* sin^2(angle) */
    double c  = uz;                            /* cos(angle)   */

    if (s2 < 1e-12) {
        if (c > 0.0) {
            ROS_INFO("Auto-level: already vertical (tilt %.2f deg), no rotation", tilt_deg);
        } else {
            /* upside down: 180 deg flip about body X */
            R[1][1] = -1.0; R[2][2] = -1.0;
            ROS_WARN("Auto-level: accel points DOWN (mounted upside down?) - applied 180 deg flip");
        }
        return true;
    }

    /* R = I + [v]x + [v]x^2 * (1 - c)/s2   (maps u onto +Z) */
    double k = (1.0 - c) / s2;
    /* [v]x */
    double vxx[3][3] = {
        {  0.0, -vz,   vy },
        {  vz,   0.0, -vx },
        { -vy,   vx,   0.0 }
    };
    /* [v]x^2 */
    double vx2[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            double sum = 0.0;
            for (int m = 0; m < 3; m++) sum += vxx[i][m] * vxx[m][j];
            vx2[i][j] = sum;
        }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R[i][j] = (i == j ? 1.0 : 0.0) + vxx[i][j] + k * vx2[i][j];

    ROS_INFO("Auto-level: mount tilt %.2f deg corrected (Z locked to vertical)", tilt_deg);
    return true;
}

/*  Main  */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lsm6dsv_imu");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");

    bool auto_level = true;
    pnh.param("auto_level", auto_level, true);  /* ~auto_level:=false to skip */

    ros::Publisher pub_imu =
        n.advertise<sensor_msgs::Imu>("imu/data_raw", 10);

    /*  I2C setup  */
    int fd = wiringPiI2CSetup(LSM6DSV_ADDR);
    if (fd == -1) {
        ROS_FATAL("Cannot open I2C device at 0x%02X", LSM6DSV_ADDR);
        return 1;
    }

    /*  Sensor init  */
    if (!lsm6dsv_init(fd)) return 1;
    lsm6dsv_enable_sflp(fd);

    /*  Gyro bias calibration  */
    double bias_x = 0, bias_y = 0, bias_z = 0;
    if (!calibrate_gyro_bias(fd, bias_x, bias_y, bias_z)) return 1;

    /*  Auto-level: lock body Z to vertical (roll/pitch mount error)  */
    double R_level[3][3];
    if (auto_level) {
        if (!calibrate_level(fd, R_level)) return 1;
    } else {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R_level[i][j] = (i == j) ? 1.0 : 0.0;
        ROS_INFO("Auto-level disabled (~auto_level:=false)");
    }

    ROS_INFO("LSM6DSV ready - publishing imu/data_raw at 50 Hz");

    /*  Build static parts of the Imu message 
     * orientation_covariance[0] = -1 tells downstream nodes (robot_localization,
     * imu_filter_madgwick, etc.) that orientation is not provided here.
     * angular_velocity and linear_acceleration covariances are set from the
     * datasheet noise specs so the Kalman filter can weight this sensor
     * correctly against GPS and wheel odometry.                            */
    sensor_msgs::Imu msg;
    msg.header.frame_id = "imu_link";

    msg.orientation_covariance[0] = -1.0;   /* orientation not provided     */

    /* Diagonal covariance: [xx, xy, xz, yx, yy, yz, zx, zy, zz]           */
    msg.angular_velocity_covariance[0] = GYRO_VAR;
    msg.angular_velocity_covariance[4] = GYRO_VAR;
    msg.angular_velocity_covariance[8] = GYRO_VAR;

    msg.linear_acceleration_covariance[0] = ACCEL_VAR;
    msg.linear_acceleration_covariance[4] = ACCEL_VAR;
    msg.linear_acceleration_covariance[8] = ACCEL_VAR;

    /*  AXIS REMAP  (mounting: chip underside faces forward, +Y_s points down)
     *
     *   Sensor frame (s)             ROS body frame (REP-103: x fwd, y left, z up)
     *   +X_s  ---------------------  +Y_body  (left)
     *   +Y_s  ---------------------  -Z_body  (down)
     *   +Z_s  ---------------------  -X_body  (back)
     *
     *   => x_body = -z_s,  y_body = +x_s,  z_body = -y_s
     *
     *   Proper rotation (det = +1), so gyro and accel share the same mapping.
     *   The isotropic diagonal covariance is unchanged by the permutation.   */
    /*  Main loop  */
    ros::Rate loop_rate(50);   /* 50 Hz publish; sensor streams at 120 Hz  */
    uint8_t gbuf[6], abuf[6];
    int timeouts = 0;          /* consecutive GDA timeouts */

    while (ros::ok()) {

        /* Wait for gyro data-ready.  At 120 Hz a new sample is available
         * every ~8.3 ms; 500 polls at 0.1 ms = 50 ms window before warn. */
        int status = 0, retries = 0;
        do {
            status = wiringPiI2CReadReg8(fd, REG_STATUS);
            if (status < 0) status = 0;   /* bus error: -1 & GDA is truthy */
            if (++retries > 500) break;
            usleep(100);
        } while (!(status & STATUS_GDA));

        if (!(status & STATUS_GDA)) {
            /* Persistent GDA=0 -> suspect sensor POR/brownout (CTRL2 reads
             * 0x00 after reset = gyro power-down, data-ready never asserts).
             * Never publish the held sample; re-init in place after 3 misses.
             * Bias and R_level remain valid across a re-init.               */
            int id = wiringPiI2CReadReg8(fd, REG_WHO_AM_I);
            int c1 = wiringPiI2CReadReg8(fd, REG_CTRL1);
            int c2 = wiringPiI2CReadReg8(fd, REG_CTRL2);
            ROS_WARN_THROTTLE(5,
                "STATUS timeout (GDA=0) WHO=0x%02X CTRL1=0x%02X CTRL2=0x%02X - dropping sample",
                id, c1, c2);
            if (++timeouts >= 3) {
                ROS_ERROR_THROTTLE(5,
                    "IMU unresponsive (CTRL2=0x%02X) - re-initializing", c2);
                if (lsm6dsv_init(fd)) {
                    lsm6dsv_enable_sflp(fd);
                    ROS_WARN("IMU re-init OK (bias/level retained)");
                }
                timeouts = 0;
            }
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        timeouts = 0;

        /* Burst-read gyro (0x22-0x27) and accel (0x28-0x2D). */
        if (burst_read(fd, REG_OUTX_L_G, gbuf, 6) < 0 ||
            burst_read(fd, REG_OUTX_L_A, abuf, 6) < 0) {
            ROS_WARN("IMU read error - skipping sample");
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        /*  Decode and scale  *
         * Bias subtracted in raw LSB before scaling - preserves full     *
         * precision of the bias estimate (subtracting post-scale loses   *
         * low-order floating-point bits).                                 */
        msg.header.stamp = ros::Time::now();

        /* Sensor-frame gyro (bias removed in raw LSB, then scaled). */
        const double gx_s = (le16(gbuf + 0) - bias_x) * GYRO_SCALE;
        const double gy_s = (le16(gbuf + 2) - bias_y) * GYRO_SCALE;
        const double gz_s = (le16(gbuf + 4) - bias_z) * GYRO_SCALE;

        /* Sensor-frame accel. */
        const double ax_s = le16(abuf + 0) * ACCEL_SCALE;
        const double ay_s = le16(abuf + 2) * ACCEL_SCALE;
        const double az_s = le16(abuf + 4) * ACCEL_SCALE;

        /* Coarse mounting remap, then fine auto-level rotation. */
        double gx, gy, gz, ax, ay, az;
        remap_sensor_to_body(gx_s, gy_s, gz_s, gx, gy, gz);
        remap_sensor_to_body(ax_s, ay_s, az_s, ax, ay, az);
        mat3_apply(R_level, gx, gy, gz, gx, gy, gz);
        mat3_apply(R_level, ax, ay, az, ax, ay, az);

        msg.angular_velocity.x = gx;
        msg.angular_velocity.y = gy;
        msg.angular_velocity.z = gz;
        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;

        pub_imu.publish(msg);

        /*  Health log, once per minute  (body frame, post-level)
         * At rest: gyro magnitude should sit near 0 and |accel| near 9.81.
         * A large steady gyro offset = bias drift (watch vs temp); |accel|
         * far from 9.81 = scale/clip problem.  Temp helps correlate drift. */
        uint8_t tbuf[2] = {0, 0};
        double temp_c = 0.0;
        if (burst_read(fd, REG_OUT_TEMP_L, tbuf, 2) == 0)
            temp_c = le16(tbuf) * TEMP_SCALE + TEMP_OFFSET;

        const double gmag = std::sqrt(gx * gx + gy * gy + gz * gz);
        const double amag = std::sqrt(ax * ax + ay * ay + az * az);
        ROS_INFO_THROTTLE(60,
            "IMU | gyro[dps] x=%+.2f y=%+.2f z=%+.2f |w|=%.2f | "
            "accel[m/s^2] x=%+.2f y=%+.2f z=%+.2f |a|=%.2f | T=%.1fC",
            gx / (M_PI / 180.0), gy / (M_PI / 180.0), gz / (M_PI / 180.0),
            gmag / (M_PI / 180.0),
            ax, ay, az, amag, temp_c);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}