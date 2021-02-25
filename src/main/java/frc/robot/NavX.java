package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


/**
 * This is a class that initializes and tracks the NavX board to maintain current information, specifically
 * heading, for the robot. We have been having a degrees vs. radians, since all of the math trig libraries
 * use radians. we decided that staying in radians would build up minimal round-off error.
 *
 * Originally this class was written to support NavX on a conventional drive that had PID direction loops
 * concerned with matching actual heading to expected heading. Right now we are a little unclear how that
 * relates to the A05annex 2021 season swerve drive.
 */
public class NavX {

    //==================================================================================================================
    // NOTE: the NavX software expresses all the navigation angles in degrees, so we maintain angles internal
    // to this class in degrees. And do the conversions to radians when this class is queried for values.
    private AHRS m_ahrs;
    /** The heading we are trying to track with the robot (in degrees)
     */
    private double m_expectedHeading = 0.0;
    private double m_updateCt = -1;

    /** The raw heading, not corrected for the spins, read directly from the NavX, in the range
     * -180 to +180. Used for determining whether the boundary between -180 and 180 has been crossed.
     */
    private double m_headingRawLast = 0.0;

    /** The number of complete revolutions the robot has made.
     */
    private int m_headingRevs = 0;

    /** The actual heading of the robot from -infinity to infinity, so the spins are included in this
     *  heading (in degrees)
     */
    private double m_heading = 0.0;
    private boolean m_setExpectedToCurrent = false;

    // --------------------------------------------------
    // Reference values - these are the values at initialization of the NavX recording the position of the robot
    // on the field in terms of the readings of the NavX and the field heading of the robot at initialization. We
    // refer to these as the reference values as everything that happens after initialization is evaluated
    // with reference to (or as the change relative to this reference).
    // --------------------------------------------------
    /**
     * The NavX reported pitch at the time the NavX in initialized.
     */
    private double m_refPitch = 0.0;
    /**
     * The NavX reported yaw at the time the NavX in initialized.
     */
    private double m_refYaw = 0.0;
    /**
     * The NavX reported roll at the time the NavX in initialized.
     */
    private double m_refRoll = 0.0;
    /**
     * The actual field heading of the robot at the time the NavX in initialized.
     */
    private double m_refHeading = 0.0;

    /**
     *
     */
    private NavX() {
        // So, if there is no navx, there is no error - it just keeps trying to connect forever, so this
        // needs to be on a thread that can be killed if it doesn't connect in time ......
        // TODO: figure out the threading, error handling, and redundancy.
        m_ahrs = new AHRS(SPI.Port.kMXP);
        m_ahrs.reset();
        while (m_ahrs.isCalibrating()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                break;
            }
        }
        m_updateCt = m_ahrs.getUpdateCount();
        initializeHeadingAndNav();
    }

    /**
     * Sets the reference start heading and navigation reference positions to the current values. This should
     * be called immediately at the start of autonomous.
     */
    public void initializeHeadingAndNav() {
        initializeHeadingAndNav(0.0);
    }

    /**
     * Sets the reference start heading and navigation reference positions to the current values. This should
     * be called immediately at the start of autonomous.
     *
     * @param heading (double) The current field heading of the robot in radians.
     */
    public void initializeHeadingAndNav(double heading) {
        // In the past we have always initialized with the front of the robot facing down field, so the
        // heading was 0.0 at initialization. In this case we are
        m_refPitch = m_ahrs.getPitch();
        m_refYaw = m_ahrs.getYaw();
        m_refRoll = m_ahrs.getRoll();
        m_refHeading = Math.toDegrees(heading);
        m_headingRawLast = 0.0;
        m_expectedHeading = m_refHeading;
        m_headingRevs = 0;
    }

    /**
     * Change the expected heading by the specified number of radians.
     *
     * @param radians The change to the expected heading.
     */
    public void incrementExpectedHeading(double radians) {
        m_expectedHeading += Math.toDegrees(radians);

    }

    /**
     * Set the expected heading to the current heading.
     */
    public void setExpectedHeadingToCurrent() {
        m_expectedHeading = m_heading;
    }

    /**
     * Recompute the heading as reported by the NavX and adjusted to be always increasing when rotation is
     * clockwise. This heading computation was introduced by Jason Barringer to the FRC 6831 AO5 Annex code base
     * in the 2017 season to make using PID loops to control heading with the IMU easier to write, and more
     * predictable. If there is a discontinuity in the sensor output, this means there needs to be special logic
     * in the PID code to deal with the discontinuity. This handles the discontinuity in a single place where
     * the heading is computed.
     *
     * @param setExpectedToCurrent (boolean) {@code true} if the expected heading should be set to the current
     *                             heading, {@code false} otherwise. This would normally be {@code true} during
     *                             robot-relative driving when the driver is turning (the expected heading is
     *                             where the driver is turning to). This would normally be {@code false} during
     *                             field-relative driving or autonomous when the program is setting a target
     *                             heading and the robot is the expected to move along, or turn towards, the
     *                             expected heading; or when robot-relative driving without any turn.
     */
    public void recomputeHeading(boolean setExpectedToCurrent) {
        m_setExpectedToCurrent = setExpectedToCurrent;
        double heading_raw = m_ahrs.getYaw();
        // This is the logic for detecting and correcting for the IMU discontinuity at +180degrees and -180degrees.
        if (m_headingRawLast < -90.0 && heading_raw > 0.0) {
            // The previous raw IMU heading was negative and close to the discontinuity, and it is now positive. We
            // have gone through the discontinuity so we decrement the heading revolutions by 1 (we completed a
            // negative revolution). NOTE: the initial check protects from the case that the heading is near 0 and
            // goes continuously through 0, which is not the completion of a revolution.
            m_headingRevs--;
        } else if (m_headingRawLast > 90.0 && heading_raw < 0.0) {
            // The previous raw IMU heading was positive and close to the discontinuity, and it is now negative. We
            // have gone through the discontinuity so we increment the heading revolutions by 1 (we completed
            // positive revolution). NOTE: the initial check protects from the case that the heading is near 0 and
            // goes continuously through 0, which is not the completion of a revolution.
            m_headingRevs++;
        }
        m_headingRawLast = heading_raw;

        m_heading = (m_headingRevs * 360.0) + heading_raw - m_refYaw + m_refHeading;
        if (setExpectedToCurrent) {
            m_expectedHeading = m_heading;
        }
    }

    /**
     * Get the robot chassis heading in radians.
     *
     * @return The robot chassis heading in radians.
     */
    public double getHeading() {
        return Math.toRadians(m_heading);
    }

    /**
     * @return Returns the heading info, returns {@code null} if there is a problem with the NavX.
     */
    public HeadingInfo getHeadingInfo() {
        if (null == m_ahrs) {
            return null;
        }
        double updateCt = m_ahrs.getUpdateCount();
        if (updateCt <= m_updateCt) {
            // there is a problem communication with the NavX - the results we would get from NavX queries
            // are unreliable.
            return null;
        }
        return new HeadingInfo(Math.toRadians(m_heading), Math.toRadians(m_expectedHeading), m_setExpectedToCurrent);
    }

    /**
     * @return Returns the navigation info, returns {@code null} if there is a problem with the NavX.
     */
    public NavInfo getNavInfo() {
        if (null == m_ahrs) {
            return null;
        }
        // The subtraction of the ref values adjusts for the construction bias of not having the NavX perfectly
        // mounted, or there being some bias in the NavX - i.e. the ref represents the value first reported when
        // the reference position is set, see initializeHeadingAndNav().
        return new NavInfo(
                Math.toRadians(m_ahrs.getPitch() - m_refPitch), Math.toRadians(m_ahrs.getYaw() - m_refYaw),
                Math.toRadians(m_ahrs.getRoll() - m_refRoll), Math.toRadians(m_ahrs.getPitch()),
                Math.toRadians(m_ahrs.getYaw()), Math.toRadians(m_ahrs.getRoll()));
    }

    public static class HeadingInfo {
        /**
         * The current heading in radians of the robot as computed in the last call
         * to {@link NavX#initializeHeadingAndNav()}.
         */
        public final double heading;
        public final double expectedHeading;
        public final boolean isExpectedTrackingCurrent;

        HeadingInfo(double heading, double expectedHeading, boolean isExpectedTrackingCurrent) {
            this.heading = heading;
            this.expectedHeading = expectedHeading;
            this.isExpectedTrackingCurrent = isExpectedTrackingCurrent;
        }
    }

    /**
     * The data class for the 'raw' navigation info from the NavX, corrected by when the reference was last set.
     */
    public static class NavInfo {
        /**
         * The pitch (lean forward or backward) of the robot, with negative being forwards, from when the robot
         * was first initialized. Measured in radians.
         */
        public final double pitch;
        public final double rawPitch;
        /**
         * The yaw (rotation or turn) of the robot, with positive being clockwise (to the right), from when the
         * robot was first initialized. Measured in radians.
         */
        public final double yaw;
        public final double rawYaw;
        /**
         * The roll (lean sideways) of the robot, with positive being the robot falling over on it's left
         * side, from when the robot was first initialized. Measured in radians.
         */
        public final double roll;
        public final double rawRoll;

        NavInfo(double pitch, double yaw, double roll, double rawPitch, double rawYaw, double rawRoll) {
            this.pitch = pitch;
            this.yaw = yaw;
            this.roll = roll;
            this.rawPitch = rawPitch;
            this.rawYaw = rawYaw;
            this.rawRoll = rawRoll;
        }
    }

    /**
     * The Singleton instance of this NavX. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static NavX INSTANCE = new NavX();

    /**
     * Returns the Singleton instance of this NavX. This static method
     * should be used -- {@code NavX.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static NavX getInstance() {
        return INSTANCE;
    }
}
