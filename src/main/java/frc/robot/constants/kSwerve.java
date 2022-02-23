package frc.robot.constants;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

public class kSwerve {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     *
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     */
    public static double MAX_VOLTAGE = 12.0;

    /**
     * The maximum velocity of the robot in meters per second.
     *
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * SdsModuleConfigurations.MK4_L1.getDriveReduction()
            * SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;

    public static double MAX_ACCELERATION = 1.6; //1.7;

    public static void recalculate(Mk4SwerveModuleHelper.GearRatio ratio){
        ModuleConfiguration type = ratio.getConfiguration();
        MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                type.getDriveReduction() * type.getWheelDiameter() * Math.PI;

        MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
        SWERVE_CORRECTION_SPEED = 0.1 * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 1.0; // FIXME Measure and set trackwidth

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 1.0; // FIXME Measure and set wheelbase

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                    Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(221.0);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(8.0);
    public static final double REAR_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(54.0);
    public static final double REAR_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(322.0);

    public static final double SWERVE_ALLOWED_OFFSET = 1.0;

    public static double SWERVE_CORRECTION_SPEED = 0.1 * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
}
