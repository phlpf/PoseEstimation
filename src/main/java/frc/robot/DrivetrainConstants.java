package frc.robot;
import static frc.robot.Constants.*;

public class DrivetrainConstants {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
    
    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    //    The formula for calculating the theoretical maximum velocity is:
    //     <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    //    By default this value is setup for a Mk3 standard module using Falcon500s to drive.
    //    An example of this constant for a Mk4 L2 module with NEOs to drive is:
    //     5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
             (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0)*
             0.1016* Math.PI;
    public static final double MAX_ACCELERATION = 1.6; //1.7;
                    
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                    Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

}
