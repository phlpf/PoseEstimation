package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class kAuto {
    private static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(kSwerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, kSwerve.MAX_VELOCITY_METERS_PER_SECOND/0.6);
    public static final ProfiledPIDController THETA_PID_CONTROLLER = new ProfiledPIDController(3.5, 0, 0, CONSTRAINTS, 0.02);

    static {
        THETA_PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static final double XY_P = 1;

    public static final PIDController X_PID_CONTROLLER = new PIDController(XY_P, 0, 0);
    public static final PIDController Y_PID_CONTROLLER = new PIDController(XY_P, 0, 0);
}
