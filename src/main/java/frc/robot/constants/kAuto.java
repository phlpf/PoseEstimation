package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class kAuto {
    private static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(kSwerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 0.6*kSwerve.MAX_VELOCITY_METERS_PER_SECOND);
    public static final ProfiledPIDController THETA_PID_CONTROLLER = new ProfiledPIDController(3.5, 0, 0, CONSTRAINTS, 0.2);
    public static final double LR_P = 0.1;
    public static final PIDController RIGHT_PID_CONTROLLER = new PIDController(LR_P, 0, 0);
    public static final PIDController LEFT_PID_CONTROLLER = new PIDController(LR_P, 0, 0);
}
