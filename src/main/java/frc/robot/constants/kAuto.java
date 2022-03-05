package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class kAuto {
    private static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(1.75, 0.75);
    public static final ProfiledPIDController THETA_PID_CONTROLLER = new ProfiledPIDController(1.3, 0.0, 0.7, CONSTRAINTS, 0.2);
    public static final PIDController RIGHT_PID_CONTROLLER = new PIDController(0.00386, 0, 0);
    public static final PIDController LEFT_PID_CONTROLLER = new PIDController(0.00386, 0, 0);
}
