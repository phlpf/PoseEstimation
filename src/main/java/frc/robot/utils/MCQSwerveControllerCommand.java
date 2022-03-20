package frc.robot.utils;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class MCQSwerveControllerCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final PathPlannerTrajectory trajectory;
    private final Supplier<Pose2d> pose;
    private final SwerveDriveKinematics kinematics;
    private final HolonomicDriveController controller;
    private final Consumer<SwerveModuleState[]> outputModuleStates;

    /**
     * Constructs a new PPSwerveControllerCommand that when executed will follow the
     * provided
     * trajectory. This command will not return output voltages but rather raw
     * module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path-
     * this is left to the user, since it is not appropriate for paths with
     * nonstationary endstates.
     *
     * @param trajectory         The trajectory to follow.
     * @param pose               A function that supplies the robot pose - use one
     *                           of the odometry classes to
     *                           provide this.
     * @param kinematics         The kinematics for the robot drivetrain.
     * @param xController        The Trajectory Tracker PID controller for the
     *                           robot's x position.
     * @param yController        The Trajectory Tracker PID controller for the
     *                           robot's y position.
     * @param thetaController    The Trajectory Tracker PID controller for angle for
     *                           the robot.
     * @param outputModuleStates The raw output module states from the position
     *                           controllers.
     * @param requirements       The subsystems to require.
     */
    @SuppressWarnings("ParameterName")
    public MCQSwerveControllerCommand(
            PathPlannerTrajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Consumer<SwerveModuleState[]> outputModuleStates,
            Subsystem... requirements) {
        this.trajectory = trajectory;
        this.pose = pose;
        this.kinematics = kinematics;

        controller = new HolonomicDriveController(
                xController,
                yController,
                thetaController);

        this.outputModuleStates = outputModuleStates;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = timer.get();
        PathPlannerTrajectory.PathPlannerState desiredState = timer.get() < trajectory.getTotalTimeSeconds() ? (PathPlannerTrajectory.PathPlannerState) trajectory.sample(curTime) : (PathPlannerTrajectory.PathPlannerState) trajectory.sample(trajectory.getTotalTimeSeconds());

        var targetChassisSpeeds = controller.calculate(pose.get(), desiredState, desiredState.holonomicRotation);
        var targetModuleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);

        outputModuleStates.accept(targetModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        outputModuleStates.accept(kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds() + 0.5);
    }
}
