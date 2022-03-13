package frc.robot.utils;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class PPSwerveControllerCommandWrapper extends PPSwerveControllerCommand {
    public Consumer<SwerveModuleState[]> outputModuleStates;
    public SwerveDriveKinematics kinematics;

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
    public PPSwerveControllerCommandWrapper(PathPlannerTrajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics, PIDController xController, PIDController yController, ProfiledPIDController thetaController, Consumer<SwerveModuleState[]> outputModuleStates, Subsystem... requirements) {
        super(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates, requirements);
        this.outputModuleStates = outputModuleStates;
        this.kinematics = kinematics;
    }

    @Override
    public void end(boolean interrupted) {
        outputModuleStates.accept(kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
        super.end(interrupted);
    }
}
