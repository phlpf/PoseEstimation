// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class AutoUtils {
    private static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1.75, 0.75);
    private static final ProfiledPIDController thetaController = new ProfiledPIDController(1.3, 0.0, 0.7, constraints, 0.2);
    private static final PIDController rightController = new PIDController(0.00386, 0, 0);
    private static final PIDController leftController = new PIDController(0.00386, 0, 0);

    public static Command generateCommand(String pathName, int maxVelocity, int maxAcceleration, DrivetrainSubsystem drives) {
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration);

        return new PPSwerveControllerCommand(
                path,
                drives::getPose,
                drives.kinematics,
                leftController,
                rightController,
                thetaController,
                drives::updateModules
        );
    }
}
