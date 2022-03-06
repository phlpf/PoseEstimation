// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.kAuto;
import frc.robot.subsystems.Drives;

/** Add your docs here. */
public class AutoUtil {
    public enum Routine {
        FOUR_BALL,
        THREE_BALL,
        TWO_BALL,
        POTATO
    }

    public static Command generateCommand(String pathName, double maxVelocity, double maxAcceleration, Drives drives) {
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration);

        return new PPSwerveControllerCommand(
                path,
                drives::getPose,
                drives.kinematics,
                kAuto.LEFT_PID_CONTROLLER,
                kAuto.RIGHT_PID_CONTROLLER,
                kAuto.THETA_PID_CONTROLLER,
                drives::updateModules
        );
    }
}
