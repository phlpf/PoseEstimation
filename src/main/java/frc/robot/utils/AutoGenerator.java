// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class AutoGenerator {
    private List<Pose2d> waypoints;
    private Pose2d beginPose;
    private Pose2d endPose;
    private DrivetrainSubsystem drives;
    private TrajectoryConfig config;
    private SwerveDriveKinematics constraint;
    private ProfiledPIDController thetaController;
    private PIDController rightController, leftController;      
    private Trajectory trajectory;

    public AutoGenerator(DrivetrainSubsystem drives, Pose2d beginPose, Pose2d endPose){
        this.beginPose = beginPose;
        this.endPose = endPose;
        config = new TrajectoryConfig(DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND, DrivetrainConstants.MAX_ACCELERATION);
        TrapezoidProfile.Constraints m_constraints =
                new TrapezoidProfile.Constraints(1.75, 0.75);
        this.thetaController =
                new ProfiledPIDController(1.3, 0.0, 0.7, m_constraints, 0.2);
        this.rightController = new PIDController(0.00386, 0, 0);
        this.leftController = new PIDController(0.00386, 0, 0); // TODO: Add constants
        waypoints.add(beginPose);
        this.drives = drives;
    }
    public void addWaypoint(Pose2d waypoint){
        waypoints.add(waypoint);
    }
    public void changeConfig(TrajectoryConfig config){
        this.config = config;
    }

    public SwerveControllerCommand generateCommand(){
        waypoints.add(endPose);
        trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);

        return new SwerveControllerCommand(trajectory, 
        drives.odometry::getPoseMeters, 
        drives.m_kinematics,
        leftController,
        rightController,
        thetaController,
        drives::getDesiredRotation,
        drives::updateModules);
    }
}
