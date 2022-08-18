// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.lang.reflect.Field;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.PhotonVisionWrapper;

/** Add your docs here. */
public class PoseEstimatorHelper {
    private PhotonVisionWrapper cam;
    private SwerveDrivePoseEstimator estimator;
    private double fieldWidth, fieldHeight;
    private final Field2d visionField = new Field2d();
    public PoseEstimatorHelper(SwerveDriveOdometry odometry, SwerveDriveKinematics kinematics, double fieldWidth, double fieldHeight){
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        tab.add(visionField).withSize(4, 4).withPosition(0, 0);
        cam = new PhotonVisionWrapper();
        estimator = new SwerveDrivePoseEstimator(odometry.getPoseMeters().getRotation(),
            new Pose2d(6.3, 2.8,  new Rotation2d()),
            kinematics,
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
            new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.01), // should be larger than local measurements
            0.02
        );
        
        this.fieldWidth = fieldWidth;
        this.fieldHeight = fieldHeight;
    }

    public void addTarget(double percentWidth, double percentHeight){
        cam.addVisionTargetPose(percentWidth*fieldWidth, percentHeight*fieldHeight);
    }

    public void update(Rotation2d angle, SwerveModuleState[] states){
        Pose2d estimated = cam.getVisionPosition(estimator);
        visionField.setRobotPose(estimated);
        estimator.addVisionMeasurement(estimated, cam.getTimestamp());
        estimator.update(angle, states);
    }

    public Pose2d getPosition(){
        return estimator.getEstimatedPosition();
    }

    public void reset(Pose2d pose){
        estimator.resetPosition(pose, pose.getRotation());
    }

}
