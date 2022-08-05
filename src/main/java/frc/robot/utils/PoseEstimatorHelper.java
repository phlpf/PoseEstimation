// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.PhotonVisionWrapper;

/** Add your docs here. */
public class PoseEstimatorHelper {
    private SwerveDriveOdometry odometry;
    private SwerveDriveKinematics kinematics;
    private PhotonVisionWrapper cam;
    private SwerveDrivePoseEstimator estimator;
    public PoseEstimatorHelper(SwerveDriveOdometry odometry, SwerveDriveKinematics kinematics){
        cam = new PhotonVisionWrapper();
        this.odometry = odometry;
        this.kinematics = kinematics;
        estimator = new SwerveDrivePoseEstimator(odometry.getPoseMeters().getRotation(),
            odometry.getPoseMeters(),
            kinematics, 
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), 
            new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01), 
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
        );
    }

    public void update(){
        Pose2d estimated = cam.getVisionPosition(odometry);
        if(estimated != null){
            estimator.addVisionMeasurement(estimated, 0.02);
        }
    }

    public Pose2d getPosition(){
        return estimator.getEstimatedPosition();
    }

    public void reset(Pose2d pose){
        estimator.resetPosition(pose, pose.getRotation());
    }
    
}
