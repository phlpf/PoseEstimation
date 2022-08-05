// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;

/** Add your docs here. */
public class PhotonVisionWrapper {
    PhotonCamera cam;
    ArrayList<Translation2d> targets;
    
    public PhotonVisionWrapper(){
        cam = new PhotonCamera("gloworm");
    } 
    public double getYaw(){
        var results = cam.getLatestResult();
        return (results.hasTargets())?results.getBestTarget().getYaw():0;
    }
    public boolean hasTargets(){
        return cam.getLatestResult().hasTargets();
    }

    public Pose2d getVisionPosition(SwerveDriveOdometry odometry){
        Pose2d robotPosition = odometry.getPoseMeters();
        if(hasTargets()){
            var result = cam.getLatestResult().getBestTarget();
            Translation2d bestPose, absPose;
            bestPose = robotPosition.getTranslation();
            double bestDistance = 10000; // stupid number, fix later
            Transform2d visionPose;
            Rotation2d heading = odometry.getPoseMeters().getRotation();
            visionPose = result.getCameraToTarget();
            double distance = visionPose.getTranslation().getDistance(new Translation2d());
            double x = distance*heading.getCos();
            double y = distance*heading.getSin();
            for(Translation2d target : targets){
                absPose =  new Translation2d(target.getX() - x, target.getY() - y);
                if(absPose.getDistance(robotPosition.getTranslation()) < bestDistance){
                    bestDistance = absPose.getDistance(robotPosition.getTranslation());
                    bestPose = absPose;
                }
            }
            return new Pose2d(bestPose, heading);
        }
        return robotPosition;
    }
}
