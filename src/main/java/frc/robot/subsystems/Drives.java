// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kSwerve;
import frc.robot.utils.PigeonWrapper;

import static frc.robot.constants.kSwerve.*;

public class Drives extends SubsystemBase {
    ProfiledPIDController thetaController; 
    SwerveModuleState[] states;
    
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    
    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                    // Front Right
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Front Left
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Back Right
                    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Back Left
                    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    private final PigeonWrapper pigeon = new PigeonWrapper(kCANIDs.DRIVETRAIN_PIGEON_ID);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation());
    
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public Drives() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(0, 0),
                        kSwerve.VEL_GEAR_RATIO,
                        kCANIDs.FRONT_LEFT_DRIVE,
                        kCANIDs.FRONT_LEFT_STEER,
                        kCANIDs.FRONT_LEFT_CANCODER,
                        kSwerve.FRONT_LEFT_MODULE_STEER_OFFSET
        );

        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(2, 0),
                        kSwerve.VEL_GEAR_RATIO,
                        kCANIDs.FRONT_RIGHT_DRIVE,
                        kCANIDs.FRONT_RIGHT_STEER,
                        kCANIDs.FRONT_RIGHT_CANCODER,
                        kSwerve.FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(4, 0),
                        kSwerve.VEL_GEAR_RATIO,
                        kCANIDs.REAR_LEFT_DRIVE,
                        kCANIDs.REAR_LEFT_STEER,
                        kCANIDs.REAR_LEFT_CANCODER,
                        kSwerve.REAR_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(6, 0),
                        kSwerve.VEL_GEAR_RATIO,
                        kCANIDs.REAR_RIGHT_DRIVE,
                        kCANIDs.REAR_RIGHT_STEER,
                        kCANIDs.REAR_RIGHT_CANCODER,
                        kSwerve.REAR_RIGHT_MODULE_STEER_OFFSET
        );
        
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        pigeon.reset();
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    public Rotation2d getDesiredRotation(){
        return new Rotation2d(1, 0);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void updateModules(SwerveModuleState[] newStates){
        frontRightModule.set(newStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, newStates[0].angle.getRadians());
        frontLeftModule.set(newStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, newStates[1].angle.getRadians());
        backRightModule.set(newStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, newStates[2].angle.getRadians());
        backLeftModule.set(newStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, newStates[3].angle.getRadians());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        states = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        updateModules(states);

        odometry.update(getGyroscopeRotation(), states);
    
        //TODO: add current for all module motors
    }
}
