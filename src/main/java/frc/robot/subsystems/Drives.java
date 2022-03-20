// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kSwerve;

import static frc.robot.constants.kSwerve.*;

public class Drives extends SubsystemBase {
    private boolean runDrive = true;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final WPI_Pigeon2 pigeonTwo = new WPI_Pigeon2(kCANIDs.DRIVETRAIN_PIGEON_ID, kSwerve.CANIVORE_NAME);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                    // Front Right
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Front Left
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Back Right
                    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Back Left
                    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );
    private SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

    private final SwerveDriveOdometry odometry;
    private final Field2d field = new Field2d();

    public Drives() {
        pigeonTwo.configFactoryDefault();
        pigeonTwo.reset();
        
        odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

        SmartDashboard.putData("Field", field);

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        // TODO: Set current limits
        // ModuleConfiguration moduleConf = new ModuleConfiguration();
        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(0, 0),
                        kSwerve.VEL_GEAR_RATIO,
                        kCANIDs.FRONT_LEFT_DRIVE,
                        kSwerve.CANIVORE_NAME,
                        kCANIDs.FRONT_LEFT_STEER,
                        kSwerve.CANIVORE_NAME,
                        kCANIDs.FRONT_LEFT_CANCODER,
                        kSwerve.CANIVORE_NAME,
                        kSwerve.FRONT_LEFT_MODULE_STEER_OFFSET
        );

        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(2, 0),
                        kSwerve.VEL_GEAR_RATIO,
                        kCANIDs.FRONT_RIGHT_DRIVE,
                        kSwerve.CANIVORE_NAME,
                        kCANIDs.FRONT_RIGHT_STEER,
                        kSwerve.CANIVORE_NAME,
                        kCANIDs.FRONT_RIGHT_CANCODER,
                        kSwerve.CANIVORE_NAME,
                        kSwerve.FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(4, 0),
                        kSwerve.VEL_GEAR_RATIO,
                        kCANIDs.REAR_LEFT_DRIVE,
                        kSwerve.CANIVORE_NAME,
                        kCANIDs.REAR_LEFT_STEER,
                        kSwerve.CANIVORE_NAME,
                        kCANIDs.REAR_LEFT_CANCODER,
                        kSwerve.CANIVORE_NAME,
                        kSwerve.REAR_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(6, 0),
                        kSwerve.VEL_GEAR_RATIO,
                        kCANIDs.REAR_RIGHT_DRIVE,
                        kSwerve.CANIVORE_NAME,
                        kCANIDs.REAR_RIGHT_STEER,
                        kSwerve.CANIVORE_NAME,
                        kCANIDs.REAR_RIGHT_CANCODER,
                        kSwerve.CANIVORE_NAME,
                        kSwerve.REAR_RIGHT_MODULE_STEER_OFFSET
        );
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        pigeonTwo.reset();
        odometry.resetPosition(new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0)), getGyroscopeRotation());
    }

    public Rotation2d getGyroscopeRotation() {
        return pigeonTwo.getRotation2d();
    }

    public void setOdometryRotation(Pose2d pose) {
        pigeonTwo.setYaw(pose.getRotation().getDegrees());
        odometry.resetPosition(pose, getGyroscopeRotation());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void updateModules(SwerveModuleState[] newStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, MAX_VELOCITY_METERS_PER_SECOND);
        states = newStates;

        frontRightModule.set(newStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, newStates[0].angle.getRadians());
        frontLeftModule.set(newStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, newStates[1].angle.getRadians());
        backRightModule.set(newStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, newStates[2].angle.getRadians());
        backLeftModule.set(newStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, newStates[3].angle.getRadians());
    }

    public void setRunDrives(boolean runDrives){
        this.runDrive = runDrives;
    }

    public Field2d getField() {
        return field;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public void periodic() {
        states[0].speedMetersPerSecond = Math.abs(frontLeftModule.getDriveVelocity());
        states[1].speedMetersPerSecond = Math.abs(frontRightModule.getDriveVelocity());
        states[2].speedMetersPerSecond = Math.abs(backLeftModule.getDriveVelocity());
        states[3].speedMetersPerSecond = Math.abs(backRightModule.getDriveVelocity());

        odometry.update(getGyroscopeRotation(), states);

        SmartDashboard.putNumber("Drives-Gyro", getGyroscopeRotation().getDegrees());
        SmartDashboard.putString("Robot Pose", getPose().toString());

        field.setRobotPose(getPose());
    }

    
    public void initDrives(){
        frontLeftModule.initAngle();
    }
}