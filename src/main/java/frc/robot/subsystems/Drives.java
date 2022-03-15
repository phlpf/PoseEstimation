// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.ctre.CanCoderFactoryBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kSwerve;
import frc.robot.utils.PigeonWrapper;

import static frc.robot.constants.kSwerve.*;

public class Drives extends SubsystemBase {
    ProfiledPIDController thetaController;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;
    private boolean runDrive = true;

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

    private final SwerveDriveOdometry odometry;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final Field2d field = new Field2d();

    private SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

    public Drives() {
        pigeon.configFactoryDefault();
        pigeon.reset();
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 1000);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 300);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 20);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 20);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 200);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 40);

        odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

        SmartDashboard.putData("Field", field);

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

        setupModule(frontLeftModule);

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

        setupModule(frontRightModule);

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

        setupModule(backLeftModule);

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

        setupModule(backRightModule);

    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        pigeon.reset();
        odometry.resetPosition(new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0)), getGyroscopeRotation());
    }

    public Rotation2d getGyroscopeRotation() {
        return pigeon.getRotation2d();
    }

    public void setOdometryRotation(Pose2d pose) {
        pigeon.setYaw(pose.getRotation().getDegrees());
        odometry.resetPosition(pose, getGyroscopeRotation());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void updateModules(SwerveModuleState[] newStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, MAX_VELOCITY_METERS_PER_SECOND);
        frontRightModule.set(newStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, newStates[0].angle.getRadians());
        frontLeftModule.set(newStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, newStates[1].angle.getRadians());
        backRightModule.set(newStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, newStates[2].angle.getRadians());
        backLeftModule.set(newStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, newStates[3].angle.getRadians());

        if(DriverStation.isAutonomous()) states = newStates;
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }
    public void setRunDrives(boolean runDrives){
        this.runDrive = runDrives;
    }

    private void setupModule(SwerveModule module) {
        TalonFX driveMotor = ((TalonFX)module.getDriveMotor());

        TalonFX angleMotor = ((TalonFX)module.getDriveMotor());
        angleMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 1000);
        angleMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        angleMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 100);
        angleMotor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 1000);

        CANCoder canCoder = ((CanCoderFactoryBuilder.EncoderImplementation)module.getSteerEncoder()).getEncoder();
        canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
    }

    public Field2d getField() {
        return field;
    }

    @Override
    public void periodic() {
        if(runDrive && !DriverStation.isAutonomous()) {
            states = kinematics.toSwerveModuleStates(chassisSpeeds);
            updateModules(states);
        }

        states[0].speedMetersPerSecond = Math.abs(frontLeftModule.getDriveVelocity());
        states[1].speedMetersPerSecond = Math.abs(frontRightModule.getDriveVelocity());
        states[2].speedMetersPerSecond = Math.abs(backLeftModule.getDriveVelocity());
        states[3].speedMetersPerSecond = Math.abs(backRightModule.getDriveVelocity());

        odometry.update(getGyroscopeRotation(), states);

        //TODO: add current for all module motors

        SmartDashboard.putNumber("Drives-Gyro", getGyroscopeRotation().getDegrees());

        field.setRobotPose(getPose());
    }
}