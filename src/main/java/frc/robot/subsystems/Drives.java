// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
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
        
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(0), new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

        SmartDashboard.putData("Field", field);

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        Mk4ModuleConfiguration config = Mk4ModuleConfiguration.getDefaultSteerFalcon500();
        config.setDriveCurrentLimit(40);
        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(0, 0),
                        config,
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
                        config,
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
                        config,
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
                        config,
                        kSwerve.VEL_GEAR_RATIO,
                        kCANIDs.REAR_RIGHT_DRIVE,
                        kSwerve.CANIVORE_NAME,
                        kCANIDs.REAR_RIGHT_STEER,
                        kSwerve.CANIVORE_NAME,
                        kCANIDs.REAR_RIGHT_CANCODER,
                        kSwerve.CANIVORE_NAME,
                        kSwerve.REAR_RIGHT_MODULE_STEER_OFFSET
        );
        checkStates();
    }

    public void checkStates(){
        SmartDashboard.putBoolean("checkFrontRight", frontRightModule.checkAngle());
        SmartDashboard.putBoolean("checkFrontLeft", frontLeftModule.checkAngle());
        SmartDashboard.putBoolean("checkBackRight", backRightModule.checkAngle());
        SmartDashboard.putBoolean("checkBackLeft", backLeftModule.checkAngle());
    
    }
    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        pigeonTwo.reset();
    }

    public Rotation2d getGyroscopeRotation() {
        return pigeonTwo.getRotation2d();
    }
    public Pigeon2 getGyro() {
        return pigeonTwo;
    }

    public void setOdometryPose(Pose2d pose) {
        pigeonTwo.setYaw(pose.getRotation().getDegrees());
        odometry.resetPosition(pose, pose.getRotation());
    }

    public void fixDeadWheel() {
        frontLeftModule.initAngle();
        frontRightModule.initAngle();
        backLeftModule.initAngle();
        backRightModule.initAngle();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void updateModules(SwerveModuleState[] newStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, MAX_VELOCITY_METERS_PER_SECOND);
        boolean shouldNotTurn = (newStates[0].speedMetersPerSecond == 0 && newStates[1].speedMetersPerSecond == 0 && newStates[2].speedMetersPerSecond == 0 && newStates[3].speedMetersPerSecond == 0);
        frontRightModule.set(newStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, (shouldNotTurn)?states[0].angle.getRadians():newStates[0].angle.getRadians());
        frontLeftModule.set(newStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, (shouldNotTurn)?states[1].angle.getRadians():newStates[1].angle.getRadians());
        backRightModule.set(newStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, (shouldNotTurn)?states[2].angle.getRadians():newStates[2].angle.getRadians());
        backLeftModule.set(newStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, (shouldNotTurn)?states[3].angle.getRadians():newStates[3].angle.getRadians());

        states = newStates;
    }

    public void setRunDrives(boolean runDrive){
        this.runDrive = runDrive;
    }
    public boolean getRunDrives(){
        return runDrive;
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

        SmartDashboard.putNumber("A-Swrv/FL", ((TalonFX)frontLeftModule.getDriveMotor()).getSupplyCurrent());
        SmartDashboard.putNumber("A-Swrv/FR", ((TalonFX)frontRightModule.getDriveMotor()).getSupplyCurrent());
        SmartDashboard.putNumber("A-Swrv/RL", ((TalonFX)backLeftModule.getDriveMotor()).getSupplyCurrent());
        SmartDashboard.putNumber("A-Swrv/RR", ((TalonFX)backRightModule.getDriveMotor()).getSupplyCurrent());

        field.setRobotPose(getPose());
    }
}