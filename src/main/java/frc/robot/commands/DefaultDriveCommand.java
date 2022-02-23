package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private double lastRotationSpeed = 0;
    private Rotation2d setpointAngle = new Rotation2d(); // Degrees

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        double rotationSpeed = rotationSupplier.getAsDouble();
        // correct for angle
        if (rotationSpeed == 0){
            if (lastRotationSpeed != 0) {
                setpointAngle = drivetrainSubsystem.getGyroscopeRotation();
            }
            Rotation2d angleOffset = setpointAngle.minus(drivetrainSubsystem.getGyroscopeRotation());
            
            SmartDashboard.putNumber("Offset Angle Drives", angleOffset.getDegrees());
            SmartDashboard.putNumber("Gyro Angle Drives", drivetrainSubsystem.getGyroscopeRotation().getDegrees());
            if(Math.abs(angleOffset.getDegrees()) > Constants.SWERVE_ALLOWED_OFFSET){
                rotationSpeed = Constants.SWERVE_CORRECTION_SPEED * (angleOffset.getDegrees()/Math.abs(angleOffset.getDegrees())) ;
            }
        }
        SmartDashboard.putNumber("Setpoint Angle Drives", setpointAngle.getDegrees());
        drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(),
                rotationSpeed, 
                drivetrainSubsystem.getGyroscopeRotation()));
        
        lastRotationSpeed = rotationSpeed;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
