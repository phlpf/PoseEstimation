package frc.robot.commands.drives;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.kSwerve;
import frc.robot.subsystems.drives.Drives;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final Drives drives;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    private double lastRotationSpeed = 0;
    private Rotation2d setpointAngle = new Rotation2d(); // Degrees

    public DefaultDriveCommand(Drives drives, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.drives = drives;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drives);
    }

    @Override
    public void execute() {
        if(DriverStation.isTeleop() && drives.getRunDrives()) {
            double rotationSpeed = rotationSupplier.getAsDouble();

            // correct for angle 
            if (rotationSpeed == 0) {
                if (lastRotationSpeed != 0) {
                    setpointAngle = drives.getRotation();
                }
                Rotation2d angleOffset = setpointAngle.minus(drives.getRotation());

                if (Math.abs(angleOffset.getDegrees()) > kSwerve.SWERVE_ALLOWED_OFFSET) {
                    rotationSpeed = kSwerve.SWERVE_CORRECTION_SPEED * (angleOffset.getDegrees() / Math.abs(angleOffset.getDegrees()));
                }
            }

            drives.updateModules(
                    drives.getKinematics().toSwerveModuleStates(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translationXSupplier.getAsDouble(),
                                    translationYSupplier.getAsDouble(),
                                    rotationSpeed,
                                    drives.getRotation()
                            )
                    )
            );

            lastRotationSpeed = rotationSpeed;
        }
    }

    

    @Override
    public void end(boolean interrupted) {
        drives.updateModules(drives.getKinematics().toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0)));
    }
}
