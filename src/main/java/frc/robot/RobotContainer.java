// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.CommandClimb;
import frc.robot.commands.DefaultAcquisition;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Climber;
import frc.robot.commands.DefaultShooter;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(GearRatio.L1);

    private final XboxController m_controller = new XboxController(0);
    private final XboxController debugController = new XboxController(2);
    private final Acquisition acquisition = new Acquisition();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();
    private final DefaultAcquisition acquisitionCommand = new DefaultAcquisition(acquisition);
    private final DefaultShooter shooterCommand = new DefaultShooter(shooter, ()->m_controller.getAButton());
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        acquisition.setDefaultCommand(acquisitionCommand);
        shooter.setDefaultCommand(shooterCommand);
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
                        m_drivetrainSubsystem,
                        () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.7,
                        () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.7,
                        () -> -modifyAxis(m_controller.getRightX()) * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.2
        ));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        //Back button zeros the gyroscope
        new Button(m_controller::getBackButton)
                        .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
        configureClimbController(debugController);
    }

    private void configureClimbController(XboxController controller){
        new Button(controller::getAButton)
                        .whenPressed(new CommandClimb(climber, controller));
        new Button(controller::getBButton)
                        .whenPressed(new InstantCommand(() -> climber.rotateArmTo(climber.outerArm, 26)));
        new Button(controller::getXButton)
                        .whenPressed(new InstantCommand(() -> climber.rotateArmTo(climber.outerArm, -27)));
        new Button(controller::getYButton)
                        .whenPressed(new InstantCommand(() -> {climber.rotateArmTo(climber.outerArm, 0); climber.extendArm(climber.outerArm, 0);}));
        //new Button(controller::getXButton)
        //                 .whenPressed(() -> climber.extendArm(climber.innerArm, 23));
        // new Button(controller::getYButton)
        //                 .whenPressed(() -> climber.extendArm(climber.innerArm, 0));
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}
