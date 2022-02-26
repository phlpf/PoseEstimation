// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.*;
import frc.robot.constants.kSwerve;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final PowerDistribution pdp = new PowerDistribution(61, PowerDistribution.ModuleType.kRev);
    public final PneumaticHub pneumaticHub = new PneumaticHub(31);
    public final Compressor compressor = new Compressor(31, PneumaticsModuleType.REVPH);

    public final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(GearRatio.L1);

    private final XboxController controller = new XboxController(0);
    private final XboxController debugController = new XboxController(2);

    private final Acquisition acquisition = new Acquisition();
    private final Shooter shooter = new Shooter();
    private final Index index = new Index();
    private final Climber climber = new Climber();

    private final DefaultAcquisition defaultAcquisitionCommand = new DefaultAcquisition(acquisition);
    private final DefaultShooter defaultShooterCommand = new DefaultShooter(shooter, controller::getAButton);
    private final DefaultIndex defaultIndexCommand = new DefaultIndex(index, controller::getLeftTriggerAxis);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        pdp.clearStickyFaults();
        pneumaticHub.clearStickyFaults();

        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
                drivetrainSubsystem,
                        () -> -modifyAxis(controller.getLeftY()) * kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.7,
                        () -> -modifyAxis(controller.getLeftX()) * kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.7,
                        () -> -modifyAxis(controller.getRightX()) * kSwerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.2
        ));

        acquisition.setDefaultCommand(defaultAcquisitionCommand);
        shooter.setDefaultCommand(defaultShooterCommand);
        index.setDefaultCommand(defaultIndexCommand);

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
        // Back button zeros the gyroscope
        new Button(controller::getBackButton)
                        .whenPressed(drivetrainSubsystem::zeroGyroscope);
        configureClimbController(debugController);
    }

    private void configureClimbController(XboxController controller){
        new Button(controller::getAButton)
                        .whenPressed(new CommandTestClimb(climber, controller));
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

    private static double modifyAxis(double value) {
        // Deadband
        double deadband = 0.05;
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                value = (value - deadband) / (1.0 - deadband);
            } else {
                value = (value + deadband) / (1.0 - deadband);
            }
        } else {
            value = 0.0;
        }

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}
