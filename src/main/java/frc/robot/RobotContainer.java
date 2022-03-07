// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.acquisition.DefaultAcquisition;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kSwerve;
import frc.robot.subsystems.*;
import frc.robot.utils.AutoUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final PowerDistribution pdp = new PowerDistribution(kCANIDs.PDP, PowerDistribution.ModuleType.kRev);
    public final PneumaticHub pneumaticHub = new PneumaticHub(kCANIDs.PNEUMATIC_HUB);
    public final Compressor compressor = new Compressor(kCANIDs.PNEUMATIC_HUB, PneumaticsModuleType.REVPH);

    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    private final Drives drives = new Drives();
    private final Acquisition acquisition = new Acquisition();
    private final Shooter shooter = new Shooter();
    private final Index index = new Index();
    private final Climber climber = new Climber();

    private final DefaultAcquisition defaultAcquisitionCommand = new DefaultAcquisition(acquisition);
    private final DefaultShooter defaultShooterCommand = new DefaultShooter(shooter);
    private final DefaultIndex defaultIndexCommand = new DefaultIndex(index);
    private final DefaultClimber defaultClimberCommand = new DefaultClimber(climber, operatorController);

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
        drives.setDefaultCommand(new DefaultDriveCommand(
                drives,
                        () -> -modifyAxis(driverController.getLeftY()) * kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 1,
                        () -> -modifyAxis(driverController.getLeftX()) * kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 1,
                        () -> -modifyAxis(driverController.getRightX()) * kSwerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 1
        ));

        acquisition.setDefaultCommand(defaultAcquisitionCommand);
        shooter.setDefaultCommand(defaultShooterCommand);
        index.setDefaultCommand(defaultIndexCommand);
        climber.setDefaultCommand(defaultClimberCommand);

        // Configure the button bindings
        configureDriverControllerBindings();
        configureOperatorControllerBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureDriverControllerBindings() {
        // Back button zeros the gyroscope
        new Button(driverController::getBackButton)
                        .whenPressed(drives::zeroGyroscope);
        new Button(driverController::getStartButton);

        // Colored buttons
        new Button(driverController::getAButton)
                .whenPressed(() -> acquisition.setRollerRPM(3800));
        new Button(driverController::getBButton)
                .whenPressed(() -> acquisition.setRollerRPM(0));
        new Button(driverController::getXButton);
                //.whenPressed(new CommandUnjamRollers(acquisition));
        new Button(driverController::getYButton);

        // POV
        new POVButton(driverController, 0);
        new POVButton(driverController, 90);
        new POVButton(driverController, 180);
        new POVButton(driverController, 270);

        // Bumpers
        new Button(driverController::getRightBumper)
                .whenPressed(() -> acquisition.setRollerRPM(3800));
        new Button(driverController::getLeftBumper)
                .whenInactive(() -> acquisition.setRollerRPM(0));

        // Joystick Buttons
        new Button(driverController::getRightStickButton);
        new Button(driverController::getLeftStickButton);

        // Triggers
        new Trigger(() -> driverController.getRightTriggerAxis() > 0.5)
                .whenActive(new ComplexShootBalls(shooter, index, acquisition));
        new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5);
    }

    private void configureOperatorControllerBindings() {
        // Start/Back
        new Button(operatorController::getStartButton)
                .whenPressed(new ComplexInitializeClimb(climber));
        new Button(operatorController::getBackButton)
                .whenPressed(() -> climber.releaseBreak());

        // Colored buttons
        new Button(operatorController::getAButton)
                .whenPressed((new CommandTestClimb(climber, operatorController))
                        .withInterrupt(() -> operatorController.getPOV() == 0)
                );
        new Button(operatorController::getBButton);
        new Button(operatorController::getXButton); //TODO: Shoot 1 ball
        new Button(operatorController::getYButton);

        // POV
        new POVButton(operatorController, 0); // TODO: Interrupt
        new POVButton(operatorController, 90)
                .whenPressed(new InstantCommand(() -> climber.moveSidewaysPOut(0.5))); //TODO: Climb sideways
        new POVButton(operatorController, 180);
        new POVButton(operatorController, 270)
                .whenPressed(new InstantCommand(() -> climber.moveSidewaysPOut(-0.5)));//TODO: Climb sideways

        // Bumpers
        new Button(operatorController::getRightBumper)
                .whenPressed(() -> acquisition.setRollerRPM(3800));
        new Button(operatorController::getLeftBumper)
                .whenInactive(() -> acquisition.setRollerRPM(0));

        // Joystick Buttons
        new Button(operatorController::getRightStickButton);
        new Button(operatorController::getLeftStickButton);

        // Triggers
        new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5)
                .whenActive(() -> shooter.setVelocity(5200))
                .whenInactive(() -> shooter.setVelocity(0));
        new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5)
                .whenActive(() -> {}); // TODO: index control
    }

    private static double modifyAxis(double rawValue) {
        // Deadband
        double deadband = 0.05;
        double computedValue = rawValue;
        if (Math.abs(computedValue) > deadband) {
            if (computedValue > 0.0) {
                computedValue = (computedValue - deadband) / (1.0 - deadband);
            } else {
                computedValue = (computedValue + deadband) / (1.0 - deadband);
            }
        } else {
            computedValue = 0.0;
        }

        // Square the axis
        computedValue = Math.copySign(computedValue * computedValue, computedValue);

        return computedValue;
    }

    public void runAutonomousRoutine(AutoUtil.Routine routine) {
        switch (routine) {
            case FOUR_BALL:
                AutoUtil.generateCommand("Four-Ball-1", 1, 0.5, drives).schedule();
                break;
            case THREE_BALL:
                AutoUtil.generateCommand("Three-Ball-1", 1, 0.5, drives).schedule();
                break;
            case TWO_BALL:
                AutoUtil.generateCommand("Northern-Two-Ball-1", 1, 0.5, drives).schedule();
                break;
            case POTATO:
                AutoUtil.generateCommand("Potato", 1, 0.5, drives).schedule();
                break;
        }
    }
}
