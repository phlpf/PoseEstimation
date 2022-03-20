// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.acquisition.DefaultAcquisition;
import frc.robot.commands.climber.CommandAutoClimb;
import frc.robot.commands.climber.CommandOnCancelClimb;
import frc.robot.commands.climber.DefaultClimber;
import frc.robot.commands.drives.DefaultDriveCommand;
import frc.robot.commands.index.DefaultIndex;
import frc.robot.commands.shooter.ComplexShootBalls;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kControl;
import frc.robot.constants.kSwerve;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Drives;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.climber.Climber;
import frc.robot.utils.AutoUtil;

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

    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        drives.setDefaultCommand(new DefaultDriveCommand(
                drives,
                        () -> -modifyAxis(driverController.getLeftY()) * kSwerve.MAX_VELOCITY_METERS_PER_SECOND * (1 - (modifyAxis(driverController.getLeftTriggerAxis()) * 0.9)),
                        () -> -modifyAxis(driverController.getLeftX()) * kSwerve.MAX_VELOCITY_METERS_PER_SECOND * (1 - (modifyAxis(driverController.getLeftTriggerAxis()) * 0.9)),
                        () -> -modifyAxis(driverController.getRightX()) * kSwerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * (1 - (modifyAxis(driverController.getLeftTriggerAxis()) * 0.9))
        ));

        acquisition.setDefaultCommand(new DefaultAcquisition(acquisition));
        index.setDefaultCommand(new DefaultIndex(index));

        climber.setDefaultCommand(new DefaultClimber(climber,
                () -> 0 * -operatorController.getRightY() * 0.4,
                () -> 0 * operatorController.getRightX() * 0.4,
                () -> 0 * -operatorController.getLeftY() * 0.4,
                () -> 0 * operatorController.getLeftX() * 0.4 // CURRENTLY DISABLED
        ));

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
        // new Button(driverController::getStartButton);

        // Colored buttons
        new Button(driverController::getAButton)
                .whenPressed(new ComplexShootBalls(shooter, index, acquisition, 4, kControl.SHOOTER_LOW_RPMS), false);
        new Button(driverController::getBButton)
                .whenPressed(new ComplexShootBalls(shooter, index, acquisition, 4, kControl.SHOOTER_HIGH_RPMS), false);
        // new Button(driverController::getXButton);
                //.whenPressed(new CommandUnjamRollers(acquisition));
        new Button(driverController::getYButton);


        // POV
        // new POVButton(driverController, 0);
        // new POVButton(driverController, 90);
        // new POVButton(driverController, 180);
        // new POVButton(driverController, 270);

        // Bumpers
        new Button(driverController::getRightBumper)
                .whenPressed(() -> acquisition.setRollerRPM(5000));
        new Button(driverController::getLeftBumper)
                .whenInactive(() -> {acquisition.setRollerRPM(0);
                                     acquisition.retractArms();
                                });

        // Joystick Buttons
        // new Button(driverController::getRightStickButton);
        // new Button(driverController::getLeftStickButton);

        // Triggers
        // new Trigger(() -> driverController.getRightTriggerAxis() > 0.5);
        // new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5);
    }

    private void configureOperatorControllerBindings() {
        // Start/Back
        new Button(operatorController::getStartButton)
                //.whenPressed(new ComplexInitializeClimb(climber));
                .whenPressed(() -> {climber.releaseLock();});
        new Button(operatorController::getBackButton)
                .whenPressed(() -> {climber.extendBreak();});

        // Colored buttons
        new Button(operatorController::getAButton)
                .whenPressed((new CommandAutoClimb(climber, drives, operatorController))
                        .withInterrupt(() -> operatorController.getPOV() == 0)
                );
        new Button(operatorController::getBButton)
                .whenPressed(new ComplexShootBalls(shooter, index, acquisition, 3, kControl.SHOOTER_HIGH_RPMS), false);
        // new Button(operatorController::getXButton); 
        new Button(operatorController::getYButton)
                .whenPressed(new ComplexShootBalls(shooter, index, acquisition, 3, kControl.SHOOTER_LOW_RPMS), false);

        // POV
        new POVButton(operatorController, 0)
                .whenPressed(new CommandOnCancelClimb(climber, drives)); 
        new POVButton(operatorController, 90)
                .whenHeld(new InstantCommand(() -> climber.moveSidewaysPOut(0.5)))
                .whenReleased(new InstantCommand(() -> climber.moveSidewaysPOut(0)));
        // new POVButton(operatorController, 180);
        new POVButton(operatorController, 270)
                .whenHeld(new InstantCommand(() -> climber.moveSidewaysPOut(-0.5)))
                .whenReleased(new InstantCommand(() -> climber.moveSidewaysPOut(0)));

        // Bumpers
        new Button(operatorController::getRightBumper)
                .whenPressed(() -> acquisition.setRollerRPM(5000));
        new Button(operatorController::getLeftBumper)
                .whenInactive(() -> {acquisition.setRollerRPM(0);
                                     acquisition.retractArms();
                                });

        // Joystick Buttons
        // new Button(operatorController::getRightStickButton);
        // new Button(operatorController::getLeftStickButton);

        // Triggers
        // new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5)
        //         .whenActive(() -> shooter.setVelocity(5200))
        //         .whenInactive(() -> shooter.setVelocity(0));
        // new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5)
                // .whenActive(() -> {});
    }

    public void resetSubsystems() {
        pdp.clearStickyFaults();
        pneumaticHub.clearStickyFaults();
        drives.zeroGyroscope();
        acquisition.setRollerRPM(0);
        shooter.setVelocityFront(0);
    }

    public static double modifyAxis(double rawValue) {
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
        drives.setOdometryRotation(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
        switch (routine) {
            case FOUR_BALL:
                new SequentialCommandGroup(
                    AutoUtil.generateCommand("Four-Ball-1", drives),
                    new WaitCommand(2),
                    AutoUtil.generateCommand("Four-Ball-2", drives),
                    new WaitCommand(2),
                    AutoUtil.generateCommand("Four-Ball-3", drives)
                ).schedule();
                break;
            case THREE_BALL:
                AutoUtil.generateCommand("Three-Ball-1", drives);
                break;
            case TWO_BALL:
                new SequentialCommandGroup(
                        new InstantCommand(() -> acquisition.setRollerRPM(3800)),
                        AutoUtil.generateCommand("Northern-Two-Ball-1", 5, 1.6, drives),
                        new ComplexShootBalls(shooter, index, acquisition, 3500),
                        new InstantCommand(() -> acquisition.setRollerRPM(3800)),
                        AutoUtil.generateCommand("Northern-Two-Ball-2", 5, 1.6, drives)
                ).schedule();
                break;
            case POTATO:
                new SequentialCommandGroup(
                        //new ComplexShootBalls(shooter, index, acquisition, 3000),
                        AutoUtil.generateCommand("Potato", 5, 1.6, drives)
                ).schedule();
                break;
        }
    }
}
