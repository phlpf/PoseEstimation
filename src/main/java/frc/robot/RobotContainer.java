// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.CommandMoveAngle.CurrentLimit;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kClimb;
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
    private final XboxController debugController = new XboxController(2);

    private final Drives drives = new Drives();
    private final Acquisition acquisition = new Acquisition();
    private final Shooter shooter = new Shooter();
    private final Index index = new Index();
    private final Climber climber = new Climber();

    private final DefaultAcquisition defaultAcquisitionCommand = new DefaultAcquisition(acquisition);
    private final DefaultShooter defaultShooterCommand = new DefaultShooter(shooter);
    private final DefaultIndex defaultIndexCommand = new DefaultIndex(index, driverController::getLeftTriggerAxis);


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
                        () -> -modifyAxis(driverController.getLeftY()) * kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.7,
                        () -> -modifyAxis(driverController.getLeftX()) * kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.7,
                        () -> -modifyAxis(driverController.getRightX()) * kSwerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.2
        ));

        acquisition.setDefaultCommand(defaultAcquisitionCommand);
        shooter.setDefaultCommand(defaultShooterCommand);
        index.setDefaultCommand(defaultIndexCommand);

        // Configure the button bindings
        configureDriverControllerBindings();
        configureOperatorControllerBindings();
        configureClimbController();
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

        // Colored buttons
        new Button(driverController::getAButton)
                .whenPressed(() -> acquisition.setArmsExtended(!acquisition.getArmsExtended()));

        // Bumpers
        new Button(driverController::getRightBumper)
                .whenPressed(() -> {}); // TODO create shoot command
        new Button(driverController::getLeftBumper)
                .whenPressed(() -> acquisition.setRollerVelocity(-2600))
                .whenReleased(() -> acquisition.setRollerVelocity(0));

        // Triggers
        new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
                .whenActive(() -> acquisition.setRollerVelocity(3800))
                .whenInactive(() -> acquisition.setRollerVelocity(0));
    }

    private void configureOperatorControllerBindings() {
        // Start/Back
        new Button(operatorController::getBackButton)
                .whenPressed(() -> {}); // TODO: create run everything backwards command

        // Colored buttons

        // Bumpers
        new Button(operatorController::getRightBumper)
                .whenPressed(() -> {}); // TODO: index control

        // Triggers
        new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5)
                .whenActive(() -> shooter.setVelocity(5200))
                .whenInactive(() -> shooter.setVelocity(0));
        new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5)
                .whenActive(() -> {}); // TODO: index control
    }

    private void configureClimbController(){
        new Button(debugController::getAButton)
                        .whenPressed((new CommandTestClimb(climber, debugController))
                                      .withInterrupt(debugController::getLeftBumper)
                        );
        new Button(debugController::getBButton)
                        .whenPressed(new InstantCommand(() -> climber.rotateArmTo(climber.innerArm, 26)));
        new Button(debugController::getXButton)
                        .whenPressed(new CommandMoveAngle(climber.outerArm, 100, CurrentLimit.OFF, kClimb.CLIMB_ANGLE_ALLOWED_ERROR_EXACT));
        new Button(debugController::getYButton)
                        .whenPressed(new InstantCommand(() -> {climber.rotateArmTo(climber.innerArm, 0);climber.extendArm(climber.innerArm, kClimb.CLIMB_MIN_EXTEND);}));
        new Button(debugController::getBackButton)
                        .whenPressed(new InstantCommand(() -> climber.setToCoast()));
        new Button(debugController::getLeftBumper)
                        .whenPressed(new InstantCommand(() -> climber.setToBrake()));
        new Button(debugController::getStartButton)
                        .whenPressed(new ComplexInitializeClimb(climber));
        //new Button(debugController::getXButton)
        //                 .whenPressed(() -> climber.extendArm(climber.innerArm, 23));
        // new Button(debugController::getYButton)
        //                 .whenPressed(() -> climber.extendArm(climber.innerArm, 0));
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
            case FORWARD:
                AutoUtil.generateCommand("Test", 1, 0.5, drives).schedule();
                break;
        }
    }
}
