// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.*;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kSwerve;
import frc.robot.subsystems.*;

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
    private final DefaultShooter defaultShooterCommand = new DefaultShooter(shooter, driverController::getAButton);
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
        configureClimbController(debugController);
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
        new Button(driverController::getXButton)
                        .whenPressed(new InstantCommand(() -> {
                            acquisition.extendArms(!acquisition.getArmsExtended());  
                        }));
        new Button (driverController::getYButton)
                        .whenPressed(new InstantCommand(() -> {
                            acquisition.setVelocity(3800);
                        }))
                        .whenReleased(new InstantCommand(() -> {
                            acquisition.setVelocity(0);
                        }));
                        
    }

    private void configureClimbController(XboxController controller){
        new Button(controller::getAButton)
                        .whenPressed(new CommandTestClimb(climber, controller));
        new Button(controller::getBButton)
                        .whenPressed(new InstantCommand(() -> climber.rotateArmTo(climber.innerArm, 26)));
        new Button(controller::getXButton)
                        .whenPressed(new InstantCommand(() -> climber.rotateArmTo(climber.innerArm, -27)));
        new Button(controller::getYButton)
                        .whenPressed(new InstantCommand(() -> {climber.rotateArmTo(climber.innerArm, 0);climber.extendArm(climber.innerArm, ClimbConstants.CLIMB_MIN_EXTEND);}));
        new Button(controller::getBackButton)
                        .whenPressed(new InstantCommand(() -> climber.setToCoast()));
        //new Button(controller::getXButton)
        //                 .whenPressed(() -> climber.extendArm(climber.innerArm, 23));
        // new Button(controller::getYButton)
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
}
