// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.kClimb;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CommandTestClimb extends SequentialCommandGroup {
  /** Creates a new CommandClimb. */
  public CommandTestClimb(Climber climber, XboxController gamepad) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> climber.rotateArmTo(climber.innerArm, -26)), // TODO: get max and min
      new CommandWaitForButton(gamepad, kClimb.CLIMB_BUTTON),
    
      new InstantCommand(()-> climber.extendArm(climber.outerArm, kClimb.CLIMB_MAX_EXTEND)),
      new CommandWaitForButton(gamepad, kClimb.CLIMB_BUTTON),
      new InstantCommand(()-> climber.extendArm(climber.outerArm, (kClimb.CLIMB_MIN_EXTEND+0.5))),
      new CommandWaitForButton(gamepad, kClimb.CLIMB_BUTTON),
      
      new InstantCommand(()-> climber.extendArm(climber.innerArm, (kClimb.CLIMB_MAX_EXTEND))),
      new CommandWaitForButton(gamepad, kClimb.CLIMB_BUTTON),
      new InstantCommand(()-> climber.outerArm.setAngleToCoast()),
      new InstantCommand(()-> climber.rotateArmTo(climber.innerArm, 0)), // TODO: get max and min
      new CommandWaitForButton(gamepad, kClimb.CLIMB_BUTTON),
      
      new InstantCommand(()-> climber.extendArm(climber.innerArm, 4)),
      new CommandWaitForButton(gamepad, kClimb.CLIMB_BUTTON),
      new InstantCommand(()-> climber.outerArm.setAngleToBrake()),
      
      new InstantCommand(()-> climber.rotateArmTo(climber.outerArm, -26)), // TODO: get max and min
      new CommandWaitForButton(gamepad, kClimb.CLIMB_BUTTON)
    );
  }
}   


