// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.groups.Reset;
import frc.robot.subsystems.arm.Shoulder;
import frc.robot.subsystems.arm.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetFlip extends SequentialCommandGroup {
  private Shoulder shoulder = Shoulder.getInstance();
  private Wrist wrist = Wrist.getInstance();
  private double wristAngle = Constants.arm.configs.RESET[2];

  public SetFlip() {
    addCommands(
      new SequentialCommandGroup(
        new InstantCommand(() -> shoulder.setFlip(!shoulder.getIsFlipped())),
        new Reset(),
        // new ConditionalCommand(
        //   new WaitUntilCommand(() -> wrist.atGoal(-wristAngle)),
        //   new WaitUntilCommand(() -> wrist.atGoal(wristAngle)),
        //     shoulder::getIsFlipped),
        new WaitCommand(1.0),
        new Reset()
        
        )
      );
    
    // Use addRequirements() here to declare subsystem dependencies.
  }




}
