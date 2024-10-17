// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls.ControlMap;
import frc.robot.subsystems.Climb;

public class ClimbTeleop extends Command {
  /** Creates a new ClimbUp. */
  private final Climb m_climb;
  public ClimbTeleop(Climb c) {
    m_climb = c;
    addRequirements(c);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climb.setLeftSpeed(0);
    m_climb.setRightSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = ControlMap.m_gunnerController.getLeftY();
    double rightSpeed = ControlMap.m_gunnerController.getRightY();
    if(leftSpeed < -Constants.SwerveConstants.stickDeadband) {
      leftSpeed = -Constants.ClimbConstants.climbLeftSpeed;
    } else if(leftSpeed > Constants.SwerveConstants.stickDeadband) {
      leftSpeed = Constants.ClimbConstants.climbLeftSpeed;
    } else {
      leftSpeed = 0;
    }
    if(rightSpeed < -Constants.SwerveConstants.stickDeadband) {
      rightSpeed = -Constants.ClimbConstants.climbRightSpeed;
    } else if(rightSpeed > Constants.SwerveConstants.stickDeadband) {
      rightSpeed = Constants.ClimbConstants.climbRightSpeed;
    } else {
      rightSpeed = 0;
    }
    m_climb.setLeftSpeed(leftSpeed);
    m_climb.setRightSpeed(rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.setLeftSpeed(0);
    m_climb.setRightSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
