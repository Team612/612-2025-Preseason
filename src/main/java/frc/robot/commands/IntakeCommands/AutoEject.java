// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rollers;

public class AutoEject extends Command {
  private final Rollers m_rollers;
  private double count = 0;
  private Timer time = new Timer();
  /** Creates a new MoveRollers. */
  public AutoEject(Rollers rollers) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_rollers = rollers;
    addRequirements(rollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rollers.moveRollers(Constants.IntakeConstants.rollerSpeedOuttake);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
    time.reset();
    m_rollers.moveRollers(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_Intake.getIRSensor();
    return time.get() >= 1.5;
  }
}