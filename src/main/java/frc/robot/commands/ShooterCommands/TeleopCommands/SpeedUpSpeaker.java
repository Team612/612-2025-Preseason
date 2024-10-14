// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class SpeedUpSpeaker extends Command {
  /** Creates a new SpeedUpShooter. */
  private Shooter m_shooter;
  private boolean spikeDone;
  public SpeedUpSpeaker(Shooter shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_shooter.getCurrent() > 10) {
      spikeDone = true;
    }
    m_shooter.shoot(Constants.ShooterConstants.shooterLeftSpeedSpeaker, Constants.ShooterConstants.shooterRightSpeedSpeaker);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.getCurrent() < 6 && spikeDone;
  }
}
