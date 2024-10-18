// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Shooter;

public class AutoShootSpeaker extends Command {
  private Shooter m_shooter;
  private Rollers m_rollers;
  private boolean spikeDone;
  private Timer time = new Timer();
  /** Creates a new AutoShootSpeaker. */
  public AutoShootSpeaker(Shooter s, Rollers r) {
    m_shooter = s;
    m_rollers = r;
    addRequirements(s, r);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spikeDone = false;
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(m_shooter.getCurrent() > 10) {
    //   spikeDone = true;
    // }
    // if(m_shooter.getCurrent() < 6 && spikeDone) {
    if(time.get() >= 1.5) {
      m_shooter.shoot(Constants.ShooterConstants.shooterLeftSpeedSpeaker, Constants.ShooterConstants.shooterRightSpeedSpeaker);
      m_rollers.moveRollers(Constants.IntakeConstants.rollerSpeedOuttake);
    } else {
      m_shooter.shoot(Constants.ShooterConstants.shooterLeftSpeedSpeaker, Constants.ShooterConstants.shooterRightSpeedSpeaker);
    }
    // } else {
    //   m_shooter.shoot(Constants.ShooterConstants.shooterLeftSpeedSpeaker, Constants.ShooterConstants.shooterRightSpeedSpeaker);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.shoot(0, 0);
    m_rollers.moveRollers(0);
    time.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.get() >= 2;
  }
}
