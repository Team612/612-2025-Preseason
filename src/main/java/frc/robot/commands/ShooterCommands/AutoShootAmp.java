// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Shooter;

public class AutoShootAmp extends Command {
  private Shooter m_shooter;
  private Rollers m_rollers;
  private boolean spikeDone;
  private Timer time = new Timer();
  /** Creates a new AutoShootAmp. */
  public AutoShootAmp(Shooter s, Rollers r) {
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
    if(time.get() >= 5) {
      m_shooter.shoot(Constants.ShooterConstants.shooterLeftSpeedAmp, Constants.ShooterConstants.shooterRightSpeedAmp);
      m_rollers.moveRollers(Constants.IntakeConstants.rollerSpeedOuttake);
    } else {
      m_shooter.shoot(Constants.ShooterConstants.shooterLeftSpeedAmp, Constants.ShooterConstants.shooterRightSpeedAmp);
    }
    // } else {
    //   m_shooter.shoot(Constants.ShooterConstants.shooterLeftSpeedAmp, Constants.ShooterConstants.shooterRightSpeedAmp);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.shoot(0, 0);
    m_rollers.moveRollers(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.get() >= 7;
  }
}
