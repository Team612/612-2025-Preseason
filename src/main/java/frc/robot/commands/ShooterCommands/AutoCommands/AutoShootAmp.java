// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoShootAmp extends Command {
  private Shooter m_shooter;
  private Intake m_intake;

  private Timer time = new Timer();
  /** Creates a new AutoShootAmp. */
  public AutoShootAmp(Shooter s, Intake i) {
    m_shooter = s;
    m_intake = i;
    addRequirements(s, i);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(m_shooter.getCurrent() > 10) {
    //   spikeDone = true;
    // }
    // if(m_shooter.getCurrent() < 6 && spikeDone) {
    if(time.get() >= 1) {
      m_shooter.shoot(Constants.ShooterConstants.shooterLeftSpeedAmp, Constants.ShooterConstants.shooterRightSpeedAmp);
      m_intake.moveRollers(Constants.IntakeConstants.rollerSpeedOuttake);
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
    m_intake.moveRollers(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.get() >= 1.5;
  }
}
