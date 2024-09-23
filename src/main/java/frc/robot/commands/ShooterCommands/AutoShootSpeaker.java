// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoShootSpeaker extends Command {
  private Shooter m_shooter;
  private Intake m_intake;
  private boolean spikeDone;
  private int count = 0;
  private Timer time = new Timer();
  /** Creates a new AutoShootSpeaker. */
  public AutoShootSpeaker(Shooter s, Intake i) {
    m_shooter = s;
    m_intake = i;
    addRequirements(s, i);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
    spikeDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(m_intake.getIRSensor() < 0.3) {
    //   count++;
    // } else {
    //   count = 0;
    // }
    // if(m_shooter.getCurrent() > 30) {
    //   spikeDone = true;
    //   time.start();
    // }
    // if(m_shooter.getCurrent() < 16 && spikeDone) {
      if(time.get() >= 5) {
        m_shooter.shoot(Constants.ShooterConstants.shooterLeftSpeedSpeaker, Constants.ShooterConstants.shooterRightSpeedSpeaker);
        m_intake.moveRollers(Constants.IntakeConstants.rollerSpeedOuttake);
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
    time.stop();
    time.reset();
    m_shooter.shoot(0, 0);
    m_intake.moveRollers(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return count >= 10;
    return time.get() >= 7;
  }
}
