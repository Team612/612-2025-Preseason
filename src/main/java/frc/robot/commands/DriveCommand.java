// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WestCoastDrive;
import java.util.function.DoubleSupplier;
public class DriveCommand extends Command {
  /** Creates a new DriveCommand. */
  private final DoubleSupplier forward;
  private final DoubleSupplier rotation;

  private final WestCoastDrive m_westcoast;

  public DriveCommand(WestCoastDrive driveSubsystem, DoubleSupplier forward, DoubleSupplier rotation) {
    this.m_westcoast = driveSubsystem;
    this.forward = forward;
    this.rotation = rotation;

    // Declare subsystem dependency
    addRequirements(driveSubsystem);
}
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_westcoast.drive(forward.getAsDouble(), rotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_westcoast.drive(0.0,0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
