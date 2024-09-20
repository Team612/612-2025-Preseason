package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

public class FieldOrientedDrive extends Command {
  private Drivetrain s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  public FieldOrientedDrive(
      Drivetrain s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed),
        rotationVal * Constants.SwerveConstants.maxAngularVelocity,
        true);
  }
}