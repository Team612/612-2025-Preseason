package frc.robot;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.subsystems.SwerveLib.Conversions;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveLib.OnboardModuleState;
import frc.robot.subsystems.SwerveLib.SwerveModuleConstants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d desiredAngle;

  private TalonFX angleMotor;
  private TalonFX driveMotor;
  private CANcoder angleEncoder;

  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
  private final PositionVoltage anglePosition = new PositionVoltage(0);


  private final PIDController regController = 
    new PIDController(Constants.SwerveConstants.angleKP,
        Constants.SwerveConstants.angleKI,
        Constants.SwerveConstants.angleKD);

  private final SimpleMotorFeedforward feedforward =
    new SimpleMotorFeedforward(
          Constants.SwerveConstants.driveKS,
          Constants.SwerveConstants.driveKV, 
          Constants.SwerveConstants.driveKA);

  private final PIDController turnFeedback =
    new PIDController(0.1, 0.0, 0.0, 0.02);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.desiredAngle = moduleConstants.desiredAngle;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new TalonFX(moduleConstants.angleMotorID);
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new TalonFX(moduleConstants.driveMotorID);
    configDriveMotor();

    lastAngle = getState().angle;

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  /* Desired state for each swerve module. takes in speed and angle. If its openLoop, that means it is in teleop */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  /* Reset wheel orientation to forward */
  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getRotations() - desiredAngle.getRotations(); 
    angleMotor.setPosition(absolutePosition);
    System.out.println(absolutePosition);
  }

  /* Settings for Angle Encoder */
  private void configAngleEncoder() {
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig, 0.1);
  }

  /* Settings for Angle Motor */
  private void configAngleMotor() {
    angleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig, 0.1);
    resetToAbsolute();
  }

  /* Settings for Drive Motor */
  private void configDriveMotor() {
    driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig, 0.1);
    driveMotor.getConfigurator().setPosition(0.0);
  }

  /* Gets the current position of the swerve module. This is an estimate */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      Conversions.rotationsToMeters(driveMotor.getPosition().getValue(), Constants.SwerveConstants.wheelCircumference), 
      Rotation2d.fromRotations(angleMotor.getPosition().getValue())
  );
  }

  /* Sets the speed of the swerve module. If it's openLoop, then it takes in a percentage, otherwise, it calculates and runs a PID */
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      //creating a feedforwad with a desired velocity
      driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference);
      driveVelocity.FeedForward = feedforward.calculate(desiredState.speedMetersPerSecond);
      driveMotor.setControl(driveVelocity);
    }
  }

  /* Sets the angle of the swerve module. */
  private void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;
    
    angleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations())); //degrees because of the conversion factor
    lastAngle = angle;

    // double turnAngleError = Math.abs(angle.getDegrees() - integratedAngleEncoder.getPosition());

    // double pidOut = regController.calculate(integratedAngleEncoder.getPosition(), angle.getDegrees());
    // // if robot is not moving, stop the turn motor oscillating
    // if (turnAngleError < Constants.Swerve.stickDeadband
    //     && Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
    //   pidOut = 0;

    // angleMotor.setVoltage(pidOut * RobotController.getBatteryVoltage());

    // angleMotor.set(pidOut);
  }

  
  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(angleMotor.getPosition().getValueAsDouble() * Constants.SwerveConstants.angleConversionFactor);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      Conversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), Constants.SwerveConstants.wheelCircumference), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return (2 * Math.PI) / 60 * driveMotor.getVelocity().getValueAsDouble();
  }

  public void runCharacterization(double volts) {
    setTurnVoltage(turnFeedback.calculate(getAngle().getRadians(), 0.0));
    setDriveVoltage(volts);
  }

  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  public void setTurnVoltage(double volts) {
    angleMotor.setVoltage(volts);
  }
}
