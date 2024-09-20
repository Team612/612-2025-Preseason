package frc.robot.subsystems.SwerveLib;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public final class CTREConfigs {
  public CANcoderConfiguration swerveCanCoderConfig;
  public TalonFXConfiguration swerveAngleFXConfig;
  public TalonFXConfiguration swerveDriveFXConfig;

  public CTREConfigs() {
    swerveAngleFXConfig = new TalonFXConfiguration();
    swerveDriveFXConfig = new TalonFXConfiguration();
    swerveCanCoderConfig = new CANcoderConfiguration();

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    //swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    /* Drive Configs */
    swerveDriveFXConfig.MotorOutput.Inverted = Constants.SwerveConstants.driveInvert;
    swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.driveNeutralMode;

    /* Limiting */
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.driveEnableCurrentLimit;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.driveContinuousCurrentLimit;
    swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.driveGearRatio;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.SwerveConstants.driveCurrentThreshold;
    swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.SwerveConstants.driveCurrentThresholdTime;

    /* PID */
    swerveDriveFXConfig.Slot0.kP = Constants.SwerveConstants.driveKP;
    swerveDriveFXConfig.Slot0.kI = Constants.SwerveConstants.driveKI;
    swerveDriveFXConfig.Slot0.kD = Constants.SwerveConstants.driveKD;

    /* Angle Configs */
    swerveAngleFXConfig.MotorOutput.Inverted = Constants.SwerveConstants.angleInvert;
    swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.angleNeutralMode;

    /* Gear Ratio and Wrapping Config */
    swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.angleGearRatio;
    swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
    
    /* Current Limiting */
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.angleEnableCurrentLimit;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.angleContinuousCurrentLimit;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.SwerveConstants.angleCurrentThreshold;
    swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.SwerveConstants.angleCurrentThresholdTime;

    /* PID Config */
    swerveAngleFXConfig.Slot0.kP = Constants.SwerveConstants.angleKP;
    swerveAngleFXConfig.Slot0.kI = Constants.SwerveConstants.angleKI;
    swerveAngleFXConfig.Slot0.kD = Constants.SwerveConstants.angleKD;
  }
}