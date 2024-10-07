package frc.robot.subsystems;



import com.kauailabs.navx.frc.AHRS;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveLib.SwerveModule;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain driveSubsystem = null;

  private SwerveModule[] mSwerveMods;

  private static AHRS navx;
  private Rotation2d navxAngleOffset;

  private Field2d fieldLayout;

  private boolean isCharacterizing = false;
  private double characterizationVolts = 0.0;
  private StructArrayPublisher<SwerveModuleState> publisher;

  public Drivetrain() {
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
          new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
          new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
          new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

    publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    
    navx = new AHRS(I2C.Port.kMXP); 
    navxAngleOffset = new Rotation2d();
    navx.reset();

    fieldLayout = new Field2d();
    SmartDashboard.putData("Field", fieldLayout);

  }

  public static Drivetrain getInstance(){
    if (driveSubsystem == null){
       driveSubsystem = new Drivetrain();
    }
    return driveSubsystem;
  }

  //Drives field relative
  public void drive(
      Translation2d translation, double rotation, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getNavxAngle()));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }


  //drives robot relative
  public void driveRobotRelative(
      Translation2d translation, double rotation, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = 
        Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
          new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  //used for autonomous driving
  public void autoDrive(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates =
        Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModule[] getModules(){
    return mSwerveMods;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds result = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
    return result;
  }

  public void zeroGyro() {
    navx.zeroYaw();
  }

  public Rotation2d getNavxAngle(){
    return Rotation2d.fromDegrees(-navx.getAngle());
  }

  public double getEncoderMeters() {
    SwerveModulePosition[] positions = getPositions();
    return positions[0].distanceMeters;
  }

    
  // setter for setting the navxAngleOffset
  public void setNavxAngleOffset(Rotation2d angle){
    navxAngleOffset = angle;
  }

  public Rotation2d getYaw(){
    return Rotation2d.fromDegrees(navx.getYaw());
  }
  public Rotation2d getPitch(){
    return Rotation2d.fromDegrees(navx.getPitch());
  }

  public void resetAlignment() {
    for(SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public void runCharacterizationVolts(double volts) {
    isCharacterizing = true;
    characterizationVolts = volts;
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : mSwerveMods) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  @Override
  public void periodic() {
    publisher.set(getStates());
    // for (SwerveModule mod : mSwerveMods) {
    //   SmartDashboard.putNumber(
    //       "Mod " + mod.moduleNumber + " velocity", mod.getCharacterizationVelocity());
    // }
    SmartDashboard.putNumber("Current Angle", navx.getAngle());
    if (isCharacterizing) {
      // Run in characterization mode
      for (SwerveModule mod : mSwerveMods) {
        mod.runCharacterization(characterizationVolts);
      }
    }
  }
}