// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.List;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveCommands.DefaultDrive;
import frc.robot.commands.DriveCommands.FieldOrientedDrive;
import frc.robot.commands.DriveCommands.LeaveZone;
import frc.robot.commands.IntakeCommands.AutoCommands.AutoEject;
import frc.robot.commands.IntakeCommands.AutoCommands.AutoIntake;
import frc.robot.commands.IntakeCommands.TeleopCommands.IntakeDown;
import frc.robot.commands.IntakeCommands.TeleopCommands.IntakeUp;
import frc.robot.commands.IntakeCommands.TeleopCommands.MoveRollersIn;
import frc.robot.commands.IntakeCommands.TeleopCommands.MoveRollersOut;
import frc.robot.commands.ShooterCommands.AutoCommands.AutoShootAmp;
import frc.robot.commands.ShooterCommands.AutoCommands.AutoShootSpeaker;
import frc.robot.commands.ShooterCommands.TeleopCommands.ShootNoteAmp;
import frc.robot.commands.ShooterCommands.TeleopCommands.ShootNoteSpeaker;
import frc.robot.commands.ShooterCommands.TeleopCommands.ShooterLeftMotor;
import frc.robot.commands.ShooterCommands.TeleopCommands.ShooterRightMotor;
import frc.robot.commands.ShooterCommands.TeleopCommands.SpeedUpAmp;
import frc.robot.commands.ShooterCommands.TeleopCommands.SpeedUpSpeaker;
import frc.robot.commands.TrajectoryCommands.AlignAmp;
import frc.robot.commands.TrajectoryCommands.AlignSpeaker;
import frc.robot.commands.TrajectoryCommands.AlignSpeakerManual;
import frc.robot.commands.TrajectoryCommands.FollowNote;
import frc.robot.commands.TrajectoryCommands.MoveToNote;
import frc.robot.commands.TrajectoryCommands.RunOnTheFly;
import frc.robot.commands.TrajectoryCommands.TrajectoryCreation;
import frc.robot.controls.ControlMap;
import frc.robot.commands.CharacterizationCommands.FeedForwardCharacterization;
import frc.robot.commands.CharacterizationCommands.forwardMeter;
import frc.robot.commands.CharacterizationCommands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.ClimbCommands.ClimbTeleop;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TrajectoryConfiguration;
import frc.robot.subsystems.Vision;
import frc.robot.util.PathPlannerUtil;

public class RobotContainer {
  //Subsystem declerations
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  private final PoseEstimator m_poseEstimator = PoseEstimator.getPoseEstimatorInstance();
  private final TrajectoryConfiguration m_trajectoryConfig = TrajectoryConfiguration.getInstance();
  private final Vision m_vision = Vision.getVisionInstance();
  private final Intake m_intake = Intake.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();
  private final Climb m_climb = Climb.getInstance();

  // Autonomous commands
  private final TrajectoryCreation m_traj = new TrajectoryCreation();
  private final RunOnTheFly m_runOnTheFly = new RunOnTheFly(m_drivetrain, m_poseEstimator, m_traj, m_vision, 0);
  private final FollowNote m_moveToNote = new FollowNote(m_drivetrain, m_poseEstimator, m_traj, m_vision, 0);
  private final AlignAmp m_alignAmp = new AlignAmp(m_poseEstimator, m_traj, m_vision);
  private final AlignSpeaker m_alignSpeaker = new AlignSpeaker(m_poseEstimator, m_traj, m_vision);
  private final MoveToNote m_justMove = new MoveToNote(m_drivetrain, m_vision);
  private final LeaveZone m_leaveZone = new LeaveZone(m_drivetrain);
  private final forwardMeter m_forwardMeter = new forwardMeter(m_drivetrain, m_poseEstimator, m_traj, m_vision, 0);
  private final AlignSpeakerManual m_manualAlign = new AlignSpeakerManual(m_drivetrain, m_vision);
  private final AutoEject m_AutoEject = new AutoEject(m_intake);

  // Drive command
  private final DefaultDrive m_defaultDrive = new DefaultDrive( m_drivetrain,
            () -> -ControlMap.m_driverController.getLeftY(),
            () -> -ControlMap.m_driverController.getLeftX(),
            () -> -ControlMap.m_driverController.getRightX());

  // Gunner commands
  private final IntakeDown m_intakeDown = new IntakeDown(m_intake);
  private final IntakeUp m_intakeUp = new IntakeUp(m_intake);
  private final MoveRollersOut m_moveRollersOut = new MoveRollersOut(m_intake);
  private final MoveRollersIn m_moveRollersIn = new MoveRollersIn(m_intake);
  private final ShootNoteSpeaker m_shootSpeaker = new ShootNoteSpeaker(m_shooter);
  private final ShootNoteAmp m_shootAmp = new ShootNoteAmp(m_shooter);
  // private final ShooterLeftMotor m_shootLeftMotor = new ShooterLeftMotor(m_shooter);
  // private final ShooterRightMotor m_shootRightMotor = new ShooterRightMotor(m_shooter);
  private final AutoShootSpeaker m_autoShootSpeaker = new AutoShootSpeaker(m_shooter, m_intake);
  private final AutoShootAmp m_autoShootAmp = new AutoShootAmp(m_shooter, m_intake);
  // private final SpeedUpSpeaker m_speedUpSpeaker = new SpeedUpSpeaker(m_shooter);
  // private final SpeedUpAmp m_speedUpAmp = new SpeedUpAmp(m_shooter);
  // private final AutoIntake autoIntake = new AutoIntake(m_drivetrain, m_vision, m_intake);
  // private final ClimbTeleop m_climbUp = new ClimbTeleop(m_climb);

  //Drive subsystems declarations 
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();



  // Speaker auto command
  // private final Command scoreSpeaker = new SequentialCommandGroup(
  //   new ParallelCommandGroup(m_alignSpeaker).alongWith(m_speedUpSpeaker)
  //   .andThen(m_moveRollersOut)
  // ).until(() -> ControlMap.m_gunnerController.getRawAxis(1) >= 0.1);

  // // Amp auto command
  // private final Command scoreAmp = new SequentialCommandGroup(
  //   new ParallelCommandGroup(m_alignAmp).alongWith(m_speedUpAmp)
  //   .andThen(m_moveRollersOut)
  // ).until(() -> ControlMap.m_gunnerController.getRawAxis(1) >= 0.1);

  // Intake auto command
  // private final SequentialCommandGroup intakeNote = new SequentialCommandGroup(m_justMove.andThen(m_moveRollersIn));

  //Auto Commands
  // private final SequentialCommandGroup scoreAndLeave = new SequentialCommandGroup(
  //   m_autoShootSpeaker
  //   .andThen(m_trajectoryConfig.followPathGui("Leave Zone Subwoofer"))
  // );

  private boolean isFieldOriented = true;

  public RobotContainer() {
    configureAutoBuilderCommands();
    // Configure the trigger bindings
    configureButtonBindings();
    configureShuffleBoardBindings();
    configureDefaultCommands();
    
  }

  private void configureShuffleBoardBindings(){
    m_chooser.addOption("Run on Fly", m_runOnTheFly);
    m_chooser.addOption("Move to Note", m_moveToNote);
    m_chooser.addOption("Leave Zone", m_leaveZone);
    m_chooser.addOption("forward Meter", m_forwardMeter);
    //m_chooser.addOption("Score Amp", scoreAmp);
    //m_chooser.addOption("Auto Intake", autoIntake);
    m_chooser.addOption("auto speaker", m_autoShootSpeaker);
    m_chooser.addOption("auto amp", m_autoShootAmp);
    m_chooser.addOption("align speaker", m_alignSpeaker);
    m_chooser.addOption("Align Manual", m_manualAlign);
    // m_chooser.addOption("Leave Starting Zone Subwoofer", m_trajectoryConfig.followPathGui("Leave Zone Subwoofer"));
    // m_chooser.addOption("Score and Leave", scoreAndLeave);
    m_chooser.addOption("Swerve Characterization", new FeedForwardCharacterization(
              m_drivetrain,
              true,
              new FeedForwardCharacterizationData("drive"),
              m_drivetrain::runCharacterizationVolts,
              m_drivetrain::getCharacterizationVelocity));

    List<String> autos = AutoBuilder.getAllAutoNames();

    for (String auto : autos) {
      m_chooser.addOption(auto,  AutoBuilder.buildAuto(auto));
    }

    
      
    SmartDashboard.putData(m_chooser);
  }

  private void configureButtonBindings() {
    // Driver button configs
    ControlMap.m_driverController.y().onTrue(new InstantCommand(() -> m_drivetrain.resetAlignment()));
    ControlMap.m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_drivetrain.zeroGyro()));
    ControlMap.m_driverController.b().toggleOnTrue(m_defaultDrive);
    ControlMap.m_driverController.a().onTrue(m_alignSpeaker);
    ControlMap.m_driverController.x().onTrue(m_alignAmp);

    // Gunner button bindings
    ControlMap.m_gunnerController.a().whileTrue(m_intakeDown);
    ControlMap.m_gunnerController.b().whileTrue(m_intakeUp);
    ControlMap.m_gunnerController.y().whileTrue(m_moveRollersOut);
    ControlMap.m_gunnerController.x().whileTrue(m_moveRollersIn);
    ControlMap.m_gunnerController.leftTrigger().whileTrue(m_shootAmp);
    ControlMap.m_gunnerController.rightTrigger().toggleOnTrue((m_shootSpeaker));

    // // FOR TESTING, REMOVE FOR COMP
    // ControlMap.m_gunnerController.leftBumper().whileTrue(m_shootLeftMotor);
    // ControlMap.m_gunnerController.rightBumper().whileTrue(m_shootRightMotor);
  }


  private void configureDefaultCommands(){
    m_drivetrain.setDefaultCommand(
        new FieldOrientedDrive(
            m_drivetrain,
            () -> -ControlMap.m_driverController.getLeftY(),
            () -> -ControlMap.m_driverController.getLeftX(),
            () -> -ControlMap.m_driverController.getRightX()));
    m_climb.setDefaultCommand(
      new ClimbTeleop(m_climb)
    );
  }

  private void configureAutoBuilderCommands(){
    //Intake (MANUAL)
    NamedCommands.registerCommand("Intake Down", m_intakeDown); //brings intake down
    NamedCommands.registerCommand("Intake Up", m_intakeUp); //brings intake up
    NamedCommands.registerCommand("Rollers Out", m_moveRollersOut); //moves the rollers out (ejects) No End Condition
    NamedCommands.registerCommand("Rollers In", m_moveRollersIn);  //moves the rollers in (intakes) End condition if note is detected
    NamedCommands.registerCommand("Apriltag Align", m_alignSpeaker); //auto aligns april tag
    NamedCommands.registerCommand("Shooter On", m_shootSpeaker); //turns on the shooter, no end condition
    NamedCommands.registerCommand("Auto Shoot Speaker", m_autoShootSpeaker); //auto shoots speaker, end condition
    NamedCommands.registerCommand("Auto Eject", m_AutoEject); //auto ejects note, end condition.
  }

 
  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}