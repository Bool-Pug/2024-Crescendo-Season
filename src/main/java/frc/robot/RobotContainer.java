// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//updated at MMR
package frc.robot;

import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Commands.DeathSpin;
import frc.robot.Commands.ManualArmControl;
import frc.robot.Commands.OutakeNoteToLaunchPos;
import frc.robot.Commands.RunIntakeSmart;
import frc.robot.Commands.SpeakerVisionAim;
import frc.robot.Commands.StaticLaunch;
import frc.robot.Commands.VisionAim;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ScoringArmConstants;
import frc.robot.subsystems.FieldDriverStick;
import frc.robot.subsystems.ScoringArm;
import frc.robot.subsystems.SignalLights;
import frc.robot.subsystems.SignalLights.LightSignal;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  
  public final ScoringArm m_ScoringArm;
  //Long Claw
  //public final String ROBOT_IN_USE = Constants.ROBOT_LONGCLAW_CONFIG_LOCATION;
  //SuperSonic
  public final String ROBOT_IN_USE = Constants.ROBOT_SUPERSONIC_CONFIG_LOCATION;
  
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),ROBOT_IN_USE));

  private static VisionSubsystem m_robotVision = new VisionSubsystem();

  public final SignalLights m_signalLights;

  final SendableChooser<Command> m_autoChooser;

  SendableChooser<Integer> m_poseSelector = new SendableChooser<>();

  private static final Integer positionOne = 1;
  private static final Integer positionTwo = 2;
  private static final Integer positionThree = 3;




  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  FieldDriverStick m_driveStick = new FieldDriverStick(m_driverController);

  // the copilot controller
  Joystick m_copilotController = new Joystick(OIConstants.kCopilotControllerPort);


  //Buttons on the drivers controller
  JoystickButton m_calibrateButton = new JoystickButton(m_driverController, 8);


  //Buttons on the copilots controller
  // JoystickButton m_incLauncherRPM = new JoystickButton(m_copilotController, 4);
  // JoystickButton m_decLauncherRPM = new JoystickButton(m_copilotController, 5);

  // JoystickButton m_intakeButton = new JoystickButton(m_copilotController, 8);
  // JoystickButton m_outtakeButton = new JoystickButton(m_copilotController, 9);


  // JoystickButton m_visionLaunchButton = new JoystickButton(m_copilotController, 7);

  // JoystickButton m_climbPrepArmPosButton = new JoystickButton(m_copilotController, 8);
  // JoystickButton m_climbFinishArmPosButton = new JoystickButton(m_copilotController, 9);
  // JoystickButton m_setLaunchArmPosButton = new JoystickButton(m_copilotController, 10);
  // JoystickButton m_ampArmPosButton = new JoystickButton(m_copilotController, 11);
  


  //Face forward
  Pose2d defaultFaceForwardPose = new Pose2d(2,7,Rotation2d.fromDegrees(0));

  //Face Right, move diagonal
  Pose2d defaultZeroPosition = new Pose2d(0.33 ,0.33,Rotation2d.fromDegrees(0));
  

  //PathPlannerTrajectory path = PathPlannerPath.loadPath("Froggy Demo Path", 1, 1);
  public String myAlliance = DriverStation.getAlliance().toString();

  public boolean driveBrakeMode;
  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    if(ROBOT_IN_USE == Constants.ROBOT_SUPERSONIC_CONFIG_LOCATION){
      m_ScoringArm = new ScoringArm();
    }
    else{
      m_ScoringArm = null;
    }

    m_signalLights = new SignalLights(m_ScoringArm);
    m_ScoringArm.signalLights = m_signalLights;
    
    configureAutoNamedCommands();
    m_autoChooser = AutoBuilder.buildAutoChooser();

    if (Constants.VisionConstants.hasCamera)
    {
      RobotContainer.setRobotVision(new VisionSubsystem(Constants.VisionConstants.SScameraY, Constants.VisionConstants.SScameraX, Constants.VisionConstants.SScameraZ, 
                                    Constants.VisionConstants.SScameraRotation, Constants.VisionConstants.cameraName));
    } 

    // Configure the button bindings
    configureButtonBindings();


    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(m_driveStick.getX(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driveStick.getY(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(m_driveStick.getZ(), OperatorConstants.RIGHT_X_DEADBAND));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(m_driveStick.getX(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driveStick.getY(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(m_driveStick.getZ(), OperatorConstants.RIGHT_X_DEADBAND));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
   

        
    drivebase.zeroGyro();


    //Set default to robot on field position
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    Shuffleboard.getTab("Game HUD").add(m_autoChooser).withSize(3, 1);
    SmartDashboard.putNumber("ArmAngleSlider", 10);
  }

  public void robotInit(){
    if(ROBOT_IN_USE == Constants.ROBOT_SUPERSONIC_CONFIG_LOCATION){
    Shuffleboard.getTab("Game HUD").addDouble("Robot Pitch", (()-> drivebase.getPitch().getDegrees())).withWidget(BuiltInWidgets.kDial);
    Shuffleboard.getTab("Game HUD").addDouble("Arm Angle", m_ScoringArm::GetArmAngle);
    Shuffleboard.getTab("Game HUD").addBoolean("Has Note", m_ScoringArm::HasNote).withSize(6, 6).withWidget(BuiltInWidgets.kBooleanBox);
    Shuffleboard.getTab("Arm HUD").addBoolean("High Sensor", m_ScoringArm::HighIntakeSensorBlocked);
    Shuffleboard.getTab("Arm HUD").addBoolean("Low Sensor", m_ScoringArm::LowIntakeSensorBlocked);
    //Shuffleboard.getTab("Game HUD").add(autoChooser).withSize(2,1);
    }


    m_poseSelector.setDefaultOption("Position 1", positionOne);
    m_poseSelector.addOption("Position 2", positionTwo);
    m_poseSelector.addOption("Position 3", positionThree);
    SmartDashboard.putData(m_poseSelector);

  }

  public void configureAutoNamedCommands(){
    if(ROBOT_IN_USE == Constants.ROBOT_SUPERSONIC_CONFIG_LOCATION){
      NamedCommands.registerCommand("Outake Launch Prep", new OutakeNoteToLaunchPos(m_ScoringArm));
      NamedCommands.registerCommand("Smart Intake", new RunIntakeSmart(m_ScoringArm,m_signalLights,true));
      NamedCommands.registerCommand("Near Static Launch", new StaticLaunch(m_ScoringArm, ScoringArmConstants.kArmPosNearStaticLaunch, 250,false));
      NamedCommands.registerCommand("Far Static Launch", new StaticLaunch(m_ScoringArm, 37 , 250,false));
      NamedCommands.registerCommand("Vision Aim", new SpeakerVisionAim(drivebase, m_robotVision, m_driveStick, m_ScoringArm, m_signalLights,true));
      NamedCommands.registerCommand("Arm Pickup Pos", new InstantCommand(() -> m_ScoringArm.SetArmAngle(ScoringArmConstants.kArmPosPickup)));

    }
    
  }

  public static DriverStation.Alliance allianceColor = DriverStation.Alliance.Blue;

  public static boolean isRedAlliance() 
  {
    return allianceColor.equals(DriverStation.Alliance.Red);
  }

  public static boolean isBlueAlliance() {
    return !isRedAlliance();
  }

  public static void setAlliance(Optional<DriverStation.Alliance> color) {
    if (color.isPresent()) {
      DriverStation.Alliance currentColor = allianceColor;      
      allianceColor = color.get();
      if (!allianceColor.equals(currentColor))
      {
        System.out.println("Changed alliance to " +(isBlueAlliance()?"Blue":"Red"));
      }
    }
  }

  public static VisionSubsystem getRobotVision() {
    return m_robotVision;
  }
  public static void setRobotVision(VisionSubsystem robotVision) {
    m_robotVision = robotVision;
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_calibrateButton.onTrue((new InstantCommand(drivebase::zeroGyro)));

    //m_incLauncherRPM.onTrue(new InstantCommand(() -> m_ScoringArm.ChangeLaunchSpeed(5)));
    //m_decLauncherRPM.onTrue(new InstantCommand(() -> m_ScoringArm.ChangeLaunchSpeed(-5)));

    //m_visionLaunchButton.whileTrue(new AimAndLaunch(m_ScoringArm));

    if (!ROBOT_IN_USE.equals(Constants.ROBOT_LONGCLAW_CONFIG_LOCATION))
    {
      new Trigger(()-> m_driverController.getTrigger()).whileTrue(new StartEndCommand(() -> m_ScoringArm.Launch(), ()-> m_ScoringArm.StopIntake()));//
      

      new JoystickButton(m_copilotController, 5).whileTrue(new RunIntakeSmart(m_ScoringArm,m_signalLights,false));
      new JoystickButton(m_copilotController, 6).whileTrue(new StartEndCommand(() ->m_ScoringArm.Outtake() , () -> m_ScoringArm.StopIntake()));    
      new JoystickButton(m_copilotController, 10).onTrue(new InstantCommand(() -> m_ScoringArm.PrepareClimb()));
      new JoystickButton(m_copilotController, 9).onTrue(new InstantCommand(() -> m_ScoringArm.Climb()));
      new JoystickButton(m_copilotController, 2).onTrue(new InstantCommand(() -> m_ScoringArm.SetArmAngle(ScoringArmConstants.kArmPosNearStaticLaunch)));
      new JoystickButton(m_copilotController, 1).onTrue(new InstantCommand(() -> m_ScoringArm.SetArmAngle(ScoringArmConstants.kArmPosFarStaticLaunch)));
      new JoystickButton(m_copilotController, 3).whileTrue((new StartEndCommand(() -> m_ScoringArm.AmpPreparation(), () -> m_ScoringArm.CoastLaunchMotors())));
      new JoystickButton(m_copilotController, 4).onTrue(new InstantCommand(() -> m_ScoringArm.SetArmAngle(ScoringArmConstants.kArmPosPickup)));
      new Trigger(() -> (m_copilotController.getRawAxis(3) > 0.5)).whileTrue(new StartEndCommand(() -> m_ScoringArm.SetLaunchSpeedWithOutake(250), () -> m_ScoringArm.CoastLaunchMotors()));
      //fix the launch speed
      new Trigger(() -> (m_copilotController.getRawAxis(2) > 0.5)).whileTrue(new StartEndCommand(() -> m_ScoringArm.Launch(), ()-> m_ScoringArm.StopIntake()));//
      new JoystickButton(m_copilotController, 8).whileTrue(new ManualArmControl(m_ScoringArm, ()-> ((( -m_copilotController.getRawAxis(5)+1)/2) * 180 ) ) ) ;//change the 60 to 180
      //new JoystickButton(m_copilotController,7).whileTrue(new StartEndCommand(() -> m_ScoringArm.SuperLaunchSpeed(), () -> m_ScoringArm.CoastLaunchMotors()));
      new JoystickButton(m_copilotController,7).whileTrue(new StartEndCommand(() -> m_ScoringArm.EnableArmAngleControl(false), () -> m_ScoringArm.EnableArmAngleControl(true)));
      
      //new JoystickButton(m_copilotController, 11).onTrue(new InstantCommand(()-> m_ScoringArm.SetArmAngleToSDBValue()));
      //new JoystickButton(m_copilotController, 7).onTrue(new StaticLaunch(m_ScoringArm));

      // new JoystickButton(m_driverController, 3).onTrue(new InstantCommand( ()-> m_ScoringArm.LatchClimb() ) );
      // new JoystickButton(m_driverController, 4).onTrue(new InstantCommand( ()-> m_ScoringArm.UnlatchClimb() ) );
      //new JoystickButton(m_driverController, 3).onTrue(new InstantCommand( ()-> m_ScoringArm.SetFlap(true) ) );
      //new JoystickButton(m_driverController, 4).onTrue(new InstantCommand( ()-> m_ScoringArm.SetFlap(false) ) );
      new JoystickButton(m_driverController, 3).whileTrue(new SpeakerVisionAim(drivebase, m_robotVision, m_driveStick,m_ScoringArm, m_signalLights,false));
      new JoystickButton(m_driverController, 4).whileTrue(new VisionAim(drivebase, m_robotVision, m_driveStick, m_signalLights));
      new JoystickButton(m_driverController, 2).whileTrue(new StaticLaunch(m_ScoringArm, 23, 250,false));
      new JoystickButton(m_driverController, 7).whileTrue(new DeathSpin(drivebase));

    }
    
    //Vision Testing
    
    new JoystickButton(m_driverController, 14).onTrue(new InstantCommand(drivebase::visionPose));

    // new JoystickButton(m_driverController, 5).onTrue(new InstantCommand(() -> signalLights.Signal(LightSignal.launchPrep)));
    // new JoystickButton(m_driverController, 6).onTrue(new InstantCommand(() -> signalLights.Signal(LightSignal.intaking)));
    // new JoystickButton(m_driverController, 7).onTrue(new InstantCommand(() -> signalLights.Signal(LightSignal.climbFinish)));
    //new JoystickButton(m_driverController, 13).whileTrue(Commands.deferredProxy(()-> drivebase.aimAtTarget(
      // m_robotVision.getVisibleSpeakerTarget(),
      // m_driveStick)));
    
    
    //signalLights.setDefaultCommand(new InstantCommand(()->signalLights.SetArmLEDBufferToCoolAnimation()));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //use the command selected in the choose, default is do nothing
    return m_autoChooser.getSelected();
    //return drivebase.sysIdAngleMotorCommand();
  }

/**
   * Set the initial pose of the robot based on pose selected in shuffleboard
   */
  public void setInitialPose()
  {
    //Read robot position from pose selector
    int location = m_poseSelector.getSelected().intValue() - 1;

    //Set default to robot on field position
    if (isRedAlliance()) 
    {
      drivebase.resetOdometry(Constants.PoseConstants.initRobotPoses[3+location]);
    } else {
      drivebase.resetOdometry(Constants.PoseConstants.initRobotPoses[location]);
    }    
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  
  public void setMotorBrake(boolean brake)
  {

    if (brake != driveBrakeMode) {
      drivebase.setMotorBrake(brake);
      driveBrakeMode = brake;
    }
    
  }


  public void zeroGyroWithAlliance()
  {
      if (isRedAlliance())
      {
        drivebase.zeroGyro();
        //Set the pose 180 degrees
        drivebase.resetOdometry(new Pose2d(drivebase.getPose().getTranslation(), Rotation2d.fromDegrees(180)));
      } else {
         drivebase.zeroGyro();      
      }
  }
}
