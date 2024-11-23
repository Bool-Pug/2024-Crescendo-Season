// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringArmConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.FieldDriverStick;
import frc.robot.subsystems.ScoringArm;
import frc.robot.subsystems.SignalLights;
import frc.robot.subsystems.SignalLights.LightSignal;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import swervelib.SwerveController;

public class SpeakerVisionAim extends Command {

  public SwerveSubsystem drivetrain;
  public VisionSubsystem visionSubsystem;
  public SwerveController swerveController;
  public FieldDriverStick driverJoystick;
  public ScoringArm scoringArm;
  public SignalLights signalLights;

  public PhotonTrackedTarget target;
  
  public double rotationTarget = 0;
  public double pidOutput = 0;
  
  public boolean inAuto = false;
  
  public PIDController angleController = new PIDController(0.06, 0, 0);//p: 0.06 i: 0 d: 0

  public Timer targetUpdateTimer = new Timer();
  public double visionUpdateTime = 1.0;
  
  
  /** Creates a new SpeakerVisionAim. */
  public SpeakerVisionAim(SwerveSubsystem drive, VisionSubsystem vision, FieldDriverStick joystick, ScoringArm arm, SignalLights lights, boolean shouldEnd) {
    drivetrain = drive;
    visionSubsystem = vision;
    driverJoystick = joystick;
    scoringArm = arm;
    signalLights = lights;
    swerveController = drivetrain.getSwerveController();
    angleController.enableContinuousInput(-180, 180);
    angleController.setIntegratorRange(-0.3, 0.3);//0.8
    inAuto = shouldEnd;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationTarget = drivetrain.getHeading().getDegrees();

    UpdateTarget();
    if(target != null){
      VisionArmAngle();
    }
    System.out.println("rotation target: "+ rotationTarget);
    System.out.println("current angle: " + drivetrain.getHeading().getDegrees());
  }


  public void UpdateTarget(){

    target = visionSubsystem.getVisibleSpeakerTarget();

    if(target != null){
      Double roughDeltaDeg = Units.millisecondsToSeconds(visionSubsystem.getLatency()) * drivetrain.getRobotVelocity().omegaRadiansPerSecond;
      Rotation2d headingAtTime = drivetrain.getHeading().plus(Rotation2d.fromDegrees(roughDeltaDeg));
      rotationTarget = headingAtTime.getDegrees() - target.getYaw() + VisionConstants.SScameraZYaw;
      //System.out.println("rotation target: "+ rotationTarget);
      
    }
    targetUpdateTimer.restart();
  }

  public void VisionArmAngle(){

    Optional<Pose3d> aprilPoseOptional = visionSubsystem.aprilTagFieldLayout.getTagPose(target.getFiducialId());
    Pose3d appriltagPose3d = (aprilPoseOptional.isPresent() ? aprilPoseOptional.get() : null);
    Optional<EstimatedRobotPose> estimatedRobotPoseOp = visionSubsystem.getEstimatedGlobalPose();
    if(appriltagPose3d != null && estimatedRobotPoseOp.isPresent()){
      

      Pose3d estimatedRobotPose = ((estimatedRobotPoseOp != null && estimatedRobotPoseOp.isPresent()) ? estimatedRobotPoseOp.get().estimatedPose : null);
      double distance = estimatedRobotPose.getTranslation().getDistance(appriltagPose3d.getTranslation());

      //double angle = -4.61 + 26.5 * distance + -2.94 * Math.pow(distance, 2);
      double angle = -35.5 + 55.4 * distance + -9.04 * Math.pow(distance, 2);
      if(distance > 3.35){
        angle = ScoringArmConstants.kArmPosFarStaticLaunch;
      }
      
      if(angle > 80){
        angle = 80;
      }
      else if (angle < 0){
        angle = 0;
      }
      System.out.println("dist: "+ distance);
      scoringArm.SetArmAngle(angle);
    }
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double headingDeg = drivetrain.getHeading().getDegrees();
    pidOutput = angleController.calculate(headingDeg, rotationTarget);
    //pidOutput += Math.signum(pidOutput) * 10;

    //System.out.println("pid output: " + pidOutput);
    // System.out.println("heading: "+ headingDeg);
    drivetrain.drive(new Translation2d(Math.pow(driverJoystick.getX(), 3) * drivetrain.getMaximumVelocity(),
          Math.pow(driverJoystick.getY(), 3) * drivetrain.getMaximumVelocity()),
          Math.pow(pidOutput, 3) * drivetrain.getMaximumAngularVelocity(),
          true
          );

    if(targetUpdateTimer.hasElapsed(visionUpdateTime)){
      UpdateTarget();
    }

    if(angleController.getPositionError() < 5 && scoringArm.atLaunchSetpoint() && scoringArm.ArmAtAngle() && target != null){
      signalLights.Signal(LightSignal.launchReady);
    }
    else{
      signalLights.Signal(LightSignal.launchPrep);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    signalLights.Signal(LightSignal.noteSignaling);
    //drivetrain.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(inAuto){
      return angleController.getPositionError() < 5 && scoringArm.atLaunchSetpoint() && scoringArm.ArmAtAngle() && target != null;
    }
    else{
      return false;
    }
    
  }
}
