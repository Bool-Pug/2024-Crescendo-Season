// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringArm;

public class StaticLaunch extends Command {
  ScoringArm m_ScoringArm;
  double launchSpeed;
  double launchAngle;
  Timer timeoutTimer;
  Timer noNoteStopDelayTimer;
  Timer launchTimer;
  boolean wasAtSP = false;
  boolean useVision = false;

  /** Creates a new StaticLaunch. */
  public StaticLaunch(ScoringArm arm,double angle,double speed,boolean tryVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ScoringArm = arm;
    launchAngle = angle;
    launchSpeed = speed;
    timeoutTimer = new Timer();
    noNoteStopDelayTimer = new Timer();
    launchTimer = new Timer();
    useVision = tryVision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if( !(useVision && m_ScoringArm.GetArmAngle() > 5) ){
      m_ScoringArm.SetArmAngle(launchAngle);
    }
    
    m_ScoringArm.SetLaunchSpeed(launchSpeed);
    m_ScoringArm.OutakeToSensorSlow();

    noNoteStopDelayTimer.reset();
    noNoteStopDelayTimer.stop();
    
    timeoutTimer.reset();
    timeoutTimer.start();
    launchTimer.reset();
    wasAtSP = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ScoringArm.CoastLaunchMotors();
    m_ScoringArm.StopIntake();
    System.out.println("launch time: "+ launchTimer.get());
    launchTimer.reset();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean atArmSP = m_ScoringArm.ArmAtAngle();

    if((atArmSP || timeoutTimer.hasElapsed(3)) && launchTimer.get() <= 0.1 && !m_ScoringArm.HighIntakeSensorBlocked()){
      //m_ScoringArm.SetLaunchSpeed(launchSpeed);
      launchTimer.start();
    }

    if(launchTimer.hasElapsed(0.5) && m_ScoringArm.atLaunchSetpoint() && !m_ScoringArm.HighIntakeSensorBlocked() && atArmSP){
      m_ScoringArm.Launch();
    }

    if(!m_ScoringArm.HasNote()){
      noNoteStopDelayTimer.start();
    }

    return (launchTimer.hasElapsed(2) && !m_ScoringArm.HighIntakeSensorBlocked()) || noNoteStopDelayTimer.hasElapsed(0.125);
    
    
  }

    
  
}
