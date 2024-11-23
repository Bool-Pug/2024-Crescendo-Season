// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringArmConstants;
import frc.robot.subsystems.ScoringArm;
import frc.robot.subsystems.SignalLights;
import frc.robot.subsystems.SignalLights.LightSignal;

public class RunIntakeSmart extends Command {

  public ScoringArm m_ScoringArm;
  public Timer stopDelayTimer;
  public Timer timeoutTimer;
  public boolean sensorTriggered = false;
  public boolean launchPrepAfter = false;
  public SignalLights signalLights;
  

  /** Creates a new RunIntakeSmart. */
  public RunIntakeSmart(ScoringArm arm, SignalLights lights, boolean launchPos) {
    stopDelayTimer = new Timer();
    timeoutTimer = new Timer();
    signalLights = lights;
    m_ScoringArm = arm;
    launchPrepAfter = launchPos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ScoringArm.Intake();
    stopDelayTimer.reset();
    stopDelayTimer.stop();

    sensorTriggered = false;
    signalLights.Signal(LightSignal.intaking);
    timeoutTimer.restart();
    m_ScoringArm.EnableIntakeLimits();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_ScoringArm.Intake();
    
    
    
    

    if (!sensorTriggered && m_ScoringArm.LowIntakeSensorBlocked()) {
      stopDelayTimer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    signalLights.Signal(LightSignal.noteSignaling);
    m_ScoringArm.StopIntake();

    if(launchPrepAfter){
      m_ScoringArm.SetArmAngle( (ScoringArmConstants.kArmPosNearStaticLaunch+ScoringArmConstants.kArmPosFarStaticLaunch) / 2 );
      m_ScoringArm.SetLaunchSpeed(250);
    }

    m_ScoringArm.OutakeToSensorSlow();
    m_ScoringArm.DisableIntakeLimits();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (m_ScoringArm.LowIntakeSensorBlocked()) {
      sensorTriggered = true;
    }

    if (ScoringArmConstants.kUseSquished) {
      return stopDelayTimer.hasElapsed(1);
    }
    else{
      return m_ScoringArm.LowIntakeSensorBlocked() || (launchPrepAfter && timeoutTimer.hasElapsed(8));
      //return m_ScoringArm.LowIntakeSensorBlocked();
    }

    
    
  }
}
