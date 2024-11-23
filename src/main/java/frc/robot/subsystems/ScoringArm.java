// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringArmConstants;
import frc.robot.subsystems.SignalLights.LightSignal;

public class ScoringArm extends SubsystemBase {

  public CANSparkMax topIntakeMotor;
  public CANSparkMax bottomIntakeMotor;
  public CANSparkMax floorIntakeMotor;
  public SparkLimitSwitch lowIntakeSensor;
  public SparkLimitSwitch highIntakeSensor;
  public SparkPIDController topIntakePIDController;
  public SparkPIDController bottomIntakePIDController;
  public RelativeEncoder topIntakeEncoder;
  public RelativeEncoder bottomIntakeEncoder;

  public CANSparkMax launchMotorLeader;
  public CANSparkMax launchMotorFollower;
  public RelativeEncoder launchSpeedLeaderEncoder;
  public RelativeEncoder launchSpeedFollowerEncoder;
  public SparkPIDController launchSpeedLeaderPIDController;
  public SparkPIDController launchSpeedFollowerPIDController;
  public double launchSpeedSetpoint = 0;
  public boolean launchCoastMode = false;

  
  public CANSparkMax armAngleLeaderMotor;
  public CANSparkMax armAngleFollowerMotor1i;
  public CANSparkMax armAngleFollowerMotor2;
  public CANSparkMax armAngleFollowerMotor3i;

  public AbsoluteEncoder absArmAngleEncoder;
  public PIDController anglePIDController = new PIDController(ScoringArmConstants.kUpAngleP, ScoringArmConstants.kUpAngleI, ScoringArmConstants.kUpAngleD);
  private boolean armControlEnabled = false;
  private boolean outakeToSensor = false;
  private double sensorOutakeSpeed = 0.1;
  private boolean raisingArm = false;
  private boolean superLaunchSpeed = false;

  public SignalLights signalLights;

  /** Creates a new ScoringArm. */
  public ScoringArm() {

    armAngleLeaderMotor = new CANSparkMax(ScoringArmConstants.kArmAngleMotor1ID, MotorType.kBrushless);
    armAngleFollowerMotor1i = new CANSparkMax(ScoringArmConstants.kArmAngleMotor2ID, MotorType.kBrushless);
    armAngleFollowerMotor2 = new CANSparkMax(ScoringArmConstants.kArmAngleMotor3iID, MotorType.kBrushless);
    armAngleFollowerMotor3i = new CANSparkMax(ScoringArmConstants.kArmAngleMotor4iID, MotorType.kBrushless);

    armAngleLeaderMotor.setInverted(true);
    //armAngleFollowerMotor1i.(armAngleLeaderMotor, false);
    armAngleFollowerMotor1i.setInverted(true);
    //armAngleFollowerMotor2.follow(armAngleLeaderMotor, false);
    armAngleFollowerMotor2.setInverted(false);
    //armAngleFollowerMotor3i.follow(armAngleLeaderMotor, false);
    armAngleFollowerMotor3i.setInverted(false);

    launchMotorLeader = new CANSparkMax(ScoringArmConstants.kLaunchMotorLeaderID, MotorType.kBrushless);
    launchMotorFollower = new CANSparkMax(ScoringArmConstants.kLaunchMotorFollowerID, MotorType.kBrushless);
    launchMotorLeader.setInverted(true);
    launchMotorFollower.setInverted(false);
    launchSpeedLeaderEncoder = launchMotorLeader.getEncoder();
    launchSpeedFollowerEncoder = launchMotorFollower.getEncoder();
    
    topIntakeMotor = new CANSparkMax(ScoringArmConstants.kTopIntakeMotorID, MotorType.kBrushless);
    bottomIntakeMotor = new CANSparkMax(ScoringArmConstants.kBottomIntakeMotorID, MotorType.kBrushless);
    floorIntakeMotor = new CANSparkMax(ScoringArmConstants.kFloorIntakeMotorID, MotorType.kBrushless);

    topIntakeMotor.setInverted(false);
    bottomIntakeMotor.setInverted(true);
    floorIntakeMotor.setInverted(false);

    topIntakePIDController = topIntakeMotor.getPIDController();
    bottomIntakePIDController = bottomIntakeMotor.getPIDController();

    topIntakePIDController.setP(ScoringArmConstants.kIntakeP);
    topIntakePIDController.setI(ScoringArmConstants.kIntakeI);
    topIntakePIDController.setD(ScoringArmConstants.kIntakeD);
    topIntakePIDController.setIZone(ScoringArmConstants.kIntakeIZone);
    topIntakePIDController.setFF(ScoringArmConstants.kIntakeFF);

    bottomIntakePIDController.setP(ScoringArmConstants.kIntakeP);
    bottomIntakePIDController.setI(ScoringArmConstants.kIntakeI);
    bottomIntakePIDController.setD(ScoringArmConstants.kIntakeD);
    bottomIntakePIDController.setIZone(ScoringArmConstants.kIntakeIZone);
    bottomIntakePIDController.setFF(ScoringArmConstants.kIntakeFF);

    topIntakeEncoder = topIntakeMotor.getEncoder();
    bottomIntakeEncoder = bottomIntakeMotor.getEncoder();

    topIntakeEncoder.setPositionConversionFactor(ScoringArmConstants.kIntakePosConversionFactor);
    topIntakeEncoder.setVelocityConversionFactor(ScoringArmConstants.kIntakeVelConversionFactor);

    bottomIntakeEncoder.setPositionConversionFactor(ScoringArmConstants.kIntakePosConversionFactor);
    bottomIntakeEncoder.setVelocityConversionFactor(ScoringArmConstants.kIntakeVelConversionFactor);

    launchSpeedFollowerEncoder.setPositionConversionFactor(ScoringArmConstants.kLaunchPosConversionFactor);
    launchSpeedFollowerEncoder.setVelocityConversionFactor(ScoringArmConstants.kLaunchVelConversionFactor);

    launchSpeedFollowerEncoder.setPositionConversionFactor(ScoringArmConstants.kLaunchPosConversionFactor);//diameter of wheel times pi
    launchSpeedFollowerEncoder.setVelocityConversionFactor(ScoringArmConstants.kLaunchVelConversionFactor);
    
    lowIntakeSensor = topIntakeMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    highIntakeSensor = topIntakeMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    

    absArmAngleEncoder = armAngleLeaderMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    absArmAngleEncoder.setPositionConversionFactor(360);
    absArmAngleEncoder.setVelocityConversionFactor(360);//degrees per second


    anglePIDController.setIZone(ScoringArmConstants.kAngleIZone);
    anglePIDController.setTolerance(ScoringArmConstants.kAnglePosTolerance,ScoringArmConstants.kAngleVelTolerance);
    anglePIDController.setSetpoint(absArmAngleEncoder.getPosition());
    anglePIDController.enableContinuousInput(0, 360);
    

    launchSpeedLeaderPIDController = launchMotorLeader.getPIDController();
    launchSpeedFollowerPIDController = launchMotorFollower.getPIDController();

    launchSpeedLeaderEncoder.setPositionConversionFactor(4*0.254*Math.PI);//diameter of wheel times pi, and gear ration
    launchSpeedLeaderEncoder.setVelocityConversionFactor(4*0.254*Math.PI/60);

    launchSpeedFollowerEncoder.setPositionConversionFactor(4*0.254*Math.PI);//diameter of wheel times pi and gear ration
    launchSpeedFollowerEncoder.setVelocityConversionFactor(4*0.254*Math.PI/60);
    
    launchSpeedLeaderPIDController.setP(ScoringArmConstants.kLaunchSpeedP);
    launchSpeedLeaderPIDController.setI(ScoringArmConstants.kLaunchSpeedI);
    launchSpeedLeaderPIDController.setD(ScoringArmConstants.kLaunchSpeedD);
    launchSpeedLeaderPIDController.setIZone(ScoringArmConstants.kLaunchSpeedIZone);
    launchSpeedLeaderPIDController.setFF(ScoringArmConstants.kLaunchSpeedFF);

    launchSpeedFollowerPIDController.setP(ScoringArmConstants.kLaunchSpeedP);
    launchSpeedFollowerPIDController.setI(ScoringArmConstants.kLaunchSpeedI);
    launchSpeedFollowerPIDController.setD(ScoringArmConstants.kLaunchSpeedD);
    launchSpeedFollowerPIDController.setIZone(ScoringArmConstants.kLaunchSpeedIZone);
    launchSpeedFollowerPIDController.setFF(ScoringArmConstants.kLaunchSpeedFF);
    
    
    //Shuffleboard.getTab("Arm Debug").addDouble("Launch Vel Error", launchSpeedPIDController::getPositionError);
    Shuffleboard.getTab("Arm Debug").addDouble("Launch Vel SP", ()-> launchSpeedSetpoint);
    Shuffleboard.getTab("Arm Debug").addDouble("Launch Vel Encoder", () -> (launchSpeedLeaderEncoder.getVelocity() / ScoringArmConstants.kLaunchVelConversionFactor));
    //Shuffleboard.getTab("Arm Debug").addDouble("Launch Vel Output", () -> launchSpeedPIDController.calculate(10));

    Shuffleboard.getTab("Arm Debug").addDouble("Arm SP", anglePIDController::getSetpoint);

    Shuffleboard.getTab("Arm Debug").addDouble("Intake Vel Encoder", topIntakeEncoder::getVelocity);

    SetLaunchSpeed(0);//theoretical max of 622.9 meters per second
    EnableArmAngleControl(false); 
    resetSetpoints();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(armControlEnabled){
      RunAnglePIDControl();
      
    }
    else{
      SetArmAngleMotors(0);
    }

    

    if(launchCoastMode){
      CoastLaunchMotors();
    }
    else if(superLaunchSpeed){
      SuperLaunchSpeed();
    }
    else{
      RunLaunchSpeedPIDControl();
    }

    if(outakeToSensor){
      if(HighIntakeSensorBlocked()){
        
        SetIntakeMotors(-1 * sensorOutakeSpeed,false);
      }
      else {
        
        SetOutakeToSensor(false);;
        StopIntake();
      }
    }
    
    

    
    //launchMotorLeader.set(0.1);

  }

  public void RunAnglePIDControl(){

    if(anglePIDController.getPositionError() > 5){
      SetFloorIntakeMotor(1);
      raisingArm = true;
    }
    else if (raisingArm){
      SetFloorIntakeMotor(0);
      raisingArm = false;
    }
    SetArmAngleMotors(anglePIDController.calculate(absArmAngleEncoder.getPosition()));
  }

  public void EnableArmAngleControl(boolean enabled){
    armControlEnabled = enabled;
  }


  public void SetArmAngleMotors(double fraction){
    armAngleLeaderMotor.set(fraction);
    armAngleFollowerMotor1i.set(fraction);
    armAngleFollowerMotor2.set(fraction);
    armAngleFollowerMotor3i.set(fraction);
  }

  public void SetArmAngle(double armDeg){
    EnableArmAngleControl(true);
    if ((anglePIDController.getSetpoint() - armDeg) > 20) {
      EnableDownAnglePID();
    }
    else{
      EnableUpAnglePID();
    }
    anglePIDController.setSetpoint(armDeg%360);
  }

  public void ChangeArmAngle(double deg){
    SetArmAngle(anglePIDController.getSetpoint() + deg);
  }

  public double GetArmAngle(){
    return absArmAngleEncoder.getPosition();
  }

  public void SetArmAngleToSDBValue(){
    SetArmAngle(SmartDashboard.getNumber("ArmAngleSlider", 5));
  }

  public boolean ArmAtAngle(){
    return anglePIDController.getPositionError() < 5;
  }

  public void RunLaunchSpeedPIDControl(){//can probably be removed, but haven't tried
    //launchMotorLeader.set(launchSpeedPIDController.calculate(launchSpeedEncoder.getVelocity()));
    launchSpeedLeaderPIDController.setReference(1 * launchSpeedSetpoint, ControlType.kVelocity);
    launchSpeedFollowerPIDController.setReference(-1 * launchSpeedSetpoint, ControlType.kVelocity);
  }

  public void SetLaunchSpeed(double launchRPM){
    
    launchSpeedSetpoint = launchRPM;
    launchCoastMode = false;
  }

  public void SetLaunchSpeedWithOutake(double launchRPM){
    OutakeToSensorSlow();
    launchSpeedSetpoint = launchRPM;
    launchCoastMode = false;
  }

  public void ChangeLaunchSpeed(double deltaRPM){
    SetLaunchSpeed(launchSpeedSetpoint+deltaRPM);
  }

  public void CoastLaunchMotors(){
    launchMotorLeader.set(0);
    launchMotorFollower.set(0);
    launchCoastMode = true;
    superLaunchSpeed = false;
  }

  public void SuperLaunchSpeed(){
    launchMotorLeader.set(1);
    launchMotorFollower.set(-1);
    launchCoastMode = false;
    superLaunchSpeed = true;
  }

  public void Intake(){
    //SetIntakeSpeed(100);
    SetIntakeMotors(0.75,true);
    SetFloorIntakeMotor(1);
  }

  public void Outtake(){
    //SetIntakeSpeed(-100);
    SetIntakeMotors(-0.5,true);
    SetFloorIntakeMotor(-1);
  }

  public void SetIntakeSpeed(double speed){
    topIntakePIDController.setReference(speed, ControlType.kVelocity);
    bottomIntakePIDController.setReference(speed, ControlType.kVelocity);
  }

  public void SetIntakeMotors(double fraction, boolean disableSensorOuttake){
    topIntakeMotor.set(fraction);
    bottomIntakeMotor.set(fraction*(0.6));
    
    if(disableSensorOuttake && outakeToSensor){
      
      SetOutakeToSensor(false);
    }
  }



  public void SetFloorIntakeMotor(double fraction){
    floorIntakeMotor.set(fraction);
  }

  public void EnableIntakeLimits(){
    lowIntakeSensor.enableLimitSwitch(true);
  }

  public void DisableIntakeLimits(){
    lowIntakeSensor.enableLimitSwitch(false);
    
  }

  public void OutakeToSensorSlow(){
    OutakeToSensor(0.1);
  }

  public void OutakeToSensorFast(){
    OutakeToSensor(0.3);
  }

  public void OutakeToSensor(double speed){
    SetOutakeToSensor(true);
    sensorOutakeSpeed = speed;
  }

  public void SetOutakeToSensor(boolean enabled){
    outakeToSensor = enabled;
    

  }

  public void Launch(){
    SetIntakeMotors(1.0, true);
    SetFloorIntakeMotor(1);
  }

  public void resetSetpoints(){

    SetLaunchSpeed(0);//theoretical max of 622.9 meters per second
    anglePIDController.setSetpoint(absArmAngleEncoder.getPosition());
    EnableArmAngleControl(false);
    SetOutakeToSensor(false);
    superLaunchSpeed = false;
    HasNote();
    StopIntake();
    DisableIntakeLimits();
  }

public void StopIntake() {
    SetIntakeMotors(0,true);
    SetFloorIntakeMotor(0);
}

  public boolean atLaunchSetpoint() {
    boolean atSP = (Math.abs(launchSpeedSetpoint-launchSpeedLeaderEncoder.getVelocity()) < 20);
    return atSP;
  }

  public void PrepareClimb(){
    signalLights.Signal(LightSignal.climbPrep);
    SetArmAngle(ScoringArmConstants.kArmPosClimbPrep);
  }

  public void Climb(){
    signalLights.Signal(LightSignal.climbFinish);
    SetArmAngle(ScoringArmConstants.kArmPosClimbFinish);
  }

  public void GoToPickupPos(){
    SetArmAngle(ScoringArmConstants.kArmPosPickup);
  }


  public boolean LowIntakeSensorBlocked() {
    
    boolean pressed = lowIntakeSensor.isPressed();
    return pressed;
  }

  public boolean HighIntakeSensorBlocked() {
    
    boolean pressed = highIntakeSensor.isPressed();
    
    return pressed;
  }

  public boolean HasNote(){
    return HighIntakeSensorBlocked() || LowIntakeSensorBlocked();
  }

  public void AmpPreparation () {
    SetArmAngle(ScoringArmConstants.kArmPosAmp);
    SetLaunchSpeedWithOutake(200);
  }

  public void EnableDownAnglePID(){
    anglePIDController.setP(ScoringArmConstants.kDownAngleP);
    anglePIDController.setI(ScoringArmConstants.kDownAngleI);
    anglePIDController.setD(ScoringArmConstants.kDownAngleD);
    anglePIDController.setIntegratorRange(-1 * ScoringArmConstants.kDownIRange, ScoringArmConstants.kDownIRange);//0.05
  }

  public void EnableUpAnglePID(){
    anglePIDController.setP(ScoringArmConstants.kUpAngleP);
    anglePIDController.setI(ScoringArmConstants.kUpAngleI);
    anglePIDController.setD(ScoringArmConstants.kUpAngleD);
    anglePIDController.setIntegratorRange(-1 * ScoringArmConstants.kUpIRange, ScoringArmConstants.kUpIRange);//0.05

  }

}
