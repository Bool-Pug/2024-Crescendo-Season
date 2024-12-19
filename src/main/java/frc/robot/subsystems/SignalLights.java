// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class SignalLights extends SubsystemBase {
  
  public AddressableLED armLEDs;
  public AddressableLEDBuffer armLEDBuffer = new AddressableLEDBuffer(LEDConstants.kArmLEDCount);
  public boolean visualizingIntakeSensor = false;
  public boolean idleAnimation = true;
  public Timer animationTimer = new Timer();
  public int animationCounter = 0;

  public LightSignal currentSignal = LightSignal.databits;

  //These are used to know when we toggle or change the LED state.
  private LightSignal previousSignal;
  private boolean currentBlinkState = false;  //False off, true = on
  private boolean previousNoteState = false;

  public enum LightSignal {
    noteSignaling,
    intaking,
    launchPrep,
    launchReady,
    climbPrep,
    climbFinish,
    databits,
    databitsAnimated
  }

  /** Creates a new SignalLights. */
  public SignalLights() {
    armLEDs = new AddressableLED(LEDConstants.kArmLEDPort);
    animationTimer.start();
    animationCounter = 0;
    armLEDBuffer = new AddressableLEDBuffer(LEDConstants.kArmLEDCount);
    //SetArmLEDBufferToSolidColor(LEDConstants.kDatabitsColor);
    armLEDs.setLength(LEDConstants.kArmLEDCount);
    //armLEDs.setData(armLEDBuffer);
    //armLEDs.start();
    currentSignal = LightSignal.databitsAnimated;
    System.out.println("no note r: " + LEDConstants.kNoNoteColor.red);
    System.out.println("no note g: " + LEDConstants.kNoNoteColor.green);
    System.out.println("no note b: " + LEDConstants.kNoNoteColor.blue);
  }

  @Override
  public void periodic() {
    boolean ledChanged = false;
    //Determine if we push LED update
    if (previousSignal != currentSignal) 
    {
      previousSignal = currentSignal;
      ledChanged = true;
    }
    // This method will be called once per scheduler run
    switch (currentSignal) {
      case noteSignaling:
        boolean hasNote = true;
          if(hasNote != previousNoteState){
            if(hasNote){
              SetArmLEDBufferToSolidColor(LEDConstants.kHasNoteColor);
              ledChanged = true;
            }
            else{
              SetArmLEDBufferToSolidColor(LEDConstants.kNoNoteColor);
              ledChanged = true;
            }
        }
        
        previousNoteState = hasNote;
        break;
      case intaking:
        //This method should return if it needs to change based on the time
        ledChanged = BlinkColorWithTime(LEDConstants.kIntakeColor, LEDConstants.kOffColor, animationTimer.get());
        break;
      case launchPrep:
        SetArmLEDBufferToSolidColor(LEDConstants.kLaunchPrepColor);
        break;
      case launchReady:
        SetArmLEDBufferToSolidColor(LEDConstants.kLaunchReadyColor);
        break;
      case climbPrep:      
        ledChanged = BlinkColorWithTime(LEDConstants.kClimbReadyColor, LEDConstants.kOffColor, animationTimer.get());
        break;
      case climbFinish:
        SetArmLEDBufferToSolidColor(LEDConstants.kClimbColor);
        break;
      case databits:
        SetArmLEDBufferToSolidColor(LEDConstants.kDatabitsColor);
        break;
      case databitsAnimated:
        WaveColorWithTime(LEDConstants.kDatabitsColor, animationTimer.get());
        //This needs to update every loop
        ledChanged = true;
        break;
    
      default:
        SetArmLEDBufferToSolidColor(LEDConstants.kErrorColor);
        break;
    }

    //Only push the LED state if it has changed
    if (ledChanged)
    {
      armLEDs.setData(armLEDBuffer);
      armLEDs.start(); 
    
    }
  }

  private void WaveColorWithTime(Color8Bit color, double timer) {
    animationCounter +=5;
    if(animationCounter>360){
      animationCounter = 0;
    }
    //double timerDeg = Units.degreesToRadians(timer);
    double degPart = armLEDBuffer.getLength();
    for (int i = 0; i < armLEDBuffer.getLength(); i++) {
      
      double brightness  = ((Math.sin( Units.degreesToRadians( i*5 + animationCounter ) ) + 1) / 2) / 8;
      armLEDBuffer.setRGB(i, (int)(brightness * color.red), (int)(brightness * color.green), (int)(brightness * color.blue));
      
   }
   
  }

  public void SetArmLEDBufferToSolidColor(Color8Bit color){

    for (var i = 0; i < armLEDBuffer.getLength(); i++) {
      
      armLEDBuffer.setLED(i, color);
   }
   
    
  }

  public void SetArmLEDBufferToCoolAnimation(){
    for (var i = 0; i < armLEDBuffer.getLength(); i++) {
      
      armLEDBuffer.setRGB(i, 68, 53, (int)Math.round(204 * Math.sin(i + (animationTimer.get()))) );
   }
    
  }

  public void SetArmLEDBufferToAllianceColor(BooleanSupplier isBlueSupplier){

    boolean isBlue = isBlueSupplier.getAsBoolean();
    for (var i = 0; i < armLEDBuffer.getLength(); i++) {

      if(isBlue){
        armLEDBuffer.setRGB(i, 0, 0,128);
      }
      else{
        armLEDBuffer.setRGB(i, 128, 0,0);
      }

   }
  }

  public void SetArmLEDBufferToDatabitsColors(){
    for (var i = 0; i < armLEDBuffer.getLength(); i++) {
      armLEDBuffer.setHSV(i, 68, 204, 53);
    }
  }

  public void Signal(LightSignal newSignal){
    if(newSignal == LightSignal.noteSignaling){
      ResetNoteSignalState();
    }
    currentSignal = newSignal;
  }

  public boolean BlinkColorWithTime(Color8Bit activeColor , Color8Bit inactiveColor, double timer){
    boolean ledChanged = false;
    if(timer % 0.5 > 0.25){
      SetArmLEDBufferToSolidColor(inactiveColor);
      if (currentBlinkState != false) 
      {
        currentBlinkState = false;
        ledChanged = true;
      }
    }
    else{
      SetArmLEDBufferToSolidColor(activeColor);
      if (currentBlinkState != true) 
      {
        currentBlinkState = true;
        ledChanged = true;
      }
    }
    return ledChanged;
  }

  public void ResetNoteSignalState(){
    previousNoteState = false;
  }


  
}
