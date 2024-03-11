// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LedSubsystem;

public class IntakeSubsystem extends SubsystemBase
{

  /**
   * Intake Motor objects.
   */
  DigitalInput intake_limit_1 = new DigitalInput(0);
  DigitalInput intake_limit_2 = new DigitalInput(1);
  private static TalonFX upperMotor;
  private static TalonFX lowerMotor;
  private static DoubleSolenoid intakeSolenoid;
  private PneumaticHub hub = new PneumaticHub(23);
  private boolean isUp = false;
  
  private static LedSubsystem led = new LedSubsystem();
  private Transfer transfer;
  public IntakeSubsystem(TalonFX upperMotor, DoubleSolenoid intakeSolenoid,Transfer transfer )
  {
    this.upperMotor = upperMotor;
    this.transfer = transfer;
    this.intakeSolenoid = intakeSolenoid;
    hub.enableCompressorAnalog(100, 120);
  }
  //Sets the intake motor speeds
  public void moveIntake(double upperSpeed, double lowerSpeed,boolean Intake_up, double transferSpeed){
    moveSolenoid(Intake_up);
    upperMotor.set(upperSpeed);
    
    
    led.setColorred();
    
  }
  //Stops the Intake
  public void Gamepiece(){
    led.setcolorgreen();
  }
  public void stopIntake(){
    
    
  }
  public boolean intake_limit(){
    return (intake_limit_1.get()||intake_limit_2.get());
  }
  public void moveSolenoid(boolean open){
    if (open){
      intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    else{
      intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }
  public void Toggle(){
    isUp = !(isUp);
    if (isUp){
      intakeSolenoid.set(DoubleSolenoid.Value.kForward);
      moveIntake(1, 1, isUp, .4);
    }
    else{
      intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
      moveIntake(0, 0, isUp, .4);
    }
  }
}
