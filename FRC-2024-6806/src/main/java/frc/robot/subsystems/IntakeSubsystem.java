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
  DigitalInput intake_limit = new DigitalInput(0);
  private static TalonFX upperMotor;
  private static TalonFX lowerMotor;
  private static DoubleSolenoid intakeSolenoid;
  private PneumaticHub hub = new PneumaticHub(23);
  private static LedSubsystem led = new LedSubsystem();
  public IntakeSubsystem(TalonFX upperMotor, TalonFX lowerMotor, DoubleSolenoid intakeSolenoid)
  {
    this.upperMotor = upperMotor;
    this.lowerMotor = lowerMotor;
    this.intakeSolenoid = intakeSolenoid;
  }
  //Sets the intake motor speeds
  public void moveIntake(double upperSpeed, double lowerSpeed,boolean Intake_up){
    moveSolenoid(Intake_up);
    upperMotor.set(upperSpeed);
    lowerMotor.set(lowerSpeed);
    led.setColorred();
    if (intake_limit.get()){
      stopIntake();
      led.SetGamepiece();
    }
  }
  //Stops the Intake
  public void stopIntake(){
    upperMotor.set(0);
    lowerMotor.set(0);
  }
  public boolean intake_limit(){
    return intake_limit.get();
  }
  public void moveSolenoid(boolean open){
    if (open){
      intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    else{
      intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }
}
