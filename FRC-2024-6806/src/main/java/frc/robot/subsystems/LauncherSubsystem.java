// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase
{

  /**
   * Launcher Motor objects.
   */
  private TalonFX launcherMotor1;
  private TalonFX launcherMotor2;
  DigitalInput toplimitSwitch = new DigitalInput(2);
  private Transfer transfer;
  private static LedSubsystem led = new LedSubsystem();
  private static DoubleSolenoid shooterSolenoid;
   private PneumaticHub hub = new PneumaticHub(23);
   private boolean isUp = false;

  public LauncherSubsystem(TalonFX launcherMotor1, TalonFX launcherMotor2,Transfer transfer, DoubleSolenoid shooterSolenoid)
  {
    this.transfer = transfer;
    this.launcherMotor1 = launcherMotor1;
    this.launcherMotor2 = launcherMotor2;
    this.shooterSolenoid = shooterSolenoid;
    hub.enableCompressorAnalog(100, 120);
    

  }
  //Sets the launcher motor speeds
  public void moveLauncher(double launcherSpeed1, double launcherSpeed2,double transferSpeed){
    transfer.activatetransfer(transferSpeed);
    launcherMotor1.set(launcherSpeed1);
    launcherMotor2.set(launcherSpeed2);
    // if (toplimitSwitch.get()){
    //   idlelauncher();
    //   led.setcolorgreen();
    // }
  }
  public void idlelauncher(){
    launcherMotor1.set(.1);
    launcherMotor2.set(.1);
  }
  public boolean Launcher_limit(){
    return toplimitSwitch.get();
  }
  //Stops the launcher moters
  public void stopLauncher(){
    launcherMotor1.set(0);
    launcherMotor2.set(0);
    transfer.activatetransfer(0);
    shooterSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  public void moveSolenoid(boolean isup){
    if (isup){
      shooterSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    else{
      shooterSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }
   public void Toggle(){
    isUp = !(isUp);
    if (isUp){
      shooterSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    else{
      shooterSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }
}
