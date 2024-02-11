// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase
{

  /**
   * Launcher Motor objects.
   */
  private TalonFX launcherMotor1;
  private TalonFX launcherMotor2;
  DigitalInput toplimitSwitch = new DigitalInput(0);
  private Transfer transfer;

  public LauncherSubsystem(TalonFX launcherMotor1, TalonFX launcherMotor2,Transfer transfer)
  {
    this.transfer = transfer;
    this.launcherMotor1 = launcherMotor1;
    this.launcherMotor2 = launcherMotor2;
    
  }
  //Sets the launcher motor speeds
  public void moveLauncher(double launcherSpeed1, double launcherSpeed2){
    transfer.activatetransfer(1,1);
    launcherMotor1.set(launcherSpeed1);
    launcherMotor2.set(launcherSpeed2);
    if (toplimitSwitch.get()){
      idlelauncher();
    }
  }
  public void idlelauncher(){
    launcherMotor1.set(.1);
    launcherMotor2.set(.1);
  }
  //Stops the launcher moters
  public void stopLauncher(){
    launcherMotor1.set(0);
    launcherMotor2.set(0);
  }
}
