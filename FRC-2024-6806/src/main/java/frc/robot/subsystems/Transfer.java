package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

public class Transfer {
private TalonFX TransferMotor1;
private TalonFX TransferMotor2;
 DigitalInput transferstop = new DigitalInput(1);

    
public Transfer(TalonFX TransferMotor1)
  {
    this.TransferMotor1 = TransferMotor1;
  
    
  }
  public void activatetransfer(double TransferSpeed1){
    TransferMotor1.set(TransferSpeed1);
    
    if (transferstop.get()){
      TransferMotor1.set(0);
      
    }
    
  }
}

