package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

public class Transfer {
private TalonFX TransferMotor1;
 DigitalInput transferstop_1 = new DigitalInput(3);
 DigitalInput transferstop_2 = new DigitalInput(4);

    
public Transfer(TalonFX TransferMotor1)
  {
    this.TransferMotor1 = TransferMotor1;
  
    
  }
  public void activatetransfer(double TransferSpeed1){
    TransferMotor1.set(TransferSpeed1);
    
    if (transferstop_1.get()||transferstop_2.get()){
      TransferMotor1.set(0);
      
    }
    
  }
}

