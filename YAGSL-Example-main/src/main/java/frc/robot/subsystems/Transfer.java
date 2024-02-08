package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

public class Transfer {
private TalonFX TransferMotor1;
private TalonFX TransferMotor2;

    
public Transfer(TalonFX TransferMotor1, TalonFX TransferMotor2)
  {
    this.TransferMotor1 = TransferMotor1;
    this.TransferMotor2 = TransferMotor2;
    
  }
  public void moveLauncher(double TransferSpeed1, double TransferSpeed2){
    TransferMotor1.set(TransferSpeed1);
    TransferMotor2.set(TransferSpeed2);
  }
}

