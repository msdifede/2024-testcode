package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climber extends SubsystemBase {
  
  // Climber Motor Controllers
  private TalonFX m_climber1; // 775pro motor
  private TalonFX m_climber2;

  /** Subsystem for controlling the climber */
  public climber() {


    m_climber1 = new TalonFX(25);
    m_climber2 = new TalonFX(26);




    // Instantiate the climber motor controller
    // m_climber = new TalonSRX(Constants.CLIMBER_MOTOR_ID);
    // m_climber.configFactoryDefault();
    // m_climber.setNeutralMode(NeutralMode.Brake);


    // Reverse it if needed
    // m_climber.setInverted(Constants.CLIMBER_INVERT);

    // Put the default speed on SmartDashboard
    //SmartDashboard.putNumber("Climber Speed", Constants.CLIMBER_SPEED);
  }

  /* Set power to the climber motor */
  public void setPower(double power) {

    m_climber1.set(power);
    m_climber2.set(power);
  }
  public void stop() {
    setPower(0);
  }

  @Override
  public void periodic() {}
}
