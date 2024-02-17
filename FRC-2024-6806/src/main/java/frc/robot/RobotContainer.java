// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.commands.PathfindingCommand;

import frc.robot.commands.swervedrive.Assist.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/falcon"));
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);
  XboxController operatorxbox = new XboxController(1);
  TalonFX ShootLow = new TalonFX(17);
  TalonFX ShootHigh = new TalonFX(15);
  TalonFX intakeHigh = new TalonFX(21);
  TalonFX intakelow = new TalonFX(22);
  TalonFX transfer_1 = new TalonFX(24);
  TalonFX transfer_2 = new TalonFX(25);
  Transfer transfer = new Transfer(transfer_2, transfer_1);
  DoubleSolenoid psht = new DoubleSolenoid(17,PneumaticsModuleType.REVPH, 0, 1);
  LedSubsystem led = new LedSubsystem();
  
  LauncherSubsystem shoot = new LauncherSubsystem(ShootLow, ShootHigh,transfer);
  IntakeSubsystem intake = new IntakeSubsystem(intakelow, intakeHigh, psht);
  
  SendableChooser<Command> m_Chooser = new SendableChooser<>(); 
  


  

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    m_Chooser.addOption("SHORT: High Cube Auto", drivebase.getAutonomousCommand("Test"));
    m_Chooser.addOption("SHORT: Mid Cube Auto", drivebase.getAutonomousCommand("left"));
    m_Chooser.addOption("CENTER: High Cube + Balance Auto", drivebase.getAutonomousCommand("right"));
    m_Chooser.addOption("LONG: High Cube Auto", drivebase.getAutonomousCommand("center"));
    SmartDashboard.putData(m_Chooser);
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox::getYButtonPressed,
                                                                   driverXbox::getAButtonPressed,
                                                                   driverXbox::getXButtonPressed,
                                                                   driverXbox::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    JoystickButton controllerA = new JoystickButton(driverXbox, XboxController.Button.kA.value);
    JoystickButton controlerY = new JoystickButton(driverXbox,XboxController.Button.kY.value );
    JoystickButton controllerA_2 = new JoystickButton(operatorxbox, XboxController.Button.kA.value);
    JoystickButton controlerY_2 = new JoystickButton(operatorxbox,XboxController.Button.kY.value );
    JoystickButton controllerB = new JoystickButton(driverXbox, XboxController.Button.kB.value);
    JoystickButton controllerX = new JoystickButton(driverXbox, XboxController.Button.kX.value);
    JoystickButton controllerStart = new JoystickButton(driverXbox, XboxController.Button.kStart.value);
    

    controlerY.onFalse(new InstantCommand(() ->shoot.moveLauncher(0, 0)));
    controlerY.onTrue(new InstantCommand(() ->shoot.moveLauncher(-1, 1)));
    controllerA.onTrue(new Intake(intake));
    // controllerA.onTrue(new InstantCommand(() ->intake.moveIntake(-1.0, -1.0,true)));
    // controllerA.onFalse(new InstantCommand(() ->intake.moveIntake(0, 0,true)));
    // controllerA.onTrue(new InstantCommand(() -> Led.setBrightness(1.0)));
    controllerA.onFalse(new InstantCommand(() ->led.setAnimation()));
    
    
    //controllerA.onFalse(new InstantCommand(() ->intake.moveSolenoid(true)));
    

    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    new JoystickButton(driverXbox,
                       2).whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return m_Chooser.getSelected();
  }

  public void setDriveMode()
  
  
  
  
  
  
  
  
  
  
  
  
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
