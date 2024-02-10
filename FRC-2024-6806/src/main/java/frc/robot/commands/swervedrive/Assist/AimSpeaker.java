package frc.robot.commands.swervedrive.Assist;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.swervedrive.Assist.Aimtag;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.swervedrive.Assist.AimShoot;




public class AimSpeaker extends SequentialCommandGroup {
    

    private final SwerveSubsystem swerveSubsystem;
    private final Vision vision;
    
    public AimSpeaker(SwerveSubsystem swerveSubsystem,Vision vision){
        
        this.swerveSubsystem = swerveSubsystem;
        this.vision = vision;
        addRequirements(this.swerveSubsystem);
        addRequirements(this.vision);
        addCommands(
            new InstantCommand(() -> new AimShoot(swerveSubsystem, vision)),
            new InstantCommand(() -> new Aimtag(swerveSubsystem, vision,1))
        );
    }
    
}
