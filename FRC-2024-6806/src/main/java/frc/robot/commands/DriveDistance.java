public class DriveDistance extends CommandBase {    
    private SwerveDrive  s_Swerve; 
    
    private double currentstate;
    private double initialstate;
    private double desiredState;
    private double speed;
    private boolean didThing = false;
    
    public DriveDistance(double desiredstateFeet, SwerveDrive s_Swerve, double speed) {
        
        this.s_Swerve = s_Swerve;
        this.desiredState = desiredstateFeet;//*14.7;//15.4839;
        // this.desiredState = desiredstateFeet; 
        initialstate = s_Swerve.getPose2D().getX();

        if( desiredState < 0 ){
            this.speed = -speed;
        } else {
            this.speed = speed;
        }

        addRequirements(s_Swerve);
      
    }

public void intitialize(){
   initialstate = (s_Swerve.getPose2D().getX());// + s_Swerve.mSwerveMods[1].getPosition().distanceMeters + s_Swerve.mSwerveMods[2].getPosition().distanceMeters + s_Swerve.mSwerveMods[3].getPosition().distanceMeters)/4.0;
   currentstate = (s_Swerve.getPose2D().getX()); //+ s_Swerve.mSwerveMods[1].getPosition().distanceMeters + s_Swerve.mSwerveMods[2].getPosition().distanceMeters + s_Swerve.mSwerveMods[3].getPosition().distanceMeters)/4.0;          

}

public void end(boolean interupted){
       swerveDrive.drive(0, 0,0,true,false);  
}

public boolean isFinished(){
    if(desiredState < 0){

        if (currentstate < desiredState - initialstate) {
            didThing = false;
        }
        
        return currentstate < desiredState + initialstate;
        
    }else if(desiredState >= 0){

        if (currentstate > initialstate + desiredState) {
            didThing = false;
        }
        
        return currentstate > initialstate + desiredState;
        
    } else {
        return true;
    }
 

}

    @Override
    public void execute() {

        
 
       swerveDrive.drive(new Translation2d(Math.pow(speed, 3) * swerveDrive.getMaximumVelocity(), 0),
                        0,
                        true,
                        false);
;  

        if (!didThing) {
            didThing = true;
            initialstate = s_Swerve.getPose2D().getX();

        
            desiredState = desiredState + Math.abs(initialstate);
            
            System.out.println(didThing);
        }
        currentstate = Math.abs((s_Swerve.getPose2D().getX()));//+ s_Swerve.mSwerveMods[1].getPosition().distanceMeters + s_Swerve.mSwerveMods[2].getPosition().distanceMeters + s_Swerve.mSwerveMods[3].getPosition().distanceMeters)/4.0;          
        SmartDashboard.putNumber("inital state", initialstate);
        SmartDashboard.putNumber("current state", currentstate);
        SmartDashboard.putNumber("desiredState", desiredState);
        
    }
}
