package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class RunClimb extends Command {
    private final ClimbSubsystem m_climb;  
    private double speed;  
    
    //variable speed
    public RunClimb(ClimbSubsystem climbSubsystem, double speed) {
        m_climb = climbSubsystem;
        this.speed = speed;
        addRequirements(m_climb);
    }

    @Override 
    //shouldn't zero encoders because we use them to know when to stop 
    public void initialize() {
    }

    @Override 
    public void execute() {
        m_climb.setMotor(speed);
    }

    @Override 
    //turns off motor when command ends
    public void end(boolean interrupted) {
        m_climb.setMotor(0);
    }

    @Override 
    //false: runs continuously, true: runs once and stops
    public boolean isFinished() { 
        if (speed < 0 && m_climb.getPosition() <= 0 ) { //going down and hit bottom
            return true;     
        } else if (speed > 0 && m_climb.getPosition() >= 100) { //going up and hit top
            return true; 
        } else {
            return false; 
        }
    }
    
}
