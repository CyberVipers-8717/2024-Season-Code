package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UptakeSubsystem;

public class AutoUptake extends Command {
    private final UptakeSubsystem m_uptake; 
    private final double speed; 
    private final Timer waitTimer = new Timer(); 

    //allows variable speeds 
    public AutoUptake(UptakeSubsystem uptakeSubsystem, double speed) {
        m_uptake = uptakeSubsystem; 
        this.speed = speed; 
        addRequirements(m_uptake);
    }

    @Override 
    //must reset and restart timer to work as expected 
    public void initialize() {
        waitTimer.reset();
        waitTimer.start();
        m_uptake.zeroEncoder();
    }

    @Override 
    public void execute() {
        m_uptake.setMotor(speed);
    }

    @Override 
    //Turns off motor and stops timer when command ends
    public void end(boolean interrupted) {
        m_uptake.setMotor(0);
        waitTimer.stop(); 

    }

    @Override 
    //finishes command after a certain amount of time
    public boolean isFinished() {
        if(waitTimer.get() < .6) {
            return false; 
        } else {
            return true;
        }
    }
    
}
