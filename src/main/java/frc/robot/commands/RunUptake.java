package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UptakeSubsystem;

public class RunUptake extends Command {
    private final UptakeSubsystem m_uptake; 
    private final double speed;

    //defaults to full speed
    public RunUptake(UptakeSubsystem uptakeSubsystem) {
        m_uptake = uptakeSubsystem;
        speed = 1;  
        addRequirements(m_uptake);
    }

    //allows variable speeds 
    public RunUptake(UptakeSubsystem uptakeSubsystem, double speed) { 
        m_uptake = uptakeSubsystem; 
        this.speed = speed; 
        addRequirements(m_uptake);
    }

    @Override 
    public void initialize() {
        m_uptake.zeroEncoder();
    }

    @Override 
    public void execute() {
        m_uptake.setMotor(speed);
    }

    @Override 
    //Turns the motor off when command ends
    public void end(boolean interrupted) {
        m_uptake.setMotor(0);

    }

    @Override 
    //false: runs continuously, true: runs once and stops
    public boolean isFinished() { 
       return false; 
    }
    
}
