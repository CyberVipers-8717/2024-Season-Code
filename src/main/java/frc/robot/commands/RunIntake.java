package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends Command {
    private final IntakeSubsystem m_intake; 
    private final double speed; 

    //defaults to full speed
    public RunIntake(IntakeSubsystem intakeSubsystem) { 
        m_intake = intakeSubsystem; 
        speed = 1; 
        addRequirements(m_intake);
    }

    //allows variable speeds 
    public RunIntake(IntakeSubsystem intakeSubsystem, double speed) { 
        m_intake = intakeSubsystem; 
        this.speed = speed; 
        addRequirements(m_intake);
    }

    @Override 
    public void initialize() {
        m_intake.zeroEncoders();
    }

    @Override 
    public void execute() {
        m_intake.setMotors(speed);
    }

    @Override 
    public void end(boolean interrupted) {
        m_intake.setMotors(0);
    }

    @Override 
    //false: runs continuously, true: runs once and stops
    public boolean isFinished() { 
        return false;
    }
    
}
