package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntake extends Command {
    private final IntakeSubsystem m_intake; 
    private final double speed; 
    private final Timer waitTimer = new Timer(); 

    //allows variable speeds 
    public AutoIntake(IntakeSubsystem intakeSubsystem, double speed) { 
        m_intake = intakeSubsystem; 
        this.speed = speed; 
        addRequirements(m_intake);
    }

    @Override 
    //must reset and restart timer to work as expected 
    public void initialize() {
        waitTimer.restart();
        waitTimer.start(); 
        m_intake.zeroEncoders();
    }

    @Override 
    public void execute() {
        m_intake.setMotors(speed);
    }

    @Override
    //Turns off motors and stops timer when command ends 
    public void end(boolean interrupted) {
        m_intake.setMotors(0);
        waitTimer.stop();  
    }

    @Override 
    //finishes command after a certain amount of time
    public boolean isFinished() {
        if (waitTimer.get() < 1) { 
            return false; 
        } else {
            return true; 
        }
    }
    
}
