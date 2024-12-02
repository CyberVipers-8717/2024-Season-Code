package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShooter extends Command {
    private final ShooterSubsystem m_shooter;
    private final double speed;
    private final Timer waitTimer = new Timer(); 

    //allows variable speeds 
    public AutoShooter(ShooterSubsystem shooterSubsystem, double speed){
        m_shooter = shooterSubsystem;
        this.speed = speed;
        addRequirements(m_shooter);
    }

    @Override 
    //must reset and restart timer to work as expected 
    public void initialize() {
        waitTimer.reset();
        waitTimer.start(); 
        m_shooter.zeroEncoders();
    }

    @Override 
    public void execute() {
        m_shooter.setMotors(speed);
    }

    @Override 
    //Turns off motors and stops timer when command ends
    public void end(boolean interrupted) {
        m_shooter.setMotors(0);
        waitTimer.stop(); 
    }

    @Override 
    //finishes command after a certain amount of time
    public boolean isFinished() { 
        if (waitTimer.get() < 2) {
            return false; 
        } else {
            return true; 
        }
    }
}
