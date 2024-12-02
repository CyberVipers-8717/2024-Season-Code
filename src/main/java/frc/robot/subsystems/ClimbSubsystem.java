package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase{
   
    private final CANSparkMax m_climb;

    private final RelativeEncoder m_climbEncoder;

    public ClimbSubsystem(){
        m_climb = new CANSparkMax(ClimbConstants.kClimbMotorPort, MotorType.kBrushless);

        m_climbEncoder = m_climb.getEncoder();

        m_climb.setSmartCurrentLimit(ClimbConstants.kClimbMotorCurrentLimit); 
        m_climb.setIdleMode(IdleMode.kBrake); 

        m_climb.setInverted(false);
    
        m_climb.burnFlash();
        zeroEncoder();
    }

    //Reset Encoders
    public void zeroEncoder() {
        m_climbEncoder.setPosition(0);
    }

    //moves th eclimb motor
    public void setMotor(double speed){
        m_climb.set(speed);
    }

    //gets the current position of the climb in rotations
    public double getPosition() {
        return m_climbEncoder.getPosition(); 
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}