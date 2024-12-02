package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UptakeConstants;

public class UptakeSubsystem extends SubsystemBase {

    private final CANSparkMax m_uptakeMotor;

    private final RelativeEncoder m_uptakeEncoder;

    public UptakeSubsystem() {
        m_uptakeMotor = new CANSparkMax(UptakeConstants.kUptakeMotorPort, MotorType.kBrushless); 

        m_uptakeEncoder = m_uptakeMotor.getEncoder(); 

        m_uptakeMotor.setIdleMode(IdleMode.kBrake); 

        m_uptakeMotor.setSmartCurrentLimit(UptakeConstants.kUptakeCurrentLimit); 

        m_uptakeMotor.burnFlash(); 

        zeroEncoder(); 
    }
    
    //resets the Encoders
    public void zeroEncoder() {
        m_uptakeEncoder.setPosition(0);
    }

    //moves the motor
    public void setMotor(double speed) {
        m_uptakeMotor.set(speed);
    }

    
}
