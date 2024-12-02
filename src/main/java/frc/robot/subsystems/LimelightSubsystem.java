package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// (UNFINISHED)
// Simple limelight wrapper class to make it better fit the command-based paradigm 
public class LimelightSubsystem extends SubsystemBase {
    
    //database to publish limelight data
    private final NetworkTable table; 

    public LimelightSubsystem(String limelightName) {
        table = NetworkTableInstance.getDefault().getTable(limelightName);
    }

    //returns the horizontal offset of the target
    public double getTX() {
        return table.getEntry("tx").getDouble(0.0);
    }

    //returns the vertical offest of the target
    public double getTY() {
        return table.getEntry("ty").getDouble(0.0);
    }
    
    //returns if a target has been detected
    public double getTV() {
        return table.getEntry("tv").getDouble(0.0);
    }

    //returns the 3d transform of the april tag in camera space
    public double[] get3dTargetPoseArray() {
        return table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); 
    }

    //gets the current limelight pipeline
    public double getPipeline() {
        return table.getEntry("getpipe").getDouble(0.0);
    }

    //sets the current limelight pipeline 
    public void setPipeline(double pipelineIndex) {
        table.getEntry("pipeline").setDouble(pipelineIndex);
    }

    //gets the current ledMode (on/off/flashing)
    public double getLedMode() {
        return table.getEntry("ledMode").getDouble(getLedMode());
    }

    //sets the current ledMode
    public void setLedMode(double ledIndex) {
        table.getEntry("ledMode").setDouble(ledIndex);
    }

    //gets the id of the april tag in view
    public double[] getAprilTagID() {
        return table.getEntry("tid").getDoubleArray(new double[6]);
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
