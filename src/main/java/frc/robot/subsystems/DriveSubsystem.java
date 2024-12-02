// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class DriveSubsystem extends SubsystemBase {

  //default constructor for drive subsystem
  public DriveSubsystem() {

    //configures path planner to work with our drivetrain
    AutoBuilder.configureHolonomic(
            this::getPose, 
            (pose) -> {resetPose(pose); m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getPositions(), pose);}, //resets pose and then updates the field 2d with the starting pose
            this::getCurrentSpeeds, 
            this::autoDrive,
            new HolonomicPathFollowerConfig( 
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.3, // Max module speed, in m/s
                    0.33, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
              
              var alliance = DriverStation.getAlliance(); //if not connected to FMS will default to whatever alliance is selected on the drive station
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    
  }

  //configuring the swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort, DriveConstants.kFrontLeftTurnMotorPort, -Math.PI/2); 
  private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort, DriveConstants.kFrontRightTurnMotorPort, 0); //maybe here? WB
  private final SwerveModule m_rearLeft = new SwerveModule(DriveConstants.kRearLeftDriveMotorPort, DriveConstants.kRearLeftTurnMotorPort, Math.PI); 
  private final SwerveModule m_rearRight = new SwerveModule(DriveConstants.kRearRightDriveMotorPort, DriveConstants.kRearRightTurnMotorPort, Math.PI/2); 

  private final SwerveModulePosition[] m_swervePositions = getPositions();
  
  //gyro to find robot's heading/angle 
  public final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  //odometry object to track the robots pose on the field
  private final SwerveDriveOdometry m_driveOdometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(), m_swervePositions); 

  //poseEstimator object which tracks the robots pose on the field (similar to odometry object but can use vision data for more accurate tracking)
  private final SwerveDrivePoseEstimator m_poseEstimator =
    new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        m_gyro.getRotation2d(),
        getPositions(),
        new Pose2d(), //below - 1x3 matrix of the form (x,y,theta) in meters and radians
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), //stddev of state-space control pose estimate (wpilib) //increase values if you want to trust this estimate less
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); //stddev of vision pose estimate (limelight) //increase values if you want to trust this estimate less

  //Field widgit for Shuffleboard
  public final Field2d m_field = new Field2d();

  //publishes data for visualization using advantagescope 
  private final StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
  
  //SlewRate variables
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  
  //default drive method that converts controller input into field Relative robot movement (can toggle fieldRelative and rateLimiter)
  public void drive(double xSpeed, double ySpeed, double arr, boolean rateLimit, boolean fieldRelative){
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(arr);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = arr;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    //generates array of swerve module states from controller input
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(fieldRelative ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, m_gyro.getRotation2d()): new ChassisSpeeds(xSpeedDelivered, ySpeedCommanded, rotDelivered), DriveConstants.kDriverPeriod));

    //caps wheel speeds to ensure they don't go faster than they're allowed to 
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    //sends the swerve module states to the pid controllers 
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  //simple auto drive method for path following (path planner requires a driv emethod that takes ChassisSpeeds)
  public void autoDrive(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(
    speeds, DriveConstants.kDriverPeriod
    ));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  //gets the estimated robot pose from the odometry object (prone to drift)
  public Pose2d getPose(){
    return m_driveOdometry.getPoseMeters();
  }
  
  //resets the pose of the odometry to the pose supplied 
  public void resetPose(Pose2d pose){
    m_driveOdometry.resetPosition(m_gyro.getRotation2d(), getPositions(), pose);
  }

  //gets the current positions of the swerve modules 
  //at the start this will return zero for all modules 
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()};
  }

  //gets the current state of the swerve modules
  //at the start this should be 0 m/s and the Rotation2d representing the current angle of the wheels 
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()};
  }

  //gets the chassis speed represeting the speed the wheels are moving in
  //at the start this should be zero 
  public ChassisSpeeds getCurrentSpeeds() {
      return DriveConstants.kDriveKinematics.toChassisSpeeds(getStates());
  }

  @Override
  public void periodic() {
    //updates data on advantagescope
    publisher.set(getStates()); 
    //updates the swerve odometry and pose Estimator every clock cycle
    updateOdometryPose();
    //publishes the estimated robot position to the Field widgit for visualization
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //simple command to reset the gyro incase it gets out of alignment
  public Command resetGyro() {
    return new InstantCommand(() -> {m_gyro.reset();}, this);
  }

  //Updates the odometry and poseEstimator  
  //Can use the Limelight to more accurately Estimate the Pose
  public void updateOdometryPose() {
    m_driveOdometry.update(m_gyro.getRotation2d(), getPositions());
    m_poseEstimator.update(m_gyro.getRotation2d(), getPositions()); 
    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight-back", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
      if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }
}
