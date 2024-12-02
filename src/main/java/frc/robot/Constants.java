// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveConstants { 
    public static final double kMaxSpeedMetersPerSecond = 1.8; //4.3 
    public static final double kMaxAccelerationMetersPerSecond = 4.3; //theoretical max
    public static final double kMaxAngularSpeed = 2 * Math.PI; //radians per sec 
    public static final double kMaxAngularAcceleration = 2 * Math.PI; // radians per second per second

    //Slew Rate helps Stop Wheel Failure 
    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    //distance from center of left wheel to right wheel
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    //distance from center of front wheel to back wheel
    public static final double kTrackLength = Units.inchesToMeters(22.5);

    //Drive Motor Ports 
    public static final int kFrontLeftDriveMotorPort = 25;
    public static final int kFrontRightDriveMotorPort = 18;
    public static final int kRearLeftDriveMotorPort = 23;
    public static final int kRearRightDriveMotorPort = 21;

    public static final int kFrontLeftTurnMotorPort = 26;
    public static final int kFrontRightTurnMotorPort = 17;
    public static final int kRearLeftTurnMotorPort = 22;
    public static final int kRearRightTurnMotorPort = 19;

    //kinematics class that generates swerve module states from chassis Speeds
    public static final SwerveDriveKinematics kDriveKinematics =
    new SwerveDriveKinematics(
      new Translation2d(kTrackLength / 2, kTrackWidth / 2), //front left
      new Translation2d(kTrackLength / 2, -kTrackWidth / 2), //front right      
      new Translation2d(-kTrackLength / 2, kTrackWidth / 2), //rear left
      new Translation2d(-kTrackLength / 2, -kTrackWidth / 2)); //rear right

    //robot clock cycle
    public static final double kDriverPeriod = TimedRobot.kDefaultPeriod;
  }

  public static class OperatorConstants {
    //usb port for drive controller 
    public static final int kDriverControllerPort = 0;

    //important buttons/axis for driver controller 
    public static final int kLeftXAxisPort = 0;
    public static final int kLeftYAxisPort = 1;
    public static final int kRightXAxisPort = 2;

    public static final int kDriverXButton = 1; 
    public static final int kDriverAButton = 2; 
    public static final int kDriverBButton = 3; 
    public static final int kDriverYButton = 4; 
    public static final int kDriverLeftShoulder = 5;
    public static final int kDriverRightShoulder = 6; 
    public static final int kDriverLeftTrigger = 7; 
    public static final int kDriverRightTrigger = 8;   
  }

  public static class ManipulatorConstants {
    //usb port for manipulator controller
    public static final int kManipulatorControllerPort = 1;

    //important buttons/axis for manipulator controller
    public static final int kManipulatorXButton = 1; 
    public static final int kManipulatorAButton = 2; 
    public static final int kManipulatorBButton = 3; 
    public static final int kManiputatorYButton = 4; 
    public static final int kManipulatorLeftShoulder = 5;
    public static final int kManipulatorRightShoulder = 6; 
    public static final int kManipulatorLeftTrigger = 7; 
    public static final int kManipulatorRightTrigger = 8;  
  }


  public static class SwerveModuleConstants {
    //note: everything commented out has to do with rev pid controllers
    public static final double kWheelDiameterMeters = 0.0762; 
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDriveMotorReduction = (45.0 * 22) / (13 * 15);
    public static final double kDriveMotorFreeSpeedRps = 5676 / 60;
    public static final double kDriveWheelFreeSpeedRps = (kDriveMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDriveMotorReduction;
    public static final double kDriveEncoderPositionFactor = kWheelCircumferenceMeters / kDriveMotorReduction; // meters
    public static final double kDriveEncoderVelocityFactor = (kWheelCircumferenceMeters / kDriveMotorReduction) / 60.0; // meters per second

    public static final double kTurnEncoderPositionFactor = 2 * Math.PI; //radians 
    public static final double kTurnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final boolean kTurnEncoderInverted = true;
    public static final boolean kTurnEncoderWrapping = true;

    public static final double kTurnEncoderPositionPIDMinInput = 0; 
    public static final double kTurnEncoderPositionPIDMaxInput = 2 * Math.PI; 

    //gains for drive motor PID (defaults)
    public static final double kDriveP = .04; 
    public static final double kDriveI = 0; 
    public static final double kDriveD = 0; 
    public static final double kDriveFF = 1 / kDriveWheelFreeSpeedRps; 
    public static final double kDriveMinOutput = -1; 
    public static final double kDriveMaxOutout = 1; 

    //gains for turn motor PID (defaults)
    public static final double kTurnP = 1; 
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kTurnFF = 0;
    public static final double kTurnMinOutput = -1;
    public static final double kTurnMaxOutput = 1;

    //current limits for swerve motors
    public static final int kDriveCurrentLimit = 40;
    public static final int kTurnCurrentLimit = 20;

  }

  public static class IntakeConstants {
    //motor ports for intake
    public static final int kLowIntakeMotorPort = 16; 
    public static final int kHighIntakeMotorPort = 6; 
    //current limit for intake motors
    public static final int kIntakeCurrentLimit = 40; 
    public static final boolean kHighIntakeInverted = false; 
  }

  public static class UptakeConstants {
    //motor ports and current limit for uptake
    public static final int kUptakeMotorPort = 9;
    public static final int kUptakeCurrentLimit = 40; 
  }

  public static class ShooterConstants {
    //motor ports and current limit for shooter
    public static final int kLowShooterMotorPort = 10; 
    public static final int kHighShooterMotorPort = 13;
    public static final int kShooterMotorCurrentLimit = 40; 
  }
  
  public static class ClimbConstants {
    //motor port and current limit for climb
    public static final int kClimbMotorPort = 2; 
    public static final int kClimbMotorCurrentLimit = 40; 
  }
}
