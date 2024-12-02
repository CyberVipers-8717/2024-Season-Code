// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.AutoUptake;
import frc.robot.commands.RunClimb;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunUptake;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UptakeSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  //initializing all subsystems for robot
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final UptakeSubsystem m_uptake = new UptakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimbSubsystem m_climb = new ClimbSubsystem();
  private final LimelightSubsystem m_frontLimeLight = new LimelightSubsystem("limelight-front");
  private final LimelightSubsystem m_backLimeLight = new LimelightSubsystem("limelight-back");

  //controllers for driver and manipulator
  Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  Joystick m_manipulatorController = new Joystick(ManipulatorConstants.kManipulatorControllerPort);

  private SendableChooser<Command> autoChooser = new SendableChooser<>(); 

  public RobotContainer() {

    //registers commands to use in path planner 
    NamedCommands.registerCommand("intake", new AutoIntake(m_intake, .65)); 
    NamedCommands.registerCommand("uptake", new AutoUptake(m_uptake, .25));
    NamedCommands.registerCommand("shooter", new AutoShooter(m_shooter, .65));
   
    //binds controller buttons to commands
    configureButtonBindings();

    //creates dropdown to select autos 
    autoChooser = AutoBuilder.buildAutoChooser(); 

    // default command for drive subsystem
    m_robotDrive.setDefaultCommand(
        new RunCommand( 
            () -> m_robotDrive.drive(
                squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(OperatorConstants.kLeftYAxisPort), .08)), 
                squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(OperatorConstants.kLeftXAxisPort), .08)),
                squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(OperatorConstants.kRightXAxisPort),.08)), 
                false, true), //turn to false to get rid of slew rate limiter
            m_robotDrive));

    
    //configures limelight pipleline at start (0 is apriltag pipeline)
    m_frontLimeLight.setPipeline(0);
    m_backLimeLight.setPipeline(0);

    //Publishes data on SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData(m_robotDrive.m_gyro);
    SmartDashboard.putData("Field", m_robotDrive.m_field); 
  }

  //simple square function to allow for smoother and more precise driving
  //can modify this to better suit the drivers liking 
  private double squared(double value) {
    if(value >= 0) {
      return value*value; 
    } else {
      return -(value*value);
    }
  }

  private void configureButtonBindings() {
    // driver controls for climb
    Trigger driverLeftShoulder = new JoystickButton(m_driverController, OperatorConstants.kDriverLeftShoulder);
    Trigger driverRightShoulder = new JoystickButton(m_driverController, OperatorConstants.kDriverRightShoulder);

    //gyro reset button
    Trigger driverBButton = new JoystickButton(m_driverController, OperatorConstants.kDriverBButton);

    //Shooter buttons
    Trigger manipulatorLeftShoulder = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorLeftShoulder);
    Trigger manipulatorRightShoulder = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorRightShoulder);
    Trigger manipulatorRightTrigger = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorRightTrigger);
    Trigger manipulatorBButton = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorBButton);
    Trigger manipulatorAButton = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorAButton);
    Trigger manipulatorXButton = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorXButton);

    //binding buttons to commands
    manipulatorLeftShoulder.whileTrue(Commands.parallel(new RunIntake(m_intake, -.25), new RunUptake(m_uptake, -.25), new RunShooter(m_shooter, -.5))); //Flushes everything backwards
    manipulatorRightShoulder.whileTrue(new RunUptake(m_uptake, .35)); // pops a note into shooter
    manipulatorRightTrigger.whileTrue(Commands.parallel(new RunIntake(m_intake, .5), new RunUptake(m_uptake, .25))); // runs intake and shooter and stops aftera note hits the uptake
    manipulatorBButton.whileTrue(new RunShooter(m_shooter, .155)); //amp
    manipulatorAButton.whileTrue(new RunShooter(m_shooter, .35)); //trap 
    manipulatorXButton.whileTrue(new RunShooter(m_shooter, .70)); //speaker

    driverLeftShoulder.whileTrue(new RunClimb(m_climb, -1)); //down 
    driverRightShoulder.whileTrue(new RunClimb(m_climb, 1)); //up 

    //resets the forward direction of the gyro
    driverBButton.onTrue(m_robotDrive.resetGyro());
  }

  //retrieves command to be scheduled during auto 
  public Command getAutonomousCommand() {
    return autoChooser.getSelected(); 
  }

}
