// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutoPathCommands.Center5Ball;
import frc.robot.AutoPathCommands.FourBallAuto;
import frc.robot.AutoPathCommands.MoveForward;
import frc.robot.AutoPathCommands.SemiCircle;
import frc.robot.AutoPathCommands.Swerve2;
import frc.robot.Autonomous.TestAuto;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Control.XBoxControllerDPad;
import frc.robot.Control.XboxControllerButton;
import frc.robot.Control.XboxControllerEE;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ExtendClimber;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.SwerveDrive;
import frc.robot.subsystems.Intake.DriveIntake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterCommand;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Tower.DriveTower;
import frc.robot.subsystems.Tower.TowerSubsystem;

import frc.swervelib.SwerveDrivetrainModel;
import frc.swervelib.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  private static SwerveDrivetrainModel dt;
  private static SwerveSubsystem m_swerveSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final ShooterSubsystem m_shooterSubystem = new ShooterSubsystem();
  private final TowerSubsystem m_towerSubsystem = new TowerSubsystem();

  private final XboxControllerEE m_driverController = new XboxControllerEE(0);
  private final XboxControllerEE m_operatorController = new XboxControllerEE(1);

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    
  
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(DriveConstants.PRACTICE);
	
	// Setup the Swerve Subsystem (w/ simulation support)
  	dt = BearSwerveHelper.createBearSwerve();
    m_swerveSubsystem = BearSwerveHelper.createSwerveSubsystem(dt);
    
    // Setup a DriveSubsystem to assist with driving the swerve like a tank
    m_driveSubsystem = new DriveSubsystem(m_swerveSubsystem);

    // Populate Auto Chooser
    Shuffleboard.getTab("Driver Dash").add("Auto Chooser", m_chooser);
    m_chooser.setDefaultOption("5BallAuto", new Center5Ball(m_swerveSubsystem));
	  m_chooser.addOption("Move Forward", new MoveForward(m_swerveSubsystem));
    m_chooser.addOption("Semi-Circle", new SemiCircle(m_swerveSubsystem));
    m_chooser.addOption("Swerve2", new Swerve2(m_swerveSubsystem));
    m_chooser.addOption("4BallAuto", new FourBallAuto(m_swerveSubsystem));
    m_chooser.addOption("Test Auto", new TestAuto(m_driveSubsystem));

    SmartDashboard.putNumber("Pressure", phCompressor.getPressure());
    
    // For simulation
    SmartDashboard.putData("Auto Chooser", m_chooser);

	// Set up the default commands for the various subsystems
    m_swerveSubsystem.setDefaultCommand(new SwerveDrive(m_swerveSubsystem,
                                      () -> -(m_driverController.getLeftY()),
                                      () -> -(m_driverController.getLeftX()),
                                      () -> -(m_driverController.getRightX())));
    
    m_intakeSubsystem.setDefaultCommand(new DriveIntake(m_intakeSubsystem,
                                        () -> -(m_driverController.getRightTriggerAxis()),  
                                        () -> -(m_driverController.getLeftTriggerAxis())));

    m_climberSubsystem.setDefaultCommand(new ExtendClimber(m_climberSubsystem, 
                                        () -> m_operatorController.getRightY()));

    m_towerSubsystem.setDefaultCommand(new DriveTower(m_towerSubsystem,  
                                      () -> m_operatorController.getLeftY()));
									  
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new XboxControllerButton(m_driverController, XboxControllerEE.Button.kBack)
        .whenPressed(m_swerveSubsystem.dt::zeroGyroscope);

    new XBoxControllerDPad(m_operatorController, XboxControllerEE.DPad.kDPadUp)
    .whileActiveContinuous(new InstantCommand(m_climberSubsystem::climberForward, m_climberSubsystem));
    
    new XBoxControllerDPad(m_operatorController, XboxControllerEE.DPad.kDPadDown)
    .whileActiveContinuous(new InstantCommand(m_climberSubsystem::climberReverse, m_climberSubsystem));

    new XboxControllerButton(m_driverController, XboxControllerEE.Button.kLeftBumper)
    .whenPressed(new InstantCommand(m_intakeSubsystem::intakeIn, m_intakeSubsystem));
    
    new XboxControllerButton(m_driverController, XboxControllerEE.Button.kRightBumper)
    .whenPressed(new InstantCommand(m_intakeSubsystem::intakeOut, m_intakeSubsystem));

    new XboxControllerButton(m_operatorController, XboxControllerEE.Button.kA)
    .whenHeld(new ShooterCommand(ShooterConstants.testSpeed, m_shooterSubystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
