// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutoPathCommands.FourBallAuto;
import frc.robot.AutoPathCommands.Swerve2;
import frc.robot.AutoPathCommands.SemiCircle;
import frc.robot.AutoPathCommands.MoveForward;
import frc.robot.AutoPathCommands.Center5Ball;
import frc.robot.Autonomous.TwoBallAuto;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.control.XBoxControllerDPad;
import frc.robot.control.XboxControllerButton;
import frc.robot.control.XboxControllerEE;
import frc.robot.Feedback.Cameras.Limelight;
import frc.robot.subsystems.Climber.A0_CalibrateClimber;
import frc.robot.subsystems.Climber.A5_LiftAndReach;
import frc.robot.subsystems.Climber.A1_PrepareToClimb;
import frc.robot.subsystems.Climber.A2_LiftToBar;
import frc.robot.subsystems.Climber.A3_ReachToNextBar;
import frc.robot.subsystems.Climber.A4_HookToNextBar;
import frc.robot.subsystems.Climber.A7_HookAndReach;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.MagicClimbByDash;
import frc.robot.subsystems.Climber.MagicClimbByStick;
import frc.robot.subsystems.Climber.ManualClimbByStick;
import frc.robot.subsystems.drive.BearSwerveHelper;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.Intake.DriveIntake;
//import frc.robot.subsystems.Intake.IntakeOverride;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.AutoShoot;
import frc.robot.subsystems.Shooter.ShootLowerHub;
import frc.robot.subsystems.Shooter.ShootUpperHub;
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
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final ShooterSubsystem m_shooterSubystem = new ShooterSubsystem();
  private final TowerSubsystem m_towerSubsystem = new TowerSubsystem();

  private final XboxControllerEE m_driverController = new XboxControllerEE(0);
  private final XboxControllerEE m_operatorController = new XboxControllerEE(1);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    
    new Pneumactics();
  
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(DriveConstants.PRACTICE);
	
	// Setup the Swerve Subsystem (w/ simulation support)
  	dt = BearSwerveHelper.createBearSwerve();
    m_swerveSubsystem = BearSwerveHelper.createSwerveSubsystem(dt);

    // Populate Auto Chooser
    Shuffleboard.getTab("Driver Dash").add("Auto Chooser", m_chooser);
    m_chooser.setDefaultOption("Two Ball Auto", new TwoBallAuto(m_swerveSubsystem, m_shooterSubystem, m_towerSubsystem, m_intakeSubsystem));
    m_chooser.addOption("5BallAuto", new Center5Ball(m_swerveSubsystem));
	  m_chooser.addOption("Move Forward", new MoveForward(m_swerveSubsystem));
    m_chooser.addOption("Semi-Circle", new SemiCircle(m_swerveSubsystem));
    m_chooser.addOption("Swerve2", new Swerve2(m_swerveSubsystem));
    m_chooser.addOption("4BallAuto", new FourBallAuto(m_swerveSubsystem));

    Limelight.initialize();
    Limelight.setDriverMode();

    // Configure the button bindings
    configureButtonBindings();

    // For simulation
    SmartDashboard.putData("Auto Chooser", m_chooser);

	  // Set up the default commands for the various subsystems
    m_swerveSubsystem.setDefaultCommand(new SwerveDrive(m_swerveSubsystem,
                                      () -> -(m_driverController.getLeftY()),
                                      () -> -(m_driverController.getLeftX()),
                                      () -> (m_driverController.getRightX())));
    
    m_intakeSubsystem.setDefaultCommand(new DriveIntake(m_intakeSubsystem,
                                        () -> (m_driverController.getRightTriggerAxis()),  
                                        () -> (m_driverController.getLeftTriggerAxis())));

    m_towerSubsystem.setDefaultCommand(new DriveTower(m_towerSubsystem,  
                                      () -> -m_operatorController.getLeftY()));

    m_climberSubsystem.setDefaultCommand(new ManualClimbByStick(m_climberSubsystem, 
                                        () -> m_operatorController.getRightY()));

    // Make the Climb Sequence commands available on SmartDash
    SmartDashboard.putData(new A0_CalibrateClimber(m_climberSubsystem));
    SmartDashboard.putData(new A1_PrepareToClimb(m_climberSubsystem, m_intakeSubsystem));
    SmartDashboard.putData(new A2_LiftToBar(m_climberSubsystem));
    SmartDashboard.putData(new A3_ReachToNextBar(m_climberSubsystem));
    SmartDashboard.putData(new A4_HookToNextBar(m_climberSubsystem));
    SmartDashboard.putData(new A5_LiftAndReach(m_climberSubsystem));
    SmartDashboard.putData(new A7_HookAndReach(m_climberSubsystem));
        
    // Climber Arm Driving Commands
    SmartDashboard.putData(new ManualClimbByStick(m_climberSubsystem, () -> (-1.0)*m_operatorController.getRightY()));
    SmartDashboard.putData(new MagicClimbByStick(m_climberSubsystem, () -> (-1.0)*m_operatorController.getRightY()));
    SmartDashboard.putData(new MagicClimbByDash(m_climberSubsystem));

    // Zero Climber Sensors
    SmartDashboard.putData(new InstantCommand(m_climberSubsystem::zeroSensors, m_climberSubsystem));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Driver Controller
    // new XboxControllerButton(m_driverController, XboxControllerEE.Button.kLeftBumper)
    // .whenPressed(new InstantCommand(m_intakeSubsystem::intakeDeploy, m_intakeSubsystem));
    
    // new XboxControllerButton(m_driverController, XboxControllerEE.Button.kRightBumper)
    // .whenPressed(new InstantCommand(m_intakeSubsystem::intakeRetract, m_intakeSubsystem));
    
    //new XboxControllerButton(m_driverController, XboxControllerEE.Button.kLeftBumper)
    //.whileHeld(new IntakeOverride(m_intakeSubsystem, true));
    
    //new XboxControllerButton(m_driverController, XboxControllerEE.Button.kRightBumper)
    //.whileHeld(new IntakeOverride(m_intakeSubsystem, false));

    // Back button zeros the gyroscope
    new XboxControllerButton(m_driverController, XboxControllerEE.Button.kBack)
        .whenPressed(m_swerveSubsystem.dt::zeroGyroscope);

    //Operator controller    
    new XboxControllerButton(m_operatorController, XboxControllerEE.Button.kA)
    .whenHeld(new ShootLowerHub(m_shooterSubystem, m_towerSubsystem));

    new XboxControllerButton(m_operatorController, XboxControllerEE.Button.kB)
    .whenHeld(new ShootUpperHub(m_shooterSubystem, m_towerSubsystem));

    new XboxControllerButton(m_operatorController, XboxControllerEE.Button.kX)
    .whenHeld(new AutoShoot(m_shooterSubystem, m_towerSubsystem, ShooterConstants.upperHubVelocity));

    new XboxControllerButton(m_operatorController, XboxControllerEE.Button.kLeftBumper)
    .whileActiveContinuous(new InstantCommand(m_shooterSubystem::deployHood, m_shooterSubystem));
    
    new XboxControllerButton(m_operatorController, XboxControllerEE.Button.kRightBumper)
    .whileActiveContinuous(new InstantCommand(m_shooterSubystem::retractHood, m_shooterSubystem));

    new XboxControllerButton(m_operatorController, XboxControllerEE.Button.kA)
    .whenPressed(new InstantCommand(m_climberSubsystem::zeroSensors));
        
    new XBoxControllerDPad(m_operatorController, XboxControllerEE.DPad.kDPadUp)
    .whileActiveContinuous(new InstantCommand(m_climberSubsystem::fixedClimberVertical));
    
    new XBoxControllerDPad(m_operatorController, XboxControllerEE.DPad.kDPadDown)
    .whileActiveContinuous(new InstantCommand(m_climberSubsystem::fixedClimberAngled));

    new XBoxControllerDPad(m_operatorController, XboxControllerEE.DPad.kDPadLeft)
    .whileActiveContinuous(new InstantCommand(m_climberSubsystem::extendingClimberAngled));
    
    new XBoxControllerDPad(m_operatorController, XboxControllerEE.DPad.kDPadRight)
    .whileActiveContinuous(new InstantCommand(m_climberSubsystem::extendingClimberVertical));
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
