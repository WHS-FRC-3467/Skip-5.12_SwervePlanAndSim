// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autocommands.MoveForward;
import frc.robot.autocommands.SemiCircle;
import frc.robot.autocommands.Swerve1;
import frc.robot.control.XboxControllerButton;
import frc.robot.control.XboxControllerEE;
import frc.robot.subsystems.drive.SwerveDriveCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {
  private static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private static final XboxControllerEE m_driverController = new XboxControllerEE(0, 0.05);
  //private static final XboxControllerEE m_operatorController = new XboxControllerEE(1, 0.05);

  private static final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are defined in the command but use both sticks and the triggers on the driver controller
    m_driveSubsystem.setDefaultCommand(new SwerveDriveCommand(
            m_driveSubsystem,
            m_driverController
    ));

    // Populate Auto Chooser
    autoChooser.setDefaultOption("Move Forward", new MoveForward(m_driveSubsystem));
    autoChooser.addOption("Semi-Circle", new SemiCircle(m_driveSubsystem));
    autoChooser.addOption("Swerve1", new Swerve1(m_driveSubsystem));
    SmartDashboard.putData("Auto Chooser", autoChooser);

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
    // No requirements because we don't need to interrupt anything
    new XboxControllerButton(m_driverController, XboxController.Button.kBack)
        .whenPressed(new InstantCommand(m_driveSubsystem.dt::zeroGyroscope, m_driveSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
