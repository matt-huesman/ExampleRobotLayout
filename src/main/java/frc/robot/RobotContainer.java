// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.Drivetrain;
import frc.robot.commands.ExampleCommand;
import frc.robot.io.ControlPannel;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystemFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final ExampleCommand autoCommand;

  private final Dashboard dashboard;

  private final DrivetrainSubsystem drivetrainSubsystem;

  private final Joystick driveJoystick;
  private final Joystick turnJoystick;
  private final ControlPannel controlPannel;

  private final JoystickButton exampleButton;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoCommand = new ExampleCommand(VisionSubsystemFactory::getVision);
    
    dashboard = Dashboard.getInstance();

    drivetrainSubsystem = new DrivetrainSubsystem();

    // Construct drive joystick
    driveJoystick = new Joystick(0);
    exampleButton = new JoystickButton(driveJoystick, 1);
    configureDriveButtonBindings();

    // Construct turn joystick
    turnJoystick = new Joystick(1);
    configureTurnButtonBindings();

    // Construct control pannel
    controlPannel = new ControlPannel(2);
    configureControlButtonBindings();

    drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> {
      double driveY = driveJoystick.getY();
      if (Math.abs(driveY) < 0.1) {
        driveY = 0;
      }

      double driveX = driveJoystick.getX();
      if (Math.abs(driveX) < 0.1) {
        driveX = 0;
      }

      double turnX = turnJoystick.getY();
      if (Math.abs(turnX) < 0.1) {
        turnX = 0;
      }
      
      drivetrainSubsystem.drive(new ChassisSpeeds(
        driveY,
        driveX,
        turnX
      ));
    }, drivetrainSubsystem));
  }

  // Configure button bindings for drive joystick
  private void configureDriveButtonBindings() {
    exampleButton.whenPressed(() -> {});
  }

  // Configure button bindings for turn joystick
  private void configureTurnButtonBindings() {}

  // Configure button bindings for control pannel
  private void configureControlButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    switch (dashboard.getAuto()) {
      case EXAMPLE_AUTO:
        return autoCommand;
    
      default:
        DriverStation.reportWarning("No auto selected!", false);
        return null;
    }
  }
}
