// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends CommandBase {
  private final Supplier<VisionSubsystem> visionSubsystemSupplier;

  private final DrivetrainSubsystem drivetrainSubsystem;

  private VisionSubsystem currentVisionSubsystem;

  private final PIDController pidController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(Supplier<VisionSubsystem> visionSubsystemSupplier) {
    this.visionSubsystemSupplier = visionSubsystemSupplier;

    this.drivetrainSubsystem = new DrivetrainSubsystem();

    pidController = new PIDController(0, 0, 0);
    pidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentVisionSubsystem = visionSubsystemSupplier.get();
    addRequirements(currentVisionSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (currentVisionSubsystem.hasTarget()) {
    //   drivetrainSubsystem.drive(
    //     new ChassisSpeeds(
    //       0,
    //       0,
    //       pidController.calculate(
    //         drivetrainSubsystem.getRotation().getRadians(), 
    //         drivetrainSubsystem.getRotation().getRadians() + currentVisionSubsystem.getXOffset().getRadians()
    //       )
    //     )
    //   );
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    getRequirements().remove(currentVisionSubsystem);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
