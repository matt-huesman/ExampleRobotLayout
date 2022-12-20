// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;

public class AprilTagVisionSubsystem implements VisionSubsystem {
  /** Creates a new AprilTagVisionSubsystem. */
  public AprilTagVisionSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public boolean hasTarget() {
    return false;
  }

  @Override
  public Rotation2d getXOffset() {
    return null;
  }

  @Override
  public Rotation2d getYOffset() {
    return null;
  }
}
