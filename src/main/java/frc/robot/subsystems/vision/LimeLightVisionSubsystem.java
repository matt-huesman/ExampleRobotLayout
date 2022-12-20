// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;

public class LimeLightVisionSubsystem implements VisionSubsystem {
  /** Creates a new LimeLightVisionSubsystem. */
  public LimeLightVisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isUpdating() {
    return false;
  }

  @Override
  public boolean hasTarget() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public Rotation2d getXOffset() {
    // -27 to 27
    return null;
  }

  @Override
  public Rotation2d getYOffset() {
    // TODO Auto-generated method stub
    return null;
  }
}
