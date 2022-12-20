// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public interface VisionSubsystem extends Subsystem {
    public boolean hasTarget();

    public Rotation2d getXOffset();

    public Rotation2d getYOffset();
}
