// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

/** Add your docs here. */
public class VisionSubsystemFactory {
    private static final AprilTagVisionSubsystem APRIL_TAG_VISION_SUBSYSTEM = new AprilTagVisionSubsystem();

    private static final LimeLightVisionSubsystem LIME_LIGHT_VISION_SUBSYSTEM = new LimeLightVisionSubsystem();

    public static VisionSubsystem getVision() {
        if (LIME_LIGHT_VISION_SUBSYSTEM.isUpdating()) {
            return LIME_LIGHT_VISION_SUBSYSTEM;
        } else {
            return APRIL_TAG_VISION_SUBSYSTEM;
        }
    }
}
