// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  private final Mk3StandardSwerveModule frontLeftModule;
  private final Mk3StandardSwerveModule frontRightModule;
  private final Mk3StandardSwerveModule backLeftModule;
  private final Mk3StandardSwerveModule backRightModule;

  // Representation of our robots swerve module posititions relative to the center of the wheels.
  private final SwerveDriveKinematics kinematics;

  private final AHRS navx;

  private final SwerveDriveOdometry odometry;

  /** Creates a new SwerveDrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    frontLeftModule = new Mk3StandardSwerveModule(
      "front left",
      Constants.Drivetrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
      Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_MOTOR,
      Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER,
      new Rotation2d(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_OFFSET_RADIANS)
    );
    frontRightModule = new Mk3StandardSwerveModule(
      "front right",
      Constants.Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
      Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
      new Rotation2d(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_OFFSET_RADIANS)
    );
    backLeftModule = new Mk3StandardSwerveModule(
      "back left",
      Constants.Drivetrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
      Constants.Drivetrain.BACK_LEFT_MODULE_STEER_MOTOR,
      Constants.Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER,
      new Rotation2d(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_OFFSET_RADIANS)
    );
    backRightModule = new Mk3StandardSwerveModule(
      "back right",
      Constants.Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
      Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_MOTOR,
      Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER,
      new Rotation2d(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_OFFSET_RADIANS)
    );

    kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    navx.reset();

    odometry = new SwerveDriveOdometry(kinematics, navx.getRotation2d());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(swerveModuleStates);

    odometry.update(navx.getRotation2d(), swerveModuleStates);
  }

  private void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    boolean hasVelocity = swerveModuleStates[0].speedMetersPerSecond != 0
      && swerveModuleStates[1].speedMetersPerSecond != 0 
      && swerveModuleStates[2].speedMetersPerSecond != 0
      && swerveModuleStates[3].speedMetersPerSecond != 0;

    frontLeftModule.setState(
      swerveModuleStates[0].speedMetersPerSecond, 
      hasVelocity ? swerveModuleStates[0].angle : frontLeftModule.getState().angle
    );
    frontRightModule.setState(
      swerveModuleStates[1].speedMetersPerSecond, 
      hasVelocity ? swerveModuleStates[1].angle : frontRightModule.getState().angle
    );
    backLeftModule.setState(
      swerveModuleStates[2].speedMetersPerSecond, 
      hasVelocity ? swerveModuleStates[2].angle : backLeftModule.getState().angle
    );
    backRightModule.setState(
      swerveModuleStates[3].speedMetersPerSecond, 
      hasVelocity ? swerveModuleStates[3].angle : backRightModule.getState().angle
    );
  }
}
