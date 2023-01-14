// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class Mk3StandardSwerveModule {
    private final String debugName;

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANCoder canCoder;

    public Mk3StandardSwerveModule(
        String debugName,
        int driveMotorChannel,
        int steerMotorChannel,
        int canCoderChannel,
        Rotation2d steerOffset
    ) {
        this.debugName = debugName;

        ErrorCode error = null;

        /*
         * Drive Motor Initialization
         */
        TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfiguration.voltageCompSaturation = Constants.SwerveModule.MAX_VOLTAGE_VOLTS;
        driveMotorConfiguration.supplyCurrLimit.currentLimit = Constants.SwerveModule.DRIVE_CURRENT_LIMIT_AMPS;
        driveMotorConfiguration.supplyCurrLimit.enable = true;

        driveMotor = new TalonFX(driveMotorChannel);
        if ((error = driveMotor.configAllSettings(driveMotorConfiguration)) != ErrorCode.OK) {
            DriverStation.reportError(
                "Failed to configure drive motor on [" + debugName + "] module: " + error.toString(),
                false
            );
        }

        driveMotor.enableVoltageCompensation(true);

        driveMotor.setSensorPhase(true);
        driveMotor.setInverted(true);

        driveMotor.setNeutralMode(NeutralMode.Brake);

        // Reduce CAN status frame rates
        if ((error = driveMotor.setStatusFramePeriod(
            StatusFrameEnhanced.Status_1_General, 
            250, 
            Constants.SwerveModule.CAN_TIMEOUT_MS
        )) != ErrorCode.OK) {
            DriverStation.reportError(
                "Failed to set drive status frame period on [" + debugName + "] module: " + error.toString(),
                false
            );
        }

        /*
         * CANCoder Initialization
         */
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfiguration.magnetOffsetDegrees = steerOffset.getDegrees();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        
        canCoder = new CANCoder(canCoderChannel);

        if ((error = canCoder.configAllSettings(
            canCoderConfiguration, 
            Constants.SwerveModule.CAN_TIMEOUT_MS
        )) != ErrorCode.OK) {
            DriverStation.reportError(
                "Failed to configure CANCoder on [" + debugName + "] module: " + 
                error.toString(),
                false
            );
        } else {
            SmartDashboard.putNumber(debugName + " : CANCoder Initial Value", canCoder.getAbsolutePosition());
        }

        // Reduce CAN status frame rates
        if ((error = canCoder.setStatusFramePeriod(
            CANCoderStatusFrame.SensorData,
            10,
            Constants.SwerveModule.CAN_TIMEOUT_MS
        )) != ErrorCode.OK) {
            DriverStation.reportError(
                "Failed to set CANCoder status frame period on [" + debugName + "] module: " + 
                error.toString(),
                false
            );
        }

        /*
         * Steer Motor Initialization
         */
        TalonFXConfiguration steerMotorConfiguration = new TalonFXConfiguration();
        steerMotorConfiguration.slot0.kP = Constants.SwerveModule.STEER_P;
        steerMotorConfiguration.slot0.kI = Constants.SwerveModule.STEER_I;
        steerMotorConfiguration.slot0.kD = Constants.SwerveModule.STEER_D;

        steerMotorConfiguration.voltageCompSaturation = Constants.SwerveModule.MAX_VOLTAGE_VOLTS;

        steerMotorConfiguration.supplyCurrLimit.currentLimit = Constants.SwerveModule.STEER_CURRENT_LIMIT_AMPS;
        steerMotorConfiguration.supplyCurrLimit.enable = true;

        steerMotor = new TalonFX(steerMotorChannel);
        if ((error = steerMotor.configAllSettings(steerMotorConfiguration)) != ErrorCode.OK) {
            DriverStation.reportError(
                "Failed to configure steer motor on [" + debugName + "] module: " + 
                error.toString(),
                false
            );
        }

        steerMotor.enableVoltageCompensation(true);

        if ((error = steerMotor.configSelectedFeedbackSensor(
            TalonFXFeedbackDevice.IntegratedSensor, 
            0, 
            Constants.SwerveModule.CAN_TIMEOUT_MS
        )) != ErrorCode.OK) {
            DriverStation.reportError(
                "Failed to set steer feedback sensor on [" + debugName + "] module: " + 
                error.toString(),
                false
            );
        }
        steerMotor.setSensorPhase(true);
        steerMotor.setInverted(true);

        steerMotor.setNeutralMode(NeutralMode.Brake);

        // getAbsoluteAngle()
        if ((error = steerMotor.setSelectedSensorPosition(
            Math.toRadians(canCoder.getAbsolutePosition()) / Constants.SwerveModule.STEER_RADIANS_PER_TICK, 
            0,
            Constants.SwerveModule.CAN_TIMEOUT_MS
        )) != ErrorCode.OK) {
            DriverStation.reportError(
                "Failed to set steer encoder position on [" + debugName + "] module: " + 
                error.toString(),
                false
            );
        }

        // Reduce CAN status frame rates
        if ((error = steerMotor.setStatusFramePeriod(
            StatusFrameEnhanced.Status_1_General,
            250,
            Constants.SwerveModule.CAN_TIMEOUT_MS
        )) != ErrorCode.OK) {
            DriverStation.reportError(
                "Failed to set steer status frame period on [" + debugName + "] module: " + 
                error.toString(),
                false
            );
        }
    }

    public void setState(double speedMetersPerSecond, Rotation2d steerAngle) {
        SwerveModuleState desiredState = new SwerveModuleState(speedMetersPerSecond, steerAngle);
        // Reduce radians to 0 to 2pi range and simplify to nearest angle
        desiredState = SwerveModuleState.optimize(
            desiredState, 
            getState().angle
        );

        // Set the motor to our desired velocity as a percentage of our max velocity

        SmartDashboard.putNumber(debugName + ": Speed", desiredState.speedMetersPerSecond);

        SmartDashboard.putNumber(debugName + ": Rotation", desiredState.angle.getRadians());

        SmartDashboard.putNumber(debugName + ": Sensor Position", steerMotor.getSelectedSensorPosition());

        driveMotor.set(
            TalonFXControlMode.PercentOutput, 
            desiredState.speedMetersPerSecond / Constants.SwerveModule.MAX_VELOCITY_METERS_PER_SECOND
        );

        steerMotor.set(
            TalonFXControlMode.Position, 
            desiredState.angle.getRadians() / Constants.SwerveModule.STEER_RADIANS_PER_TICK
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getSelectedSensorPosition() * Constants.SwerveModule.DRIVE_METERS_PER_TICK,
            new Rotation2d(
                steerMotor.getSelectedSensorPosition() * Constants.SwerveModule.STEER_RADIANS_PER_TICK
            )
        );
    }
}
