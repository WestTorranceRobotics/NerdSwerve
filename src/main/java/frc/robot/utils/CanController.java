// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.fasterxml.jackson.databind.BeanProperty;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */

public class CanController {
    private SparkMax motor;
    private SparkMaxConfig config = new SparkMaxConfig();

    public CanController(int motorid) {
        motor = new SparkMax(motorid, MotorType.kBrushless);
    }

    public void configureMotor(double rotorToSensorRatio, double openLoopRampRate, int controlFramePeriod,
            int encoderControlFramePeriod, boolean inverted, boolean idleBrake) {

        config.smartCurrentLimit(50);


        // motor.restoreFactoryDefaults();
        // motor.setSmartCurrentLimit(50);

        if (idleBrake) {
            config.idleMode(IdleMode.kBrake);
        } else {
            config.idleMode(IdleMode.kCoast);
        }
        // motor.setOpenLoopRampRate(openLoopRampRate);
        config.inverted(inverted);
        // motor.setControlFramePeriodMs(controlFramePeriod);

        // Encoder Stuffs
        // motor.getEncoder().setMeasurementPeriod(encoderControlFramePeriod);
        // motor.getEncoder().setPositionConversionFactor(rotorToSensorRatio);
        // motor.getEncoder().setVelocityConversionFactor(rotorToSensorRatio);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void configurePIDF(double kP, double kI, double kD, double kIz, double kF, double kMinOutput,
            double kMaxOutput) {

        // motor.getClosedLoopController().setP(kP);
        // motor.getClosedLoopController().setI(kI);
        // motor.getClosedLoopController().setD(kD);
        // motor.getClosedLoopController().setIZone(kIz);
        // motor.getClosedLoopController().setFF(kF);
        // motor.getClosedLoopController().setOutputRange(kMinOutput, kMaxOutput);
        
        config.closedLoop.pid(kP, kI, kD);
        config.closedLoop.iZone(kIz);
        config.closedLoop.velocityFF(kF);
        config.closedLoop.outputRange(kMinOutput, kMaxOutput);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void configureSmartMotion(double kMaxVelocity, double kMinVelocity, double kMaxAccel, double kAllowedError) {
        // int smartMotionSlot = 0;

        // var config = new MAXMotionConfig();
        // config.maxVelocity(kMaxVelocity);
        // config.maxAcceleration(kMaxAccel);
        // config.allowedClosedLoopError(kAllowedError);

        
        config.closedLoop.smartMotion.maxVelocity(kMaxVelocity);
        config.closedLoop.smartMotion.allowedClosedLoopError(kAllowedError);
        config.closedLoop.smartMotion.minOutputVelocity(kMinVelocity);
        config.closedLoop.smartMotion.allowedClosedLoopError(kAllowedError);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // config.smart

        // motor.getClosedLoopController().setSmartMotionMaxVelocity(kMaxVelocity, smartMotionSlot);
        // ;
        // motor.getClosedLoopController().setSmartMotionMinOutputVelocity(kMinVelocity, smartMotionSlot);
        // motor.getClosedLoopController().setSmartMotionMaxAccel(kMaxAccel, smartMotionSlot);
        // motor.getClosedLoopController().setSmartMotionAllowedClosedLoopError(kAllowedError, smartMotionSlot);

    }

    // Encoder and logging stuffs
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    public double getAbsolutePosition() {
        return motor.getAbsoluteEncoder().getPosition();
    }

    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    public void setPosition(double pos) {
        motor.getEncoder().setPosition(pos);
    }

    public double getMotorVoltage() {
        return motor.getAppliedOutput();
    }

    // public double getCompensationVoltage() {
    //     return motor.getVoltageCompensationNominalVoltage();
    // }

    // Set Speed Stuffs
    public void stop() {
        motor.set(0);
    }

    public void setSpeed(double percent) {
        motor.set(percent);
    }

    // PID Stuffs
    public void setVelocityControl(double velocity) {
        motor.getClosedLoopController().setReference(velocity, SparkBase.ControlType.kVelocity);
    }

    public void setPositionControl(double position) {
        motor.getClosedLoopController().setReference(position, SparkBase.ControlType.kPosition);
    }

    public void setSmartMotionPositionControl(double position) {
        motor.getClosedLoopController().setReference(position, SparkBase.ControlType.kSmartMotion);
    }

    public SparkClosedLoopController getPIDController() {
        return motor.getClosedLoopController();
    }

    public void updateShuffleboard() {

    }

}
