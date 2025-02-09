/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/*-
 * File containing all of the configurations that different motors require.
 *
 * Such configurations include:
 *  - If it is Inverted
 *  - The Idle Mode of the Motor
 *  - The Current Limit
 *  - The Open Loop Ramp Rate
 */
public interface Motors {

    public interface Funnel {
        TalonFXConfig MOTOR_CONFIG = new TalonFXConfig()
            .withCurrentLimitAmps(40)
            .withRampRate(0.25)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInvertedValue(InvertedValue.Clockwise_Positive);
    }

    public interface Arm {
        TalonFXConfig MOTOR_CONFIG = new TalonFXConfig()
            .withCurrentLimitAmps(80)
            .withRampRate(0.25)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withPIDConstants(Settings.Arm.PID.kP, Settings.Arm.PID.kI, Settings.Arm.PID.kD)
            .withFFConstants(Settings.Arm.FF.kS, Settings.Arm.FF.kV, Settings.Arm.FF.kA, Settings.Arm.FF.kG)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withSensorToMechanismRatio(Constants.Arm.GEAR_RATIO)
            .withRemoteSensor(Ports.Arm.ABSOLUTE_ENCODER, FeedbackSensorSourceValue.FusedCANcoder, Constants.Arm.GEAR_RATIO)
            .withMotionProfile(Settings.Arm.MAX_VEL_ROTATIONS_PER_S, Settings.Arm.MAX_ACCEL_ROTATIONS_PER_S_PER_S);
    }

    public interface Froggy {
        TalonFXConfig ROLLER_MOTOR_CONFIG = new TalonFXConfig()
            .withCurrentLimitAmps(40)
            .withRampRate(0.25)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInvertedValue(InvertedValue.Clockwise_Positive);

        TalonFXConfig PIVOT_MOTOR_CONFIG = new TalonFXConfig()
            .withCurrentLimitAmps(80)
            .withRampRate(0.25)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withPIDConstants(Settings.Froggy.PID.kP, Settings.Froggy.PID.kI, Settings.Froggy.PID.kD)
            .withFFConstants(Settings.Froggy.FF.kS, Settings.Froggy.FF.kV, Settings.Froggy.FF.kA, Settings.Froggy.FF.kG)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withSensorToMechanismRatio(Constants.Froggy.GEAR_RATIO)
            .withRemoteSensor(Ports.Froggy.ABSOLUTE_ENCODER, FeedbackSensorSourceValue.FusedCANcoder, Constants.Froggy.GEAR_RATIO)
            .withMotionProfile(Settings.Froggy.MAX_VEL_ROTATIONS_PER_S, Settings.Froggy.MAX_ACCEL_ROTATIONS_PER_S_PER_S);
    }

    public interface Elevator {
        TalonFXConfig MOTOR_CONFIG = new TalonFXConfig()
            .withCurrentLimitAmps(80)
            .withRampRate(0.25)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withPIDConstants(Settings.Elevator.PID.kP, Settings.Elevator.PID.kI, Settings.Elevator.PID.kD)
            .withFFConstants(Settings.Elevator.FF.kS, Settings.Elevator.FF.kV, Settings.Elevator.FF.kA, Settings.Elevator.FF.kG)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withSensorToMechanismRatio(Constants.Elevator.GEAR_RATIO)
            .withMotionProfile(Settings.Elevator.MAX_VELOCITY_METERS_PER_SECOND, Settings.Elevator.MAX_ACCEL_METERS_PER_SECOND_PER_SECOND);
    }

    public interface Climb {
        TalonFXConfig MOTOR_CONFIG = new TalonFXConfig()
            .withCurrentLimitAmps(80)
            .withRampRate(0.25)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withPIDConstants(Settings.Climb.PID.kP, Settings.Climb.PID.kI, Settings.Climb.PID.kD)
            .withFFConstants(Settings.Climb.FF.kS, Settings.Climb.FF.kV, Settings.Climb.FF.kA, Settings.Climb.FF.kG)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withSensorToMechanismRatio(Constants.Climb.GEAR_RATIO)
            .withRemoteSensor(Ports.Climb.ABSOLUTE_ENCODER, FeedbackSensorSourceValue.FusedCANcoder, Constants.Froggy.GEAR_RATIO);
    }

    public static class TalonFXConfig {
        public final TalonFXConfiguration configuration = new TalonFXConfiguration();

        private final Slot0Configs slot0Configs = new Slot0Configs();
        private final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        private final ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        private final OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        private final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        private final FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

        public void configure(TalonFX motor) {
            motor.getConfigurator().apply(configuration);
        }

        // SLOT 0 CONFIGS 

        public TalonFXConfig withPIDConstants(double kP, double kI, double kD) {
            slot0Configs.kP = kP;
            slot0Configs.kI = kI;
            slot0Configs.kD = kD;

            configuration.withSlot0(slot0Configs);

            return this;
        }

        public TalonFXConfig withFFConstants(double kS, double kV, double kA) {
            slot0Configs.kS = kS;
            slot0Configs.kV = kV;
            slot0Configs.kA = kA;
            slot0Configs.kG = 0.0;

            configuration.withSlot0(slot0Configs);

            return this;
        }

        public TalonFXConfig withFFConstants(double kS, double kV, double kA, double kG) {
            slot0Configs.kS = kS;
            slot0Configs.kV = kV;
            slot0Configs.kA = kA;
            slot0Configs.kG = kG;

            configuration.withSlot0(slot0Configs);

            return this;
        }

        public TalonFXConfig withGravityType(GravityTypeValue gravityType) {
            slot0Configs.GravityType = gravityType;

            configuration.withSlot0(slot0Configs);

            return this;
        }

        // MOTOR OUTPUT CONFIGS 

        public TalonFXConfig withInvertedValue(InvertedValue invertedValue) {
            motorOutputConfigs.Inverted = invertedValue;

            configuration.withMotorOutput(motorOutputConfigs);

            return this;
        }

        public TalonFXConfig withNeutralMode(NeutralModeValue neutralMode) {
            motorOutputConfigs.NeutralMode = neutralMode;

            configuration.withMotorOutput(motorOutputConfigs);

            return this;
        }

        // RAMP RATE CONFIGS 

        public TalonFXConfig withRampRate(double rampRate) {
            closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = rampRate;
            closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = rampRate;
            closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = rampRate;

            openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = rampRate;
            openLoopRampsConfigs.TorqueOpenLoopRampPeriod = rampRate;
            openLoopRampsConfigs.VoltageOpenLoopRampPeriod = rampRate;

            configuration.withClosedLoopRamps(closedLoopRampsConfigs);
            configuration.withOpenLoopRamps(openLoopRampsConfigs);

            return this;
        }

        // CURRENT LIMIT CONFIGS 

        public TalonFXConfig withCurrentLimitAmps(double currentLimitAmps) {
            currentLimitsConfigs.StatorCurrentLimit = currentLimitAmps;
            currentLimitsConfigs.StatorCurrentLimitEnable = true;

            configuration.withCurrentLimits(currentLimitsConfigs);

            return this;
        }

        // MOTION MAGIC CONFIGS

        public TalonFXConfig withMotionProfile(double maxVelocity, double maxAcceleration) {
            motionMagicConfigs.MotionMagicCruiseVelocity = maxVelocity;
            motionMagicConfigs.MotionMagicAcceleration = maxAcceleration;

            configuration.withMotionMagic(motionMagicConfigs);

            return this;
        }

        // FEEDBACK CONFIGS

        public TalonFXConfig withRemoteSensor(int ID, FeedbackSensorSourceValue source, double rotorToSensorRatio) {
            feedbackConfigs.FeedbackRemoteSensorID = ID;
            feedbackConfigs.FeedbackSensorSource = source;
            feedbackConfigs.RotorToSensorRatio = rotorToSensorRatio;

            configuration.withFeedback(feedbackConfigs);

            return this;
        }

        public TalonFXConfig withSensorToMechanismRatio(double sensorToMechanismRatio) {
            feedbackConfigs.SensorToMechanismRatio = sensorToMechanismRatio;

            configuration.withFeedback(feedbackConfigs);

            return this;
        }
    }
}
