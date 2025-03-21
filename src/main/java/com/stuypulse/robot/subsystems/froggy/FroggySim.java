/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.froggy;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import static edu.wpi.first.units.Units.Second;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class FroggySim extends Froggy {

    private final SingleJointedArmSim sim;
    private final LinearSystemLoop<N2, N1, N2> controller;
    private final MotionProfile motionProfile;

    private Optional<Double> pivotVoltageOverride;
    private Rotation2d pivotOperatorOffset;

    protected FroggySim() {
        sim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            Constants.Froggy.GEAR_RATIO,
            Constants.Froggy.MOI,
            Constants.Froggy.LENGTH,
            Constants.Froggy.MINIMUM_ANGLE.getRadians(),
            Constants.Froggy.MAXIMUM_ANGLE.getRadians(),
            false,
            Settings.Froggy.STOW_ANGLE.getRadians()

        );

        LinearSystem<N2, N1, N2> armSystem = LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getKrakenX60(1), 
            Constants.Froggy.MOI, 
            Constants.Froggy.GEAR_RATIO);

        KalmanFilter<N2, N1, N2> kalmanFilter = new KalmanFilter<>(
            Nat.N2(), 
            Nat.N2(), 
            armSystem, 
            VecBuilder.fill(3.0, 3.0), 
            VecBuilder.fill(0.01, 0.01), 
            Settings.DT);
        
        LinearQuadraticRegulator<N2, N1, N2> lqr = new LinearQuadraticRegulator<N2, N1, N2>(
            armSystem, 
            VecBuilder.fill(0.00001, 100), 
            VecBuilder.fill(12),
            Settings.DT);
        
        controller = new LinearSystemLoop<>(armSystem, lqr, kalmanFilter, 12.0, Settings.DT);

        motionProfile = new MotionProfile(
            Settings.Froggy.MAX_VEL.getRadians(),
            Settings.Froggy.MAX_ACCEL.getRadians()
        );
        motionProfile.reset(Settings.Arm.MIN_ANGLE.getRadians());

        pivotVoltageOverride = Optional.empty();
        pivotOperatorOffset = Rotation2d.kZero;
    }

    @Override
    public SysIdRoutine getPivotSysIdRoutine() {
        return SysId.getRoutine(
            3, 
            7, 
            "Froggy Pivot", 
            voltage -> setPivotVoltageOverride(Optional.of(voltage)), 
            () -> getCurrentAngle().getDegrees(), 
            () -> Units.radiansToDegrees(sim.getVelocityRadPerSec()), 
            () -> pivotVoltageOverride.get(), 
            getInstance()
        );
    }

    @Override
    public boolean isAtTargetAngle() {
        return Math.abs(getCurrentAngle().getRadians() - getTargetAngle().getRadians()) < Settings.Arm.ANGLE_TOLERANCE.getRadians();
    }

    private Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(
            SLMath.clamp(
                getPivotState().getTargetAngle().getDegrees() + pivotOperatorOffset.getDegrees(),
                Constants.Froggy.MINIMUM_ANGLE.getDegrees(),
                Constants.Froggy.MAXIMUM_ANGLE.getDegrees()));
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRadians(sim.getAngleRads());
    }

    @Override
    public void setPivotVoltageOverride(Optional<Double> voltage) {
        this.pivotVoltageOverride = voltage;
    }

    @Override
    public void periodic() {
        super.periodic();

        double setpoint = motionProfile.get(getTargetAngle().getRadians());

        controller.setNextR(VecBuilder.fill(setpoint, 0));
        controller.correct(VecBuilder.fill(sim.getAngleRads(), sim.getVelocityRadPerSec()));
        controller.predict(Settings.DT);

        if (Settings.EnabledSubsystems.FROGGY.get()) {
            if (pivotVoltageOverride.isPresent()) {
                sim.setInputVoltage(pivotVoltageOverride.get());
            }
            else {
                sim.setInputVoltage(controller.getU(0));
            }
        }
        else {
            sim.setInputVoltage(0);
        }

        sim.update(Settings.DT);

        SmartDashboard.putNumber("Froggy/Pivot/Current Angle (deg)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Froggy/Pivot/Target Angle (deg)", getTargetAngle().getDegrees());
        SmartDashboard.putNumber("Froggy/Pivot/Setpoint (deg)", Units.radiansToDegrees(setpoint));
    }
}
