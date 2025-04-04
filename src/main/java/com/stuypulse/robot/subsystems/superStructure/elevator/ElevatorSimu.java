
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.superStructure.elevator;

import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.Derivative;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SettableNumber;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

public class ElevatorSimu extends Elevator {

    private final ElevatorSim sim;
    private final LinearSystemLoop<N2, N1, N2> controller;
    private final MotionProfile motionProfile;

    private SettableNumber velLimitMetersPerSecond;
    private SettableNumber accelLimitMetersPerSecond;

    private IStream accel;

    private Optional<Double> voltageOverride;

    protected ElevatorSimu() {
        sim = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            Constants.Elevator.Encoders.GEAR_RATIO,
            Constants.Elevator.MASS_KG,
            Constants.Elevator.DRUM_RADIUS_METERS,
            Constants.Elevator.MIN_HEIGHT_METERS,
            Constants.Elevator.MAX_HEIGHT_METERS,
            false,
            Constants.Elevator.MIN_HEIGHT_METERS);
        
        sim.setState(Constants.Elevator.MIN_HEIGHT_METERS, 0);

        LinearSystem<N2, N1, N2> elevatorSystem = LinearSystemId.createElevatorSystem(
            DCMotor.getKrakenX60(1), 
            Constants.Elevator.MASS_KG,
            Constants.Elevator.DRUM_RADIUS_METERS, 
            Constants.Elevator.Encoders.GEAR_RATIO);
        
        KalmanFilter<N2, N1, N2> kalmanFilter = new KalmanFilter<>(
            Nat.N2(), 
            Nat.N2(), 
            elevatorSystem, 
            VecBuilder.fill(1000.0, 0), 
            VecBuilder.fill(0.00001, 0.00001), 
            Settings.DT);

        LinearQuadraticRegulator<N2, N1, N2> lqr = new LinearQuadraticRegulator<N2, N1, N2>(
            elevatorSystem, 
            VecBuilder.fill(1E-9, 100), 
            VecBuilder.fill(12.0),
            Settings.DT);
        
        controller = new LinearSystemLoop<>(elevatorSystem, lqr, kalmanFilter, 12.0, Settings.DT);

        velLimitMetersPerSecond = new SettableNumber(Settings.Elevator.Constraints.MAX_VELOCITY_METERS_PER_SECOND_TELEOP);
        accelLimitMetersPerSecond = new SettableNumber(Settings.Elevator.Constraints.MAX_ACCEL_METERS_PER_SECOND_PER_SECOND_TELEOP);
        motionProfile = new MotionProfile(velLimitMetersPerSecond, accelLimitMetersPerSecond);

        motionProfile.reset(Constants.Elevator.MIN_HEIGHT_METERS);

        accel = IStream.create(() -> sim.getVelocityMetersPerSecond())
            .filtered(new Derivative());

        voltageOverride = Optional.empty();
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
            2, 
            7, 
            "Elevator", 
            voltage -> setVoltageOverride(Optional.of(voltage)), 
            () -> getCurrentHeight(), 
            () -> sim.getVelocityMetersPerSecond(), 
            () -> voltageOverride.get(), 
            getInstance()
        );
    }

    private double getTargetHeight() {
        return getState().getTargetHeight();
    }

    @Override
    public double getCurrentHeight() {
        return sim.getPositionMeters();
    }

    private boolean isWithinTolerance(double toleranceMeters) {
        return Math.abs(getTargetHeight() - getCurrentHeight()) < toleranceMeters;
    }

    @Override
    public boolean atTargetHeight() {
        return isWithinTolerance(Settings.Elevator.HEIGHT_TOLERANCE_METERS);
    }

    @Override
    public boolean atCanSkipClearanceHeight() {
        return isWithinTolerance(Settings.Elevator.HEIGHT_TOLERANCE_TO_SKIP_CLEARANCE);
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public void setMotionProfileConstraints(double velLimitMetersPerSecond, double accelLimitMetersPerSecondSquared) {
        this.velLimitMetersPerSecond.set(velLimitMetersPerSecond);
        this.accelLimitMetersPerSecond.set(accelLimitMetersPerSecondSquared);
    }

    @Override
    public void periodic() {
        super.periodic();

        double setpoint = motionProfile.get(getTargetHeight());

        SmartDashboard.putNumber("Elevator/Constraints/Max vel (m per s)", velLimitMetersPerSecond.get());
        SmartDashboard.putNumber("Elevator/Constraints/Max accel (m per s per s)", accelLimitMetersPerSecond.get());

        SmartDashboard.putNumber("Elevator/Setpoint", setpoint);

        controller.setNextR(VecBuilder.fill(setpoint, 0));
        controller.correct(VecBuilder.fill(sim.getPositionMeters(), sim.getVelocityMetersPerSecond()));
        controller.predict(Settings.DT);

        if (Settings.EnabledSubsystems.ELEVATOR.get()) {
            if (voltageOverride.isPresent()) {
                sim.setInputVoltage(0);
            }
            else {
                sim.setInputVoltage(controller.getU(0));
            }
        }
        else {
            sim.setInputVoltage(0);
        }

        sim.update(Settings.DT);
    }
}
