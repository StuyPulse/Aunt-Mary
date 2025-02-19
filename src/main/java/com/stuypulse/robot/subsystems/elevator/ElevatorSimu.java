/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.elevator;

import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;

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
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ElevatorSimu extends Elevator {

    private final ElevatorSim sim;
    private final LinearSystemLoop<N2, N1, N2> controller;
    private final MotionProfile motionProfile;

    private Optional<Double> voltageOverride;
    private double operatorOffset;

    private SysIdRoutine sysidRoutine;
    private boolean isRunningSysid;

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

        motionProfile = new MotionProfile(
            Settings.Elevator.MAX_VELOCITY_METERS_PER_SECOND,
            Settings.Elevator.MAX_ACCEL_METERS_PER_SECOND_PER_SECOND);

        motionProfile.reset(Constants.Elevator.MIN_HEIGHT_METERS);

        voltageOverride = Optional.empty();
        operatorOffset = 0;

        isRunningSysid = false;

        sysidRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, 
                Units.Volts.of(4), 
                null,
                state -> SignalLogger.writeString("SysIdElevator_State", state.toString())), 
            new SysIdRoutine.Mechanism(
                output -> {
                    isRunningSysid = true;
                    sim.setInputVoltage(output.in(Units.Volts));
                }, 
                null, 
                this));
    }

    @Override
    public Command getSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysidRoutine.quasistatic(direction);
    }

    @Override
    public Command getSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysidRoutine.dynamic(direction);
    }

    private double getTargetHeight() {
        return getState().getTargetHeight() + operatorOffset;
    }

    @Override
    public double getCurrentHeight() {
        return sim.getPositionMeters();
    }

    @Override
    public boolean atTargetHeight() {
        return Math.abs(getTargetHeight() - getCurrentHeight()) < Settings.Elevator.HEIGHT_TOLERANCE_METERS;
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public void setOperatorOffset(double offset) {
        this.operatorOffset = offset;
    }

    @Override
    public double getOperatorOffset() {
        return this.operatorOffset;
    }

    @Override
    public void periodic() {
        super.periodic();

        double setpoint = motionProfile.get(getTargetHeight());

        SmartDashboard.putNumber("Elevator/Setpoint", setpoint);

        controller.setNextR(VecBuilder.fill(setpoint, 0));
        controller.correct(VecBuilder.fill(sim.getPositionMeters(), sim.getVelocityMetersPerSecond()));
        controller.predict(Settings.DT);

        if (Settings.EnabledSubsystems.ELEVATOR.get() && !isRunningSysid) {
            if (voltageOverride.isPresent()) {
                sim.setInputVoltage(0);
                sim.update(Settings.DT);
            }
            else {
                sim.setInputVoltage(controller.getU(0));
                sim.update(Settings.DT);
            }
        }
        else {
            sim.setInputVoltage(0);
        }
    }
}
