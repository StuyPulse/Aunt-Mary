/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.elevator;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ElevatorFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSimu extends Elevator {

    private final ElevatorSim sim;

    private final Controller controller;

    protected ElevatorSimu() {
        sim = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            Constants.Elevator.Encoders.GEAR_RATIO,
            Constants.Elevator.MASS_KG,
            Constants.Elevator.DRUM_RADIUS_METERS,
            Constants.Elevator.MIN_HEIGHT_METERS,
            Constants.Elevator.MAX_HEIGHT_METERS,
            true,
            Constants.Elevator.MIN_HEIGHT_METERS
        );

        MotionProfile motionProfile = new MotionProfile(
            Settings.Elevator.MAX_VELOCITY_METERS_PER_SECOND,
            Settings.Elevator.MAX_ACCEL_METERS_PER_SECOND_PER_SECOND
        );

        controller = new MotorFeedforward(Gains.Elevator.FF.kS, Gains.Elevator.FF.kV, Gains.Elevator.FF.kA).position()
            .add(new ElevatorFeedforward(Gains.Elevator.FF.kG))
            .add(new PIDController(Gains.Elevator.PID.kP, Gains.Elevator.PID.kI, Gains.Elevator.PID.kD))
            .setSetpointFilter(motionProfile);
    }

    private double getTargetHeight() {
        return getState().getTargetHeight();
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
    public void periodic() {
        super.periodic();

        controller.update(getTargetHeight(), getCurrentHeight());

        sim.setInputVoltage(controller.getOutput());
        sim.update(Settings.DT);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));
    }
}
