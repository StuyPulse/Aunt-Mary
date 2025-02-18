/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ArmSim extends Arm {

    private final SingleJointedArmSim sim;
    private final LinearSystemLoop<N2, N1, N2> controller;
    private final MotionProfile motionProfile;

    private Optional<Double> voltageOverride;
    private Rotation2d operatorOffset;

    private SysIdRoutine sysidRoutine;
    private boolean isRunningSysid;

    protected ArmSim() {
        sim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            Constants.Arm.GEAR_RATIO,
            Constants.Arm.MOMENT_OF_INERTIA,
            Constants.Arm.ARM_LENGTH,
            Constants.Arm.MIN_ANGLE.getRadians(),
            Constants.Arm.MAX_ANGLE.getRadians(),
            false,
            Settings.Arm.STOW_ANGLE.getRadians()
        );

        LinearSystem<N2, N1, N2> armSystem = LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getKrakenX60(1), 
            Constants.Arm.MOMENT_OF_INERTIA, 
            Constants.Arm.GEAR_RATIO);

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
            Settings.Arm.MAX_VEL.getRadians(),
            Settings.Arm.MAX_ACCEL.getRadians()
        );
        motionProfile.reset(Settings.Arm.STOW_ANGLE.getRadians());

        voltageOverride = Optional.empty();
        operatorOffset = Rotation2d.kZero;

        isRunningSysid = false;

        sysidRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, 
                edu.wpi.first.units.Units.Volts.of(5), 
                null,
                state -> SignalLogger.writeString("SysIdArm_State", state.toString())), 
            new SysIdRoutine.Mechanism(
                output -> {
                    sim.setInputVoltage(output.in(edu.wpi.first.units.Units.Volts));
                    isRunningSysid = true;
                }, 
                state -> {
                    SignalLogger.writeDouble("Arm Position (rad)", getCurrentAngle().getRadians());
                    SignalLogger.writeDouble("Arm Velocity (rad per s)", sim.getVelocityRadPerSec());
                    SignalLogger.writeDouble("Arm Voltage", controller.getU(0));
                }, 
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

    @Override
    public boolean atTargetAngle() {
        return Math.abs(getCurrentAngle().getRadians() - getTargetAngle().getRadians()) < Settings.Arm.ANGLE_TOLERANCE.getRadians();
    }

    private Rotation2d getTargetAngle() {
        return getState().getTargetAngle().plus(operatorOffset);
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRadians(sim.getAngleRads());
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public void setOperatorOffset(Rotation2d offset) {
        this.operatorOffset = offset;
    }

    @Override
    public Rotation2d getOperatorOffset() {
        return this.operatorOffset;
    }

    @Override
    public void periodic() {
        super.periodic();

        double setpoint = motionProfile.get(getTargetAngle().getRadians());

        SmartDashboard.putNumber("Arm/Setpoint (deg)", Units.radiansToDegrees(setpoint));

        controller.setNextR(VecBuilder.fill(setpoint, 0));
        controller.correct(VecBuilder.fill(sim.getAngleRads(), sim.getVelocityRadPerSec()));
        controller.predict(Settings.DT);

        if (Settings.EnabledSubsystems.ARM.get() && !isRunningSysid) {
            if (voltageOverride.isPresent()) {
                sim.setInputVoltage(voltageOverride.get());
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

        SmartDashboard.putNumber("Arm/Current Angle (deg)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Target Angle (deg)", getTargetAngle().getDegrees());
    }
}
