
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.superStructure.arm;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SettableNumber;
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

import java.util.Optional;

public class ArmSim extends Arm {

    private final SingleJointedArmSim sim;
    private final LinearSystemLoop<N2, N1, N2> controller;
    private final MotionProfile motionProfile;

    private SettableNumber velLimitRadiansPerSecond;
    private SettableNumber accelLimitRadiansPerSecondSquared;

    private Optional<Double> voltageOverride;

    protected ArmSim() {
        sim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            Constants.Arm.GEAR_RATIO,
            Constants.Arm.MOMENT_OF_INERTIA,
            Constants.Arm.ARM_LENGTH,
            Units.degreesToRadians(Settings.Arm.MIN_ANGLE_DEG),
            Units.degreesToRadians(Settings.Arm.MAX_ANGLE_DEG),
            false,
            Units.degreesToRadians(Settings.Arm.MIN_ANGLE_DEG)
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

        velLimitRadiansPerSecond = new SettableNumber(Settings.Arm.Constraints.MAX_VEL_TELEOP_DEG);
        accelLimitRadiansPerSecondSquared = new SettableNumber(Settings.Arm.Constraints.MAX_ACCEL_TELEOP_DEG);

        motionProfile = new MotionProfile(velLimitRadiansPerSecond, accelLimitRadiansPerSecondSquared);
        motionProfile.reset(Units.degreesToRadians(Settings.Arm.MIN_ANGLE_DEG));

        voltageOverride = Optional.empty();
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
            3, 
            7, 
            "Arm", 
            voltage -> setVoltageOverride(Optional.of(voltage)), 
            () -> getCurrentAngleDeg(), 
            () -> Units.radiansToDegrees(sim.getVelocity()), 
            () -> voltageOverride.get(), 
            getInstance()
        );
    }

    private boolean isWithinTolerance(Rotation2d tolerance) {
        return Math.abs(getCurrentAngleDeg() - getTargetAngle().getDegrees()) < tolerance.getDegrees();
    }

    @Override
    public boolean atTargetAngle() {
        return (!isFunnelSide(getState()) ? isWithinTolerance(Settings.Arm.ANGLE_TOLERANCE_FRONT) : isWithinTolerance(Settings.Arm.ANGLE_TOLERANCE_BACK));
    }

    @Override
    public boolean atCanSkipClearanceAngle() {
        return isWithinTolerance(Settings.Arm.ANGLE_TOLERANCE_TO_SKIP_CLEARANCE);
    }

    private Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(
            SLMath.clamp(getState().getTargetAngle(), Settings.Arm.MIN_ANGLE_DEG, Settings.Arm.MAX_ANGLE_DEG));
    }

    @Override
    public double getCurrentAngleDeg() {
        return Units.radiansToDegrees(sim.getAngle());
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public double getVoltageOverride() {
        if (voltageOverride.isPresent()) {
            return voltageOverride.get();
        }
        else {
            return 0;
        }
    }

    @Override
    public void setMotionProfileConstraints(double velLimit, double accelLimit) {
        this.velLimitRadiansPerSecond.set(velLimit);
        this.accelLimitRadiansPerSecondSquared.set(accelLimit);
    }
    
    @Override
    public void simulationPeriodic() {
        super.periodic();

        double setpoint = motionProfile.get(getTargetAngle().getRadians());

        SmartDashboard.putNumber("Arm/Constraints/Max vel (deg per s)", Units.radiansToDegrees(velLimitRadiansPerSecond.get()));
        SmartDashboard.putNumber("Arm/Constraints/Max accel (deg per s per s)", Units.radiansToDegrees(accelLimitRadiansPerSecondSquared.get()));

        SmartDashboard.putNumber("Arm/Setpoint (deg)", Units.radiansToDegrees(setpoint));

        controller.setNextR(VecBuilder.fill(setpoint, 0));
        controller.correct(VecBuilder.fill(sim.getAngle(), sim.getVelocity()));
        controller.predict(Settings.DT);

        if (Settings.EnabledSubsystems.ARM.get()) {
            if (voltageOverride.isPresent()) {
                sim.setInputVoltage(voltageOverride.get());
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
