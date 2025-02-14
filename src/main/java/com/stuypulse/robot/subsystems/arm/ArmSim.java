/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSim extends Arm {

    private final SingleJointedArmSim sim;
    private final Controller controller;

    protected ArmSim() {
        sim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            Constants.Arm.GEAR_RATIO,
            Constants.Arm.MOMENT_OF_INERTIA,
            Constants.Arm.ARM_LENGTH,
            Constants.Arm.MIN_ANGLE.getRadians(),
            Constants.Arm.MAX_ANGLE.getRadians(),
            true,
            Settings.Arm.STOW_ANGLE.getRadians(),
            0
        );

        MotionProfile motionProfile = new MotionProfile(
            Settings.Arm.MAX_VEL_ROTATIONS_PER_S,
            Settings.Arm.MAX_ACCEL_ROTATIONS_PER_S_PER_S
        );

        controller = new MotorFeedforward(Gains.Arm.FF.kS, Gains.Arm.FF.kV, Gains.Arm.FF.kA).position()
            .add(new ArmFeedforward(Gains.Arm.FF.kG))
            .add(new PIDController(Gains.Arm.PID.kP, Gains.Arm.PID.kI, Gains.Arm.PID.kD))
            .setSetpointFilter(motionProfile);
    }

    @Override
    public boolean atTargetAngle() {
        return Math.abs(getCurrentAngle().getDegrees() - getTargetAngle().getDegrees()) < Settings.Arm.ANGLE_TOLERANCE_DEGREES;
    }

    private Rotation2d getTargetAngle() {
        return getState().getTargetAngle();
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRadians(sim.getAngleRads());
    }

    @Override
    public void periodic() {
        super.periodic();

        controller.update(getTargetAngle().getRotations(), getCurrentAngle().getRotations());

        sim.setInputVoltage(controller.getOutput());
        sim.update(Settings.DT);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        SmartDashboard.putNumber("Arm/Current Angle (deg)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Target Angle (deg)", getTargetAngle().getDegrees());
    }
}
