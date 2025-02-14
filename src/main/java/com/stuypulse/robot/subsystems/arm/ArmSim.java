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
        Rotation2d targetAngle;
        switch (getState()) {
            case STOW:
                targetAngle = Settings.Arm.STOW_ANGLE;
                break;
            case FEED:
                targetAngle = Settings.Arm.FEED_ANGLE;
                break;
            case L2_FRONT:
                targetAngle = Settings.Arm.L2_ANGLE_FRONT;
                break;
            case L3_FRONT:
                targetAngle = Settings.Arm.L3_ANGLE_FRONT;
                break;
            case L4_FRONT:
                targetAngle = Settings.Arm.L4_ANGLE_FRONT;
                break;
            case L2_BACK:
                targetAngle = Settings.Arm.L2_ANGLE_BACK;
                break;
            case L3_BACK:
                targetAngle = Settings.Arm.L3_ANGLE_BACK;
                break;
            case L4_BACK:
                targetAngle = Settings.Arm.L4_ANGLE_BACK;
                break;
            case ALGAE_L2:
                targetAngle = Settings.Arm.ALGAE_L2_ANGLE;
                break;
            case ALGAE_L3:
                targetAngle = Settings.Arm.ALGAE_L3_ANGLE;
                break;
            case BARGE:
                targetAngle = Settings.Arm.BARGE_ANGLE;
                break;
            case VERTICAL:
                targetAngle = Rotation2d.fromDegrees(90); // could also be -270...
                break;
            default:
                targetAngle = Settings.Arm.STOW_ANGLE;
                break;
        }
        return Rotation2d.fromDegrees(SLMath.clamp(targetAngle.getDegrees(), Constants.Arm.MIN_ANGLE.getDegrees(), Constants.Arm.MAX_ANGLE.getDegrees()));
    }

    private Rotation2d getCurrentAngle() {
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
