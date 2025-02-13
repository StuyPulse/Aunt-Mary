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
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSim extends Arm {

    private final SingleJointedArmSim sim;
    private final Controller controller;

    private final SmartNumber targetAngle;
    private boolean rotateOverElevator;
    private static final ArmSim instance;

    private static ArmVisualizer armVisualizer = ArmVisualizer.getInstance();

    static {
        instance = new ArmSim();
    }

    public ArmSim() {
        sim =
                new SingleJointedArmSim(
                        DCMotor.getKrakenX60(1),
                        Constants.Arm.GEAR_RATIO,
                        Constants.Arm.MOMENT_OF_INERTIA,
                        Constants.Arm.ARM_LENGTH,
                        Constants.Arm.LOWER_ANGLE_LIMIT,
                        Constants.Arm.UPPER_ANGLE_LIMIT,
                        true,
                        0);

        MotionProfile motionProfile =
                new MotionProfile(
                        Settings.Arm.MAX_VEL_ROTATIONS_PER_S,
                        Settings.Arm.MAX_ACCEL_ROTATIONS_PER_S_PER_S);

        controller =
                new MotorFeedforward(Settings.Arm.FF.kS, Settings.Arm.FF.kV, Settings.Arm.FF.kA)
                        .position()
                        .add(new ArmFeedforward(Settings.Arm.FF.kG))
                        .add(
                                new PIDController(
                                        Settings.Arm.PID.kP,
                                        Settings.Arm.PID.kI,
                                        Settings.Arm.PID.kD))
                        .setSetpointFilter(motionProfile);

        targetAngle = new SmartNumber("Arm/Target Angle", 0.0);
    }

    public static ArmSim getSim() {
        return instance;
    }

    @Override
    public Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(targetAngle.getAsDouble());
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRadians(sim.getAngleRads());
    }

    @Override
    public boolean atTargetAngle() { 
        return Math.abs(getCurrentAngle().getDegrees() - getTargetAngle().getDegrees())
                < Settings.Arm.ANGLE_TOLERANCE_DEGREES;
    }

    @Override
    public void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle.set(targetAngle.getDegrees());
    }

    public boolean getRotateBoolean() {
        return rotateOverElevator;
    }

    @Override
    public void setRotateBoolean(boolean overElevator) {
        rotateOverElevator = overElevator;
    }

    @Override
    public void periodic() {
        super.periodic();

        controller.update(getTargetAngle().getDegrees(), getCurrentAngle().getDegrees());
        sim.setInputVoltage(controller.getOutput());
        sim.update(Settings.DT);
        
        // RoboRioSim.setVInVoltage(
        //         BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        armVisualizer.update();

        SmartDashboard.putNumber("Arm/Current Arm Angle", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Target Angle", getTargetAngle().getDegrees());
    }
}
