package com.stuypulse.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;

public class ElevatorImpl extends Elevator {
    private final TalonFX motor;
    private final SmartNumber targetHeight;
    private final DigitalInput bumpSwitchBottom;

    public ElevatorImpl() {
        motor = new TalonFX(Ports.Elevator.MOTOR);
        Motors.Elevator.MOTOR_CONFIG.configure(motor);

        bumpSwitchBottom = new DigitalInput(Ports.Elevator.BOTTOM_SWITCH);

        targetHeight = new SmartNumber("Elevator/Target Height", Constants.Elevator.MIN_HEIGHT_METERS);
    }

    @Override
    public void setTargetHeight(double height) {
        targetHeight.set(
                SLMath.clamp(
                        height,
                        Constants.Elevator.MIN_HEIGHT_METERS,
                        Constants.Elevator.MAX_HEIGHT_METERS));
    }

    @Override
    public double getTargetHeight() {
        return targetHeight.getAsDouble();
    }

    @Override
    public double getCurrentHeight() {
        return motor.getPosition().getValueAsDouble() * Constants.Elevator.Encoders.DISTANCE_PER_ROTATION;
    }

    @Override
    public boolean atTargetHeight() {
        return Math.abs(getTargetHeight() - getCurrentHeight()) < Settings.Elevator.HEIGHT_TOLERANCE_METERS;
    }

    @Override
    public boolean atBottom() {
        return !bumpSwitchBottom.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        motor.setControl(new MotionMagicVoltage( getTargetHeight() / Constants.Elevator.Encoders.POSITION_CONVERSION_FACTOR));

        if (atBottom()) {
            motor.setPosition(Constants.Elevator.MIN_HEIGHT_METERS / Constants.Elevator.Encoders.POSITION_CONVERSION_FACTOR);
        }

        SmartDashboard.putNumber("Elevator/Current Height", getCurrentHeight());
        SmartDashboard.putNumber("Elevator/Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Current", motor.getStatorCurrent().getValueAsDouble());
    }
}
