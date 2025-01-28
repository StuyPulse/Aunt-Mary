package com.stuypulse.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ElevatorFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorImpl extends Elevator {
    private final TalonFX motor;

    private final DigitalInput bottomBumpSwitch;
    private final DigitalInput topBumpSwitch;

    private final SmartNumber targetHeight;

    private final Controller controller;

    private boolean hasBeenReset;

    public ElevatorImpl() {
        motor = new TalonFX(Ports.Elevator.MOTOR);

        bottomBumpSwitch = new DigitalInput(Ports.Elevator.BOTTOM_SWITCH);
        topBumpSwitch = new DigitalInput(Ports.Elevator.TOP_SWITCH);

        targetHeight = new SmartNumber("Elevator/Target Height", Constants.Elevator.MIN_HEIGHT_METERS);

        MotionProfile motionProfile = new MotionProfile(Settings.Elevator.MAX_VELOCITY_METERS_PER_SECOND, Settings.Elevator.MAX_ACCEL_METERS_PER_SECOND_PER_SECOND);
        
        controller = new MotorFeedforward(Settings.Elevator.FF.kS, Settings.Elevator.FF.kV, Settings.Elevator.FF.kA).position()
            .add(new ElevatorFeedforward(Settings.Elevator.FF.kG))
            .add(new PIDController(Settings.Elevator.PID.kP, Settings.Elevator.PID.kI, Settings.Elevator.PID.kD))
            .setSetpointFilter(motionProfile);
        
        hasBeenReset = false;
    }
    
    @Override
    public void setTargetHeight(double height) {
        targetHeight.set(
            SLMath.clamp(
                height, 
                Constants.Elevator.MIN_HEIGHT_METERS, 
                Constants.Elevator.MAX_HEIGHT_METERS
            )
        );
    }

    @Override
    public double getTargetHeight() {
        return targetHeight.getAsDouble();
    }

    @Override
    public double getCurrentHeight() {
        return motor.getPosition().getValueAsDouble() * Constants.Elevator.Encoders.POSITION_CONVERSION_FACTOR;
    }

    @Override
    public boolean atTargetHeight() {
        return Math.abs(getTargetHeight() - getCurrentHeight()) < Settings.Elevator.HEIGHT_TOLERANCE_METERS.get();
    }

    private void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (bottomBumpSwitch.get()) {
            hasBeenReset = true;
            motor.setPosition(0);
        }
        
        if (topBumpSwitch.get()) {
            hasBeenReset = true;
            motor.setPosition(Constants.Elevator.MAX_HEIGHT_METERS / Constants.Elevator.Encoders.POSITION_CONVERSION_FACTOR);
        }

        if (!hasBeenReset) {
            setVoltage(-1);
        } else {
            controller.update(getTargetHeight(), getCurrentHeight());
            setVoltage(controller.getOutput());
        }

        SmartDashboard.putNumber("Elevator/Current Height", getCurrentHeight());
    }
}