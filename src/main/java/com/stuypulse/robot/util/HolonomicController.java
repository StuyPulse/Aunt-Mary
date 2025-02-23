package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class HolonomicController implements Sendable {
    private Controller xController;
    private Controller yController;
    private AngleController angleController;

    public HolonomicController(Controller xController, Controller yController, AngleController angleController) {
        this.xController = xController;
        this.yController = yController;
        this.angleController = angleController;
    }

    public ChassisSpeeds update(Pose2d setpoint, Pose2d measurement) {
        xController.update(setpoint.getX(), measurement.getX());
        yController.update(setpoint.getY(), measurement.getY());
        angleController.update(
                Angle.fromRotation2d(setpoint.getRotation()),
                Angle.fromRotation2d(measurement.getRotation()));

        return getOutput();
    }

    public ChassisSpeeds getOutput() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xController.getOutput(),
                yController.getOutput(),
                angleController.getOutput(),
                angleController.getMeasurement().getRotation2d());
    }

    public ChassisSpeeds getError() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xController.getError(),
                yController.getError(),
                angleController.getError().toDegrees(),
                angleController.getMeasurement().getRotation2d());
    }

    public boolean isDone(double xToleranceMeters, double yToleranceMeters, double angleToleranceDegrees) {
        return xController.isDone(xToleranceMeters)
                && yController.isDone(yToleranceMeters)
                && angleController.isDoneDegrees(angleToleranceDegrees);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Holonomic Controller");
        builder.addDoubleProperty("Angle Setpoint (degrees)", () -> angleController.getSetpoint().toDegrees(), null);
        builder.addDoubleProperty("Angle Measurement (degrees)", () -> angleController.getMeasurement().toDegrees(), null);
        builder.addDoubleProperty("X Setpoint (meters)", () -> xController.getSetpoint(), null);
        builder.addDoubleProperty("X Measurement (meters)", () -> xController.getMeasurement(), null);
        builder.addDoubleProperty("Y Setpoint (meters)", () -> yController.getSetpoint(), null);
        builder.addDoubleProperty("Y Measurement (meters)", () -> yController.getMeasurement(), null);
        builder.addDoubleProperty("X Error (meters)", () -> xController.getError(), null);
        builder.addDoubleProperty("Y Error (meters)", () -> yController.getError(), null);
        builder.addDoubleProperty("Angle Error (degrees)", () -> angleController.getError().getRotation2d().getDegrees(), null);
    }
}