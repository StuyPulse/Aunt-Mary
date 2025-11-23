package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ObjectData {

    Pose2d objectPose;
    double timestamp;

    public ObjectData(Pose2d Pose, double timestamp) {
        this.objectPose = Pose;
        this.timestamp = timestamp;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public Pose2d getObjectPose() {
        return objectPose;
    }

    public static Translation2d calculateCoralTranslation(double txnc, double tync) {
        Pose3d froggyCameraPose3d = Cameras.LimelightCameras[2].getLocation();

        Rotation3d froggyCameraRotation3d = froggyCameraPose3d.getRotation();

        double totalAngleX = froggyCameraRotation3d.getZ() - Units.degreesToRadians(txnc); // getZ seems wrong
        double totalAngleY = froggyCameraRotation3d.getY() + Units.degreesToRadians(-tync);
        SmartDashboard.putNumber("Vision/Froggy Offset X Axis", froggyCameraRotation3d.getX());
        SmartDashboard.putNumber("Vision/Froggy Offset Y Axis", froggyCameraRotation3d.getY());
        SmartDashboard.putNumber("Vision/Froggy Offset Z Axis", froggyCameraRotation3d.getZ());
        SmartDashboard.putNumber("Vision/Froggy Offset Translation Y Axis", froggyCameraPose3d.getX());
        SmartDashboard.putNumber("Vision/Froggy Offset Translation Y Axis", froggyCameraPose3d.getY());
        SmartDashboard.putNumber("Vision/Froggy Offset Translation Y Axis", froggyCameraPose3d.getZ());

        double totalHeight = froggyCameraPose3d.getZ()
                - Units.inchesToMeters(Constants.Gamepiece.CORAL_DIAMETER - 2.25); // account for it being half

        SmartDashboard.putNumber("Vision/TX BEING FED IN X", txnc);
        SmartDashboard.putNumber("Vision/Total angle X", totalAngleX); // good
        SmartDashboard.putNumber("Vision/Total angle Y", totalAngleY); // good
        SmartDashboard.putNumber("Vision/Height", totalHeight); // good
        SmartDashboard.putNumber("Vision/Angle", totalAngleY); // didnt check

        // double xDistance = (totalHeight) * 1.0/Math.tan(totalAngleY) + 0.2; // bad

        double xDistance = (totalHeight) / Math.tan(totalAngleY); // bad
        SmartDashboard.putNumber("Vision/X Distance", xDistance); // good
        // WHERE X IS FORWARD DISTANCE, Y IS SIDEWAYS DISTANCE
        // double yDistance = totalHeight * Math.tan(Units.degreesToRadians(txnc)) /
        // Math.cos(totalAngleY); // bad
        double yDistance = totalHeight * Math.tan(Units.degreesToRadians(txnc)) * xDistance;
        SmartDashboard.putNumber("Vision/Y Distance", yDistance); // good
        // WHERE X IS FORWARD DISTANCE, Y IS SIDEWAYS DISTANCE
        return new Translation2d(xDistance, yDistance);
    }
}
