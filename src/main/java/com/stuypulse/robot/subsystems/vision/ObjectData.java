package com.stuypulse.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;

public class ObjectData {
    private Optional<Pose3d> closestObject;
    double timestamp;
    boolean hasData;

    public ObjectData(Optional<Pose3d> closestObject, double timestamp, boolean hasData) {
        this.closestObject = closestObject == null ? Optional.empty() : closestObject;
        this.timestamp = timestamp;
        this.hasData = hasData;
    }

    public ObjectData() {
        closestObject = Optional.empty();
        timestamp = 0.0;
        hasData = false;
    }

    public Optional<Pose3d> getClosestObject() {
        return closestObject;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public boolean hasData() {
        return hasData;
    }
}
