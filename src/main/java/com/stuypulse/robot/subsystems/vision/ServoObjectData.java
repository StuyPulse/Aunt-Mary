package com.stuypulse.robot.subsystems.vision;

public class ServoObjectData {
    private final double timeStamp;
    private final double rotationAngle;

    public ServoObjectData(double rotationAngle, double timestamp) {
        this.rotationAngle = rotationAngle;
        this.timeStamp = timestamp;
    }

    public double getObjectAngle() {
        return this.rotationAngle;
    }

    public double getTimeStamp() {
        return this.timeStamp;
    }

}
