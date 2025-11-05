package com.stuypulse.robot.subsystems.vision;

import java.util.ArrayList;

public class ServoObjectData {
    private final double timestamp;
    private ArrayList<Double> rotationAngles;
    private ArrayList<Double> areas;

    public ServoObjectData(double timestamp) {
        rotationAngles = new ArrayList<>();
        areas = new ArrayList<>();
        this.timestamp = timestamp;
    }

    public void addData(double rotationAngle, double ta) {
        rotationAngles.add(rotationAngle);
        areas.add(ta);
    }

    public boolean hasData() {
        return rotationAngles.size() > 0;
    }

    public double getHighestArea() {
        int index = 0;
        double maxArea = 0;
        for (int i = 0; i < areas.size(); i++) {
            if (areas.get(i) > maxArea) {
                maxArea = areas.get(i);
                index = i;
            }
        }
        return areas.get(index);
    }

    public double getAngleOfHighestAreaCoral() {
        int index = 0;
        double maxArea = 0;
        for (int i = 0; i < areas.size(); i++) {
            if (areas.get(i) > maxArea) {
                maxArea = areas.get(i);
                index = i;
            }
        }
        return rotationAngles.get(index);
    }

    public ArrayList<Double> getAngles() {
        return rotationAngles;
    }

    public ArrayList<Double> getAreas() {
        return areas;
    }

    public double getTimeStamp() {
        return timestamp;
    }
}