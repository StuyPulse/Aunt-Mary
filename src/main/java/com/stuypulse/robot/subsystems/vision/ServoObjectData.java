package com.stuypulse.robot.subsystems.vision;

import java.util.ArrayList;

public class ServoObjectData {
    private final double timestamp;
    private ArrayList<Double> txncList;
    private ArrayList<Double> areas;

    public ServoObjectData(double timestamp) {
        txncList = new ArrayList<>();
        areas = new ArrayList<>();
        this.timestamp = timestamp;
    }

    public void addData(double txnc, double ta) {
        txncList.add(txnc);
        areas.add(ta);
    }

    public boolean hasData() {
        return txncList.size() > 0;
    }

    public double getHighestArea() {
        if (areas.isEmpty()) { // handle empty case
            return 0.0;
        }
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

    public double txncOfHighestArea() {
        if (areas.isEmpty()) { // handle empty case
            return 0.0;
        }
        int index = 0;
        double maxArea = 0;
        for (int i = 0; i < areas.size(); i++) {
            if (areas.get(i) > maxArea) {
                maxArea = areas.get(i);
                index = i;
            }
        }
        return txncList.get(index);
    }

    public ArrayList<Double> getTXNCs() {
        return txncList;
    }

    public ArrayList<Double> getAreas() {
        return areas;
    }

    public double getTimeStamp() {
        return timestamp;
    }
}
