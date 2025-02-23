package com.stuypulse.robot.util;

public class SettableNumber extends Number {
    private double value;

    public SettableNumber(double value) {
        this.value = value;
    }

    public void set(double value) {
        this.value = value;
    }

    public double get() {
        return value;
    }

    @Override
    public int intValue() {
        return (int) value;
    }

    @Override
    public long longValue() {
        return (long) value;
    }

    @Override
    public float floatValue() {
        return (float) value;
    }

    @Override
    public double doubleValue() {
        return (double) value;
    }
}
