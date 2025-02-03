package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToLvl3Alt extends ElevatorToHeight{
    public ElevatorToLvl3Alt () {
        super(Elevator.ALT_L3_HEIGHT_METERS);
    }
}