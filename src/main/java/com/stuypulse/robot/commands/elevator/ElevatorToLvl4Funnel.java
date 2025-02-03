package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToLvl4Funnel extends ElevatorToHeight{
    public ElevatorToLvl4Funnel() {
        super(Elevator.FUNNEL_L4_HEIGHT_METERS);
    }
}
