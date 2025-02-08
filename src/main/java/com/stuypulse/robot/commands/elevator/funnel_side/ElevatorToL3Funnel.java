package com.stuypulse.robot.commands.elevator.funnel_side;

import com.stuypulse.robot.commands.elevator.ElevatorToHeight;
import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToL3Funnel extends ElevatorToHeight{
    public ElevatorToL3Funnel() {
        super(Elevator.FUNNEL_L3_HEIGHT_METERS);
    }
}
