package com.stuypulse.robot.commands.elevator.funnel_side;

import com.stuypulse.robot.commands.elevator.ElevatorToHeight;
import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToL4Funnel extends ElevatorToHeight{
    public ElevatorToL4Funnel() {
        super(Elevator.FUNNEL_L4_HEIGHT_METERS);
    }
}
