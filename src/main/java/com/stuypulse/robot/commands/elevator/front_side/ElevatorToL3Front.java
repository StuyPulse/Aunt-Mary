package com.stuypulse.robot.commands.elevator.front_side;

import com.stuypulse.robot.commands.elevator.ElevatorToHeight;
import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToL3Front extends ElevatorToHeight{
    public ElevatorToL3Front () {
        super(Elevator.FRONT_L3_HEIGHT_METERS);
    }
}