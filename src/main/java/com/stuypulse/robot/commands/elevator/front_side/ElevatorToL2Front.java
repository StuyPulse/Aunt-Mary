package com.stuypulse.robot.commands.elevator.front_side;

import com.stuypulse.robot.commands.elevator.ElevatorToHeight;
import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToL2Front extends ElevatorToHeight{
    public ElevatorToL2Front(){
        super(Elevator.FRONT_L2_HEIGHT_METERS);
    }
}