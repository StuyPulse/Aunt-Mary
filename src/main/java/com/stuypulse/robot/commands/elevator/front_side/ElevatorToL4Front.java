package com.stuypulse.robot.commands.elevator.front_side;

import com.stuypulse.robot.commands.elevator.ElevatorToHeight;
import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToL4Front extends ElevatorToHeight{
    public ElevatorToL4Front(){
        super(Elevator.FRONT_L4_HEIGHT_METERS);
    }
}
