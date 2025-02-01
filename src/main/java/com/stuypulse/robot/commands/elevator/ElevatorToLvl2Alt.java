package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToLvl2Alt extends ElevatorToHeight{
    public ElevatorToLvl2Alt(){
        super(Elevator.ALT_L2_HEIGHT_METERS);
    }
}