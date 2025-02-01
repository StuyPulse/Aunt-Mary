package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToLvl2 extends ElevatorToHeight{
    public ElevatorToLvl2Alt(){
        super(Elevator.L2_HEIGHT_METERS);
    }
}