package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToLvl3 extends ElevatorToHeight{
    public ElevatorToLvl3(){
        super(Elevator.L3_HEIGHT_METERS);
    }
}