package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToLvl2Funnel extends ElevatorToHeight{
    public ElevatorToLvl2Funnel(){
        super(Elevator.FUNNEL_L2_HEIGHT_METERS);
    }  
}
