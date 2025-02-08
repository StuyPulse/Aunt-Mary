package com.stuypulse.robot.commands.elevator.funnel_side;

import com.stuypulse.robot.commands.elevator.ElevatorToHeight;
import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToL2Funnel extends ElevatorToHeight{
    public ElevatorToL2Funnel(){
        super(Elevator.FUNNEL_L2_HEIGHT_METERS);
    }  
}
