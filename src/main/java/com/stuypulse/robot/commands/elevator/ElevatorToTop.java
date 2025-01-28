package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Constants;

public class ElevatorToTop extends ElevatorToHeight{
    public ElevatorToTop() {
        super(Constants.Elevator.MAX_HEIGHT_METERS);
    }
    
}
