package com.stuypulse.robot.commands.elevator.algae;

import com.stuypulse.robot.constants.Settings.Elevator;
import com.stuypulse.robot.commands.elevator.*;

public class ElevatorToAlgaeL3 extends ElevatorToHeight{
    public ElevatorToAlgaeL3() {
        super(Elevator.ALGAE_L3_HEIGHT_METERS);
    }
}