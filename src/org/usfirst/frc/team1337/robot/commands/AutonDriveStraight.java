package org.usfirst.frc.team1337.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonDriveStraight extends CommandGroup {

    public AutonDriveStraight() {
    	//dirve straight x ticks
    		addSequential(new DriveStraightToDistance(5000,10));
    		addSequential(new ZeroEncoders());
    }
}
