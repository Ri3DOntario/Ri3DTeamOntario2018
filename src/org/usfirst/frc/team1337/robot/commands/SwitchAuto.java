package org.usfirst.frc.team1337.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SwitchAuto extends CommandGroup {

    public SwitchAuto() {
    	///current values are just placeholders
    	//To-Do: instead of encoder ticks, input distance in cm/inches instead.
    	
    	addSequential(new ZeroEncoders()); //zero the encoders
    	addParallel(new DriveStraightToDistance(999, 10)); 
    	addParallel(new AutoClimbMove(999));
    	addSequential(new AutoArmMove(999));
    	addSequential(new AutoClimbMove(999));
    	addParallel(new AutoArmMove(999));
    	addSequential(new AutoIntake(-0.5,50)); 
    }
}
