package org.usfirst.frc.team1337.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SwitchAuto extends CommandGroup {

    public SwitchAuto() {
    	addSequential(new ZeroEncoders());
    	addParallel(new DriveStraightToDistance(999, 10));
    	addParallel(new AutoClimbMove(999));
    	addSequential(new AutoArmMove(999));
    	addSequential(new AutoClimbMove(999));
    	addParallel(new AutoArmMove(999));
    	addSequential(new AutoIntake(-0.5,50));
    }
}
