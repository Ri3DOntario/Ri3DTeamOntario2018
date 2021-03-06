package org.usfirst.frc.team1337.robot.commands;

import org.usfirst.frc.team1337.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ReverseDrive extends Command {

	public ReverseDrive() {
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		logging();
		
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.driveSubsystem.reverse = -1;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {
		
		//since this is toggled, does this mean that this works?
		Robot.driveSubsystem.reverse = 1;
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
	
	void logging() {
		//current monitoring
		Robot.logCurrentCommand("ReverseDrive");
		Robot.driveSubsystem.logging();
	}
}
