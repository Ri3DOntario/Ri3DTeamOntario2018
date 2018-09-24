package org.usfirst.frc.team1337.robot.commands;

import org.usfirst.frc.team1337.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoIntake extends Command {
int timer = 0;
int counter=0;
double setSpeed =0;
    public AutoIntake(double speed, int time) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intake);
    	//
    	timer = time;
    	setSpeed = speed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	counter=0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	counter++;
    	Robot.intake.set(setSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(counter >= timer) {
    		return true;
    	}else {
        return false;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    	//stop the intake, kinda a failsafe.
    	Robot.intake.set(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
