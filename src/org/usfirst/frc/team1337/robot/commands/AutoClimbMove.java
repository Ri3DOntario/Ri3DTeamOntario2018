package org.usfirst.frc.team1337.robot.commands;

import org.usfirst.frc.team1337.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoClimbMove extends Command {
double pos;
int timer;
    public AutoClimbMove(double position) {
    	pos = position;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.climb);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	timer = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.climb.AutoClimb(pos);
    	timer++;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if((Math.abs(Robot.climb.leftMaster.getSelectedSensorPosition(0))-Math.abs(Robot.climb.leftMaster.getActiveTrajectoryPosition()))>35 || timer < 40) {
    		return false;
    	}else {
    		return true;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.climb.leftMaster.set(ControlMode.PercentOutput, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
