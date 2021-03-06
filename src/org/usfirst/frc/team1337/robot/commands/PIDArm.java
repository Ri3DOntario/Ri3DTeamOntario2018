package org.usfirst.frc.team1337.robot.commands;

import org.usfirst.frc.team1337.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PIDArm extends Command {

    public PIDArm() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.armSub);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	
    	if(Robot.oi.joystick1.getRawButton(1)) {
    		Robot.armSub.ArmPID(0.5); //why is this set to this value?
    	}else if (Robot.oi.joystick1.getRawButton(2)) {
    		Robot.armSub.ArmPID(-0.5);

    	}else {
    		Robot.armSub.ArmPID(0.0);

    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
