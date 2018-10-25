package org.usfirst.frc.team1337.robot.commands;

import org.usfirst.frc.team1337.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ZeroEncoders extends Command {

    public ZeroEncoders() {
    		requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		Robot.driveSubsystem.resetEnc();
    		Robot.driveSubsystem.resetGyro();
    		
    		//why is this called here?
    		logging();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	//lol this makes things easier
    		end();
    }
    
    void logging() {
    		Robot.logCurrentCommand("ZeroEncoders");
    		Robot.driveSubsystem.logging();
    }
}