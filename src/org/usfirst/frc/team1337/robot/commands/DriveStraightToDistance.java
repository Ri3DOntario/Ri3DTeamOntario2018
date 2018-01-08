package org.usfirst.frc.team1337.robot.commands;

import org.usfirst.frc.team1337.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveStraightToDistance extends Command {
	private PIDController gyroPID;
	double counter = 0;
	double timeout = 0;
	double mySetpoint;
	// Probably needs to be tuned
	private double kPG = 0.055, kIG = 0.0, kDG = 0.055; // Gyro PID
	private double kPE = 0.001, kIE = 0.00, kDE = 0; // Encoder PID

	private double rotate;
    @SuppressWarnings("deprecation")
	public DriveStraightToDistance(double ticks, double time) {
    		timeout = time;
		requires(Robot.driveSubsystem);
		mySetpoint = ticks;
		gyroPID = new PIDController(kPG, kIG, kDG, new PIDSource() {
			PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				m_sourceType = pidSource;
			}

			@Override
			public PIDSourceType getPIDSourceType() {
				return m_sourceType;
			}

			@Override
			public double pidGet() {
				return Robot.driveSubsystem.getGyro();
			}
		}, new PIDOutput() {
			@Override
			public void pidWrite(double pidRotate) {
				rotate = pidRotate;
			}
		});
		gyroPID.setAbsoluteTolerance(0.05);
		gyroPID.setSetpoint(0);

		LiveWindow.addActuator("Drive Gyro", "Gyro PID Tuning", gyroPID);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		Robot.driveSubsystem.initEncPID();
		Robot.driveSubsystem.resetEncPID();
		Robot.driveSubsystem.Setpoint(mySetpoint);
		Robot.driveSubsystem.resetEnc();
		Robot.driveSubsystem.zeroGyro();
		gyroPID.reset();
		gyroPID.enable();
		Robot.driveSubsystem.enableEncPID();   	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		Robot.driveSubsystem.enableEncPID();

		Robot.driveSubsystem.arcadeDrive(Robot.driveSubsystem.scaleSpeedPID(), rotate); // changed for desired speeds

		counter++;
		logging();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    		if (counter >= timeout)
			return true;
		else
			return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    		logging();
		Robot.driveSubsystem.disableEncPID();
		gyroPID.disable();
		Robot.driveSubsystem.arcadeDrive(0, 0); // stop motors
		Robot.driveSubsystem.resetEnc();
		Robot.driveSubsystem.zeroGyro();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		end();
    }
    
    void logging() {
    		Robot.logCurrentCommandAndSubsystem("DriveStraightToDistance");
    		Robot.driveSubsystem.logging();
    		SmartDashboard.putBoolean("On Target Enc", Robot.driveSubsystem.encOnTarget());
    }
}
