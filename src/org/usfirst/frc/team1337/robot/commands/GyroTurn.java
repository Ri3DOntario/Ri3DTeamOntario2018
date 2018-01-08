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
public class GyroTurn extends Command {
	private PIDController gyroPID;
	double counter = 0;
	double timeout = 0;
	// Probably needs to be tuned
	private double kPG = 0.05, kIG = 0.0005, kDG = 0.05; // Gyro PID

	private double rotate;

	public GyroTurn(double angle, double time) {
		requires(Robot.driveSubsystem);
		timeout = time;
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
		
		gyroPID.setAbsoluteTolerance(1); // might need to be tuned if it never ends
		gyroPID.setInputRange(-180.0f, 180.0f);
		gyroPID.setOutputRange(-1.0, 1.0);
		gyroPID.setContinuous(true);
		gyroPID.setSetpoint(angle);

		/* Add the PID Controller to the Test-mode dashboard, allowing manual */
		/* tuning of the Turn Controller's P, I and D coefficients. */
		/* Typically, only the P value needs to be modified. */
		LiveWindow.addActuator("Drive", "Gyro Only Tuning", gyroPID);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveSubsystem.resetEnc();
		Robot.driveSubsystem.zeroGyro();

		gyroPID.reset();
		gyroPID.enable();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		counter++; // might not even be necessary... or work at all
		rotate = Math.max(Math.min(0.8, rotate), -0.8); // the double value can be changed for desired speeds
		Robot.driveSubsystem.arcadeDrive(0, rotate);
		SmartDashboard.putBoolean("gyroTurnPID.onTarget()", gyroPID.onTarget());
		SmartDashboard.putNumber("Gyro Error", gyroPID.getError());
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
		SmartDashboard.putBoolean("gyroTurnPID.onTarget()", gyroPID.onTarget());
		gyroPID.disable();
		Robot.driveSubsystem.arcadeDrive(0, 0);
		Robot.driveSubsystem.resetEnc();
		Robot.driveSubsystem.zeroGyro();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}