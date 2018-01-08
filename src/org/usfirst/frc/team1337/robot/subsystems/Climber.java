package org.usfirst.frc.team1337.robot.subsystems;

import org.usfirst.frc.team1337.robot.Robot;
import org.usfirst.frc.team1337.robot.commands.ClimbControl;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {
	WPI_TalonSRX leftMaster, rightMaster, leftSlave, rightSlave;
	SpeedControllerGroup climbLeft, climbRight;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public Climber() {
		leftMaster = new WPI_TalonSRX(5);
		leftSlave = new WPI_TalonSRX(6);
		rightMaster = new WPI_TalonSRX(7);
		rightSlave = new WPI_TalonSRX(8);

		climbLeft = new SpeedControllerGroup(leftMaster, leftSlave);
		climbRight = new SpeedControllerGroup(rightMaster, rightSlave);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new ClimbControl());
	}

	public void climbSet(double speed) {
		climbLeft.set(speed);
		climbRight.set(-speed);
	}
}
