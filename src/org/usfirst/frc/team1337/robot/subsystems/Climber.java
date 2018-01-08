package org.usfirst.frc.team1337.robot.subsystems;

import org.usfirst.frc.team1337.robot.Robot;
import org.usfirst.frc.team1337.robot.RobotMap;
import org.usfirst.frc.team1337.robot.commands.ClimbControl;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Climber extends Subsystem {
	WPI_TalonSRX leftMaster, rightMaster, leftSlave, rightSlave;
	SpeedControllerGroup climbLeft, climbRight;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public Climber() {
		leftMaster = new WPI_TalonSRX(RobotMap.LEFT_CLIMB_MASTER);
		leftSlave = new WPI_TalonSRX(RobotMap.LEFT_CLIMB_SLAVE);
		rightMaster = new WPI_TalonSRX(RobotMap.RIGHT_CLIMB_MASTER);
		rightSlave = new WPI_TalonSRX(RobotMap.RIGHT_CLIMB_SLAVE);

		climbLeft = new SpeedControllerGroup(leftMaster, leftSlave);
		climbRight = new SpeedControllerGroup(rightMaster, rightSlave);
		climbRight.setInverted(true);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new ClimbControl());
	}

	public void climbSet(double speed) {
		climbLeft.set(speed);
		climbRight.set(speed);
	}
	
	public void logging() {
		SmartDashboard.putNumber("Climb speed", climbLeft.get());
	}
}
