package org.usfirst.frc.team1337.robot.subsystems;

import org.usfirst.frc.team1337.robot.Robot;
import org.usfirst.frc.team1337.robot.RobotMap;
import org.usfirst.frc.team1337.robot.commands.ClimbControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Climber extends Subsystem {
	public WPI_TalonSRX leftMaster, rightMaster, leftSlave, rightSlave;
	SpeedControllerGroup climbLeft, climbRight;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public Climber() {
		leftMaster = new WPI_TalonSRX(8);
		leftSlave = new WPI_TalonSRX(7);
		rightMaster = new WPI_TalonSRX(6);
		rightSlave = new WPI_TalonSRX(5);

		//climbLeft = new SpeedControllerGroup(leftMaster, leftSlave);
		//climbRight = new SpeedControllerGroup(rightMaster, rightSlave);
		
		leftSlave.set(ControlMode.Follower, leftMaster.getDeviceID());
		rightMaster.set(ControlMode.Follower, leftMaster.getDeviceID());
		rightSlave.set(ControlMode.Follower, leftMaster.getDeviceID());

		
		//climbRight.setInverted(true);
		rightMaster.setInverted(true);
		rightSlave.setInverted(true);
		
		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 5);
		leftMaster.config_kP(0, 0.0, 0);
		leftMaster.config_kI(0, 0.0, 0);
		leftMaster.config_kD(0, 0.0, 0);
		
		leftMaster.configForwardSoftLimitThreshold(0, 0);
		leftMaster.configReverseSoftLimitThreshold(0, 0);
		leftMaster.configForwardSoftLimitEnable(false, 0);
		leftMaster.configReverseSoftLimitEnable(false, 0);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new ClimbControl());
	}

	public void set(double speed) {
		//climbLeft.set(speed);
		//climbRight.set(speed);
	     if(Robot.oi.joystick2.getRawButton(3)){ //intake
				leftMaster.set(ControlMode.Position, 0);
	 	} else if (Robot.oi.joystick1.getRawButton(4)){ //scoring high scale
			leftMaster.set(ControlMode.Position, 0);
		} else if (Robot.oi.joystick1.getPOV() == 180) { //climb down
			leftMaster.set(ControlMode.Position, 0);
		} else if (Robot.oi.joystick1.getRawAxis(2) >= .60) { //intake
			leftMaster.set(ControlMode.Position, 0);
		} else if (Robot.oi.joystick1.getRawAxis(3) >= 0.60) {//outake
			leftMaster.set(ControlMode.Position, 0);
		} else if (Robot.oi.joystick1.getRawButton(2)) {//climb 
			leftMaster.set(ControlMode.Position, 0);
		} else if (Robot.oi.joystick1.getPOV() == 0) {//climb up
			leftMaster.set(ControlMode.Position, 0);	
		} else {
			leftMaster.set(ControlMode.PercentOutput, speed*0.25);
		}
	}
	
	public void AutoClimb(double pos) {
		leftMaster.set(ControlMode.Position, pos);
	}
	public void logging() {
		SmartDashboard.putNumber("climber pos", leftMaster.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("climber preset", leftMaster.getActiveTrajectoryPosition());

	}
}
