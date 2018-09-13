package org.usfirst.frc.team1337.robot.subsystems;

import org.usfirst.frc.team1337.robot.Robot;
import org.usfirst.frc.team1337.robot.RobotMap;
import org.usfirst.frc.team1337.robot.commands.ClimbControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
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
		
		//set other motors to follow the left master
		leftSlave.set(ControlMode.Follower, leftMaster.getDeviceID());
		rightMaster.set(ControlMode.Follower, leftMaster.getDeviceID());
		rightSlave.set(ControlMode.Follower, leftMaster.getDeviceID());

		
		//climbRight.setInverted(true);
		rightMaster.setInverted(true);
		rightSlave.setInverted(true);
		
	//set the sensor to an absolute magnetic encoder
		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 5);
		
		//set PID values
		//oof this 
		leftMaster.config_kP(0, 0.0, 0);
		leftMaster.config_kI(0, 0.0, 0);
		leftMaster.config_kD(0, 0.0, 0);
		
		//configure software limits for the climber
		//why are there duplicates
		leftMaster.configForwardSoftLimitThreshold(0, 0);
		leftMaster.configReverseSoftLimitThreshold(0, 0);
		leftMaster.configForwardSoftLimitEnable(false, 0);
		leftMaster.configReverseSoftLimitEnable(false, 0);
	}

	//this command will always run and will be the first to run unless interrupted 
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
	/*	} else if (Robot.oi.joystick1.getRawButton(2)) {//climb 
			leftMaster.set(ControlMode.Position, 0);
		} else if (Robot.oi.joystick1.getRawButton(1)) {//switch
			leftMaster.set(ControlMode.Position, 0);	*/
		} else {
			leftMaster.set(ControlMode.PercentOutput, speed*1);
		}
	     logging();
	}
	
	//Automatically climb using PID
	//oof this could really break things if not tuned properly
	//I don't recommend using auto climb unless we spend time doing this 
	public void AutoClimb(double pos) {
		leftMaster.set(ControlMode.Position, pos);
	}
	
	//oof more not logging logging
	public void logging() {
		SmartDashboard.putNumber("climber pos", leftMaster.getSelectedSensorPosition(0)); //wtf is this
    	SmartDashboard.putNumber("climber preset", leftMaster.getActiveTrajectoryPosition());//wtf is this also

	}
}
