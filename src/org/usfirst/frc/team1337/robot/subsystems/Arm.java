package org.usfirst.frc.team1337.robot.subsystems;

import org.usfirst.frc.team1337.robot.Robot;
import org.usfirst.frc.team1337.robot.commands.PIDArm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Arm extends Subsystem {
WPI_TalonSRX arm;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
public Arm() {
	arm = new WPI_TalonSRX(8);
	arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 5);
	arm.config_kP(0, 0.0, 0);
	arm.config_kI(0, 0.0, 0);
	arm.config_kD(0, 0.0, 0);
	
	arm.configForwardSoftLimitThreshold(0, 0);
	arm.configReverseSoftLimitThreshold(0, 0);
	arm.configForwardSoftLimitEnable(true, 0);
	arm.configReverseSoftLimitEnable(true, 0);
	
}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new PIDArm());
    }
 
    public void ArmPID(double speed) {
     if(Robot.oi.joystick2.getRawButton(1)){
			arm.set(ControlMode.Position, 0);
	} else if (Robot.oi.joystick2.getRawButton(2)){
		arm.set(ControlMode.Position, 0);
	} else if (Robot.oi.joystick2.getRawButton(3)) {
		arm.set(ControlMode.Position, 0);
	} else if (Robot.oi.joystick2.getRawButton(4)) {
		arm.set(ControlMode.Position, 0);
	} else if (Robot.oi.joystick2.getRawButton(5)) {
		arm.set(ControlMode.Position, 0);
	} else {
		arm.set(ControlMode.PercentOutput, speed*0.25);
	}
 	logging();

    }
    public void logging() {
    	SmartDashboard.putNumber("arm position", arm.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("arm set pos", arm.getActiveTrajectoryPosition());
    }
}

