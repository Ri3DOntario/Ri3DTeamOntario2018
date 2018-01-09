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
public WPI_TalonSRX arm;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
public Arm() {
	arm = new WPI_TalonSRX(9);
	arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 5);
	arm.config_kP(0, 0.0, 0);
	arm.config_kI(0, 0.0, 0);
	arm.config_kD(0, 0.0, 0);
	
	//arm.configForwardSoftLimitThreshold(0, 0);
	//arm.configReverseSoftLimitThreshold(0, 0);
	arm.configForwardSoftLimitEnable(false, 0);
	arm.configReverseSoftLimitEnable(false, 0);
	
}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new PIDArm());
    }
 
    public void ArmPID(double speed) {
     if(Robot.oi.joystick2.getRawButton(3)){ //intake
			arm.set(ControlMode.Position, 0);
	} else if (Robot.oi.joystick1.getRawButton(4)){ //scoring high scale
		arm.set(ControlMode.Position, 0);
	} else if (Robot.oi.joystick1.getRawAxis(2) >= .60) { //intake
		arm.set(ControlMode.Position, 0);
	} else if (Robot.oi.joystick1.getRawAxis(3) >= 0.60) {//outake
		arm.set(ControlMode.Position, 0);
	} else if (Robot.oi.joystick1.getRawButton(2)) {//climb position
		arm.set(ControlMode.Position, 0);
	} else {
		arm.set(ControlMode.PercentOutput, speed*0.40);
	}
 	logging();

    }
    public void logging() {
    	SmartDashboard.putNumber("arm position", arm.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("arm set pos", arm.getActiveTrajectoryPosition());
    }
    public void AutoArm(double pos) {
    	arm.set(ControlMode.Position, pos);
    }
}

