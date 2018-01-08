package org.usfirst.frc.team1337.robot;

import org.usfirst.frc.team1337.robot.commands.ReverseDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public Joystick joystick1 = new Joystick(0);
	Button j1button1 = new JoystickButton(joystick1, 1);
	Button j1button2 = new JoystickButton(joystick1, 2);
	Button j1button3 = new JoystickButton(joystick1, 3);
	Button j1button4 = new JoystickButton(joystick1, 4);
	Button j1button5 = new JoystickButton(joystick1, 5);
	Button j1button6 = new JoystickButton(joystick1, 6);
	Button j1button7 = new JoystickButton(joystick1, 7);
	Button j1button10 = new JoystickButton(joystick1, 10);
	
	public OI(){
	j1button10.whenPressed(new ReverseDrive());	
	}
	
	public Joystick getJoystick1(){
		return joystick1; // returns joystick1 values
	}
	
}
