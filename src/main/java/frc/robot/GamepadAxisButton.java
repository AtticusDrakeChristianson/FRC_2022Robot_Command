// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 */
public class GamepadAxisButton extends Button {
	private final GenericHID m_joystick;
	private final int m_axisNumber;
	private final double m_threshold;

	/**
	 * Create a gamepad axis for triggering commands as if it were a button.
	 *
	 * @param joystick     The GenericHID object that has the axis (e.g. Joystick, KinectStick,
	 *                     etc)
	 * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
	 * 
	 * @param threshold The threshold above which the axis shall trigger a command
	 */
	public GamepadAxisButton(GenericHID joystick, int axisNumber, double threshold) {
		m_joystick = joystick;
		m_axisNumber = axisNumber;
		m_threshold = threshold;
	}

	/**
	 * Create a GamepadAxisButton with a default threshold of 0.5 to trigger
	 *
	 * @param joystick     The GenericHID object that has the axis (e.g. Joystick, KinectStick,
	 *                     etc)
	 * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
	 */
	public GamepadAxisButton(GenericHID joystick, int axisNumber) {
		this(joystick,axisNumber,0.6);
	}

	/**
	 * Gets the value of the gamepad axis.
	 *
	 * @return The value of the gamepad axis
	 */
	@Override
	public boolean get() {
		double rawAxis = m_joystick.getRawAxis(m_axisNumber);

		return Math.abs(rawAxis) > m_threshold;
	}
}