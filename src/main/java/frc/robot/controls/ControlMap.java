package frc.robot.Controls;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class ControlMap {
    // port ids
    private static int DRIVER_PORT = 0;
    private static int GUNNER_PORT = 1;
    private static int DRIVER_PORT_BUTTONS = 2;
    private static int GUNNER_PORT_BUTTONS = 3;

    // Controller objects
    public static CommandXboxController m_driverController = new CommandXboxController(DRIVER_PORT);
    public static CommandXboxController m_gunnerController = new CommandXboxController(GUNNER_PORT);
    public static Joystick m_driverButtons = new Joystick(DRIVER_PORT_BUTTONS);
    public static Joystick m_gunnerButtons = new Joystick(GUNNER_PORT_BUTTONS);
}