// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Swerve;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int PIGEON_CAN = 2;

    //FRONT LEFT MODULE
    public static final int FRONT_LEFT_CANCODER = 5;
    public static final int FRONT_LEFT_DRIVE_MOTOR = 4;
    public static final int FRONT_LEFT_TURN_MOTOR = 3;
    public static final double FRONT_LEFT_TURN_OFFSET = -Math.toRadians(92.46); //Needs tuning

    //FRONT RIGHT MODULE
    public static final int FRONT_RIGHT_CANCODER = 14;
    public static final int FRONT_RIGHT_DRIVE_MOTOR = 6;
    public static final int FRONT_RIGHT_TURN_MOTOR = 7;
    public static final double FRONT_RIGHT_TURN_OFFSET = -Math.toRadians(138.16); //Needs tuning

    //REAR LEFT MODULE
    public static final int REAR_LEFT_CANCODER = 8;
    public static final int REAR_LEFT_DRIVE_MOTOR = 9;
    public static final int REAR_LEFT_TURN_MOTOR = 10;
    public static final double REAR_LEFT_TURN_OFFSET = -Math.toRadians(349.53); //Needs tuning

    //REAR RIGHT MODULE
    public static final int REAR_RIGHT_CANCODER = 11;
    public static final int REAR_RIGHT_DRIVE_MOTOR = 12;
    public static final int REAR_RIGHT_TURN_MOTOR = 13;
    public static final double REAR_RIGHT_TURN_OFFSET = -Math.toRadians(274.82); //Needs tuning

    public static final double MAX_METERS_PER_SECOND = 0.2;//Change based on max speed Original 0.5

    public static final double MAX_RADIANS_PER_SECOND = Math.PI;//Change based on how fast the turning motors turn
    public static final double MAX_VOLTAGE = 12.5;
    public static final double AUTONOMOUS_VELOCITY_PER_SECOND = 0.1;
    public static final double AUTONOMOUS_RADIANS_PER_SECOND = 0.1;

    //joystick ports
    public static final int X_AXIS = 0; //originally port 4 (11/30/22)
    public static final int Y_AXIS = 1; //originally port 5 (11/30/22)
    public static final int ROTATIONAL_AXIS = 3; //originally port 0 (11/30/22)

    public static final double TRANSLATION_2D_METERS = 0.625/2;

    public static final Swerve.JoystickConfiguration M_JOYSTICK = Swerve.JoystickConfiguration.RotationalJoystick;


}
