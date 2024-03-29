// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.math.BetterArrayList;
import frc.robot.math.ElevatorMath;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

    public static final double cArmMinLength = 0.43;
    public static final double cArmMaxLength = 1;
    public static final Translation2d cArmOrigin = new Translation2d(0, 0);
    public static final BetterArrayList<ElevatorMath.ElevatorBoundary> cElevatorBoundaries = new BetterArrayList<>();

    //TODO FIX BEFORE RUNNING
    public static final Translation2d cArmInit = new Translation2d(1, 1);

    public static final Joystick joystick = new Joystick(0);

    public static final Trigger button3 = new Trigger(() -> joystick.getRawButton(3));
    public static final Trigger button4 = new Trigger(() -> joystick.getRawButton(4));
    public static final Trigger button5 = new Trigger(() -> joystick.getRawButton(5));
    public static final Trigger button6 = new Trigger(() -> joystick.getRawButton(6));
    public static final Trigger button7 = new Trigger(() -> joystick.getRawButton(7));
    public static final Trigger button8 = new Trigger(() -> joystick.getRawButton(8));
    public static final Trigger trigger = new Trigger(joystick::getTrigger);

    public static final int rotateMotor1ID = 1;
    public static final int rotateMotor2ID = 2;
    public static final int extendMotorID = 3;
    public static final int ankleMotorID = 4;

    public static final int shooterMotor1ID = 5;
    public static final int shooterMotor2ID = 6;
    public static final int shooterControlMotorID = 7;

    public static final int rotateEncoderID = 0;
    public static final int extendEncoderID = 1;
    public static final int ankleEncoderID = 2;

    public static final double cRotateP = 5.5;
    public static final double cRotateI = 0.2;
    public static final double cRotateD = 0.2;
    public static final double cRotateMax = 1;
    public static final double cRotateMin = -1;
    public static final double cRotateDeadband = 0;
    public static final double cExtendP = 2;
    public static final double cExtendI = 0.2;
    public static final double cExtendD = 0.1;
    public static final double cExtendMax = 1;
    public static final double cExtendMin = -1;
    public static final double cExtendDeadband = 0;

    public static final double cAnkleP = 6;
    public static final double cAnkleI = 0;
    public static final double cAnkleD = 0;
    public static final double cAnkleMax = 1;
    public static final double cAnkleMin = -1;
    public static final double cAnkleDeadband = 0;

//    public static final double cArmMin = 0.597; //1 degree
//    public static final double cArmMax = 0.89; //105 degrees
//    public static final double cAnkleMin = 0.325; // -90 degrees
//    public static final double cAnkleMax = 0.825; // 90 degrees

    //NEGATIVE EXTENSION

    public static class OperatorConstants
    {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}
