/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

/**
 * Add your docs here.
 */
public class RobotMap {
    private static int backLeftTurnID  = 1;
    private static int backLeftDriveID = 2;
    private static int backRightTurnID = 3;
    private static int backRightDriveID = 4;
    private static int frontRightTurnID = 5;
    private static int frontRightDriveID = 6;
    private static int frontLeftTurnID = 7;
    private static int frontLeftDriveID = 8;
    public static CANSparkMax backLeftTurn = new CANSparkMax(backLeftTurnID, MotorType.kBrushless);
    public static CANSparkMax backLeftDrive = new CANSparkMax(backLeftDriveID, MotorType.kBrushless); 
    public static CANSparkMax backRightTurn = new CANSparkMax(backRightTurnID, MotorType.kBrushless);
    public static CANSparkMax backRightDrive = new CANSparkMax(backRightDriveID, MotorType.kBrushless); 
    public static CANSparkMax frontLeftTurn = new CANSparkMax(frontLeftTurnID, MotorType.kBrushless);
    public static CANSparkMax frontLeftDrive = new CANSparkMax(frontLeftDriveID, MotorType.kBrushless); 
    public static CANSparkMax frontRightTurn = new CANSparkMax(frontRightTurnID, MotorType.kBrushless);
    public static CANSparkMax frontRightDrive = new CANSparkMax(frontRightDriveID, MotorType.kBrushless); 
    public static Counter backLeftAbsEnconder = new Counter(0);
    public static Counter backRightAbsEnconder = new Counter(1);
    public static Counter frontRightAbsEnconder = new Counter(2);
    public static Counter frontLeftAbsEnconder = new Counter(3);
    private static double kP = 1.8;
    private static double kI = 0.02;
    private static double kD = 18;
    public static SwerveModule backLeftSwerveModule = new SwerveModule( kP, kI, kD, backLeftTurn, backLeftDrive, backLeftAbsEnconder, 524.55);
    public static SwerveModule backRightSwerveModule = new SwerveModule( kP, kI, kD, backRightTurn, backRightDrive, backRightAbsEnconder, 2039.7);
    public static SwerveModule frontLeftSwerveModule = new SwerveModule( kP, kI, kD, frontLeftTurn, frontLeftDrive, frontLeftAbsEnconder, 1036.55);
    public static SwerveModule frontRightSwerveModule = new SwerveModule( kP, kI, kD, frontRightTurn, frontRightDrive, frontRightAbsEnconder,3596.7);
    public static Drivetrain drive = new Drivetrain();
    public static ButtonMap buttonMap = new ButtonMap();
}
