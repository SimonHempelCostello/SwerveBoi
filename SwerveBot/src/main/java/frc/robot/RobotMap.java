/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
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
    public static Drivetrain drive = new Drivetrain();
    public static ButtonMap buttonMap = new ButtonMap();
    public static AHRS navx = new AHRS(Port.kMXP);
}
