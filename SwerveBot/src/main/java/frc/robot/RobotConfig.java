/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

/**
 * Add your docs here.
 */
public class RobotConfig {
    public static void setStartingConfig(){
        RobotMap.backLeftAbsEnconder.setSemiPeriodMode(true);
        RobotMap.backRightAbsEnconder.setSemiPeriodMode(true);
        RobotMap.frontLeftAbsEnconder.setSemiPeriodMode(true);
        RobotMap.frontRightAbsEnconder.setSemiPeriodMode(true);
        RobotMap.frontLeftDrive.setIdleMode(IdleMode.kBrake);
        RobotMap.frontLeftTurn.setIdleMode(IdleMode.kBrake);
        RobotMap.backLeftDrive.setIdleMode(IdleMode.kBrake);
        RobotMap.backLeftTurn.setIdleMode(IdleMode.kBrake);
        RobotMap.frontRightDrive.setIdleMode(IdleMode.kBrake);
        RobotMap.frontRightTurn.setIdleMode(IdleMode.kBrake);
        RobotMap.backRightDrive.setIdleMode(IdleMode.kBrake);
        RobotMap.backRightTurn.setIdleMode(IdleMode.kBrake);
       
    }
    public static void setTeleopConfig(){
        RobotMap.backLeftTurn.setIdleMode(IdleMode.kBrake);
        RobotMap.frontLeftTurn.setIdleMode(IdleMode.kBrake);
        RobotMap.backRightTurn.setIdleMode(IdleMode.kBrake);
        RobotMap.frontRightTurn.setIdleMode(IdleMode.kBrake);
        RobotMap.backLeftSwerveModule.initModule();
        RobotMap.backRightSwerveModule.initModule();
        RobotMap.frontLeftSwerveModule.initModule();
        RobotMap.frontRightSwerveModule.initModule();
    }
    public static void setDisabledConfig(){
        RobotMap.backLeftTurn.setIdleMode(IdleMode.kCoast);
        RobotMap.frontLeftTurn.setIdleMode(IdleMode.kCoast);
        RobotMap.backRightTurn.setIdleMode(IdleMode.kCoast);
        RobotMap.frontRightTurn.setIdleMode(IdleMode.kCoast);


    }
}
