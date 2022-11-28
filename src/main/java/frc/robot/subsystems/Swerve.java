// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
public class Swerve extends SubsystemBase {

  public static enum JoystickConfiguration{
    Controller,
    Joystick,
    RotationalJoystick
  }
  //IMU
  Pigeon2 imu = new Pigeon2(Constants.PIGEON_CAN);

  public Pose2d robotPosition = new Pose2d(new Translation2d(10, 5), new Rotation2d(Math.toRadians(0)));
  NetworkTableEntry joystick_x = Shuffleboard.getTab("Joystick").add("X-Axis", 0).getEntry();
  NetworkTableEntry joystick_y = Shuffleboard.getTab("Joystick").add("Y-Axis", 0).getEntry();

  double speedAxis;

  //Module placement from the center of robot
  Translation2d m_frontLeftLocation = new Translation2d(Constants.TRANSLATION_2D_METERS, Constants.TRANSLATION_2D_METERS);
  Translation2d m_frontRightLocation = new Translation2d(Constants.TRANSLATION_2D_METERS, -Constants.TRANSLATION_2D_METERS);
  Translation2d m_backLeftLocation = new Translation2d(-Constants.TRANSLATION_2D_METERS, Constants.TRANSLATION_2D_METERS);
  Translation2d m_backRightLocation = new Translation2d(-Constants.TRANSLATION_2D_METERS, -Constants.TRANSLATION_2D_METERS);

  //Kinematics and Odometry for Swerve Drive
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, gyroAngle(), new Pose2d(5.0, 13.5, new Rotation2d()));

  //Show status of each module(velocity, offset, etc)
  ShuffleboardLayout frontLeftModuleStateShuffleboard = Shuffleboard.getTab("Modulestates").getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(0, 0);
  ShuffleboardLayout frontRightModuleStateShuffleboard = Shuffleboard.getTab("Modulestates").getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(2, 0);
  ShuffleboardLayout backLeftModuleStateShuffleboard = Shuffleboard.getTab("Modulestates").getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(4, 0);
  ShuffleboardLayout backRightModuleStateShuffleboard = Shuffleboard.getTab("Modulestates").getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(6, 0);

  Field2d field = new Field2d();
  ComplexWidget fieldShuffleboard = Shuffleboard.getTab("Field").add("hi", field).withWidget(BuiltInWidgets.kField).withProperties(Map.of("robot icon size", 20));

  //Starting ChassisSpeed
  ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0);

  GearRatio gearRatio = Mk4iSwerveModuleHelper.GearRatio.L1;

  //Each module created
  SwerveModule frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(frontLeftModuleStateShuffleboard, gearRatio, Constants.FRONT_LEFT_DRIVE_MOTOR, Constants.FRONT_LEFT_TURN_MOTOR, Constants.FRONT_LEFT_CANCODER, Constants.FRONT_LEFT_TURN_OFFSET);
  SwerveModule frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(frontRightModuleStateShuffleboard, gearRatio, Constants.FRONT_RIGHT_DRIVE_MOTOR, Constants.FRONT_RIGHT_TURN_MOTOR, Constants.FRONT_RIGHT_CANCODER, Constants.FRONT_RIGHT_TURN_OFFSET);
  SwerveModule backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(backLeftModuleStateShuffleboard, gearRatio, Constants.REAR_LEFT_DRIVE_MOTOR, Constants.REAR_LEFT_TURN_MOTOR, Constants.REAR_LEFT_CANCODER, Constants.REAR_LEFT_TURN_OFFSET);
  SwerveModule backRightModule = Mk4iSwerveModuleHelper.createFalcon500(backRightModuleStateShuffleboard, gearRatio, Constants.REAR_RIGHT_DRIVE_MOTOR, Constants.REAR_RIGHT_TURN_MOTOR, Constants.REAR_RIGHT_CANCODER, Constants.REAR_RIGHT_TURN_OFFSET);

  public Swerve() {}
   
  public void setChasisSpeed(ChassisSpeeds speed){
    speeds = speed;

  }

  public Rotation2d gyroAngle() {
    return Rotation2d.fromDegrees(imu.getYaw());
  }
  
  public void resetGyro(){
    imu.setYaw(0);
  }

  public double zAxis(){
      return 0.3;//((Math.round(((RobotContainer.bigdriveStick.getRawAxis(3) * -1)+1)/2))/5);
  }

  
  @Override
  public void periodic() {
    speedAxis = zAxis();

    // This method will be called once per scheduler run
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, speedAxis);

    joystick_x.setDouble(RobotContainer.bigdriveStick.getRawAxis(Constants.X_AXIS));
    joystick_y.setDouble(RobotContainer.bigdriveStick.getRawAxis(Constants.Y_AXIS));


    robotPosition = m_odometry.update(gyroAngle(), moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3]);
    
    field.setRobotPose(robotPosition);
    frontLeftModule.set(moduleStates[0].speedMetersPerSecond * speedAxis * Constants.MAX_VOLTAGE, moduleStates[0].angle.getRadians());
    frontRightModule.set(moduleStates[1].speedMetersPerSecond * speedAxis * Constants.MAX_VOLTAGE, moduleStates[1].angle.getRadians());
    backLeftModule.set(moduleStates[2].speedMetersPerSecond * speedAxis * Constants.MAX_VOLTAGE, moduleStates[2].angle.getRadians());
    backRightModule.set(moduleStates[3].speedMetersPerSecond * speedAxis * Constants.MAX_VOLTAGE, moduleStates[3].angle.getRadians());

    
  }
}