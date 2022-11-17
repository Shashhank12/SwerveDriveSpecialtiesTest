// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class SwerveDrive extends CommandBase {
  static DoubleSupplier xAxisValue;
  static DoubleSupplier yAxisValue;
  static DoubleSupplier rotationalXAxisValue;
  public static Swerve.JoystickConfiguration m_joystick;
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerve);
  }

  private static double deadbandCalc(double joystickRawAxis) {
    if (Math.abs(joystickRawAxis) > 0.01) {
      if (joystickRawAxis > 0.01) {
        return (joystickRawAxis - 0.01) / (1.0 - 0.01);
      } else {
        return (joystickRawAxis + 0.01) / (1.0 - 0.01);
      }
    } else {
      return 0.0;
    }
  }

  private static double squareAxis(double value) {
    value = deadbandCalc(value);
    value = Math.copySign(value * value, value);
    return value;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Constants.M_JOYSTICK == Swerve.JoystickConfiguration.Joystick) {
      xAxisValue = () -> -squareAxis(RobotContainer.bigdriveStick.getRawAxis(0)) * Constants.MAX_METERS_PER_SECOND;
      yAxisValue = () -> -squareAxis(RobotContainer.bigdriveStick.getRawAxis(1)) * -Constants.MAX_METERS_PER_SECOND;
      rotationalXAxisValue = () -> -squareAxis(RobotContainer.driveStick.getRawAxis(Constants.ROTATIONAL_AXIS)) * Constants.MAX_RADIANS_PER_SECOND;
    } else if(Constants.M_JOYSTICK == Swerve.JoystickConfiguration.Controller) {
    
      xAxisValue = () -> -squareAxis(RobotContainer.driveStick.getRawAxis(Constants.X_AXIS)) * Constants.MAX_METERS_PER_SECOND;
      yAxisValue = () -> -squareAxis(RobotContainer.driveStick.getRawAxis(Constants.Y_AXIS)) * -Constants.MAX_METERS_PER_SECOND;
      rotationalXAxisValue = () -> -squareAxis(RobotContainer.driveStick.getRawAxis(Constants.ROTATIONAL_AXIS)) * Constants.MAX_RADIANS_PER_SECOND;

    } else if(Constants.M_JOYSTICK == Swerve.JoystickConfiguration.RotationalJoystick) {
      xAxisValue = () -> -squareAxis(RobotContainer.bigdriveStick.getRawAxis(0)) * Constants.MAX_METERS_PER_SECOND;
      yAxisValue = () -> -squareAxis(RobotContainer.bigdriveStick.getRawAxis(1)) * -Constants.MAX_METERS_PER_SECOND;
      rotationalXAxisValue = () -> -squareAxis(RobotContainer.bigdriveStick.getRawAxis(2)) * Constants.MAX_RADIANS_PER_SECOND;
    }
    
    RobotContainer.m_swerve.setChasisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(xAxisValue.getAsDouble(), yAxisValue.getAsDouble(),
        rotationalXAxisValue.getAsDouble(), RobotContainer.m_swerve.gyroAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_swerve.setChasisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, RobotContainer.m_swerve.gyroAngle()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
