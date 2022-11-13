// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveDrive extends CommandBase {
  static double xAxisValue = 0;
  static double yAxisValue = 0;
  static double rotationalXAxisValue = 0;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(RobotContainer.driveStick.getRawAxis(Constants.X_AXIS)) > 0.15) {
      xAxisValue = RobotContainer.driveStick.getRawAxis(Constants.X_AXIS) * Constants.MAX_METERS_PER_SECOND;
    } else {
      xAxisValue = 0;
    }

    if (Math.abs(RobotContainer.driveStick.getRawAxis(Constants.Y_AXIS)) > 0.15) {
      yAxisValue = RobotContainer.driveStick.getRawAxis(Constants.Y_AXIS) * Constants.MAX_METERS_PER_SECOND;
    } else {
      yAxisValue = 0;
    }

    if (Math.abs(rotationalXAxisValue = RobotContainer.driveStick.getRawAxis(Constants.ROTATIONAL_AXIS)) > 0.15) {
      rotationalXAxisValue = RobotContainer.driveStick.getRawAxis(Constants.ROTATIONAL_AXIS)
          * Constants.MAX_METERS_PER_SECOND;
    } else {
      rotationalXAxisValue = 0;
    }

    RobotContainer.m_swerve.setChasisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(xAxisValue, yAxisValue,
        rotationalXAxisValue, RobotContainer.m_swerve.gyroAngle()));
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
