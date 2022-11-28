// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveDriveOdometry extends CommandBase {
  static Pose2d pos;
  static double targetXPos;
  static double targetYPos;
  static double x_vel;
  static double y_vel;
  static double turn_vel;
  static double targetTurnDegrees;
  /** Creates a new SwerveDriveOdometry. */
  public SwerveDriveOdometry(Pose2d m_targetPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerve);
    
    targetXPos = m_targetPos.getX();
    targetYPos = m_targetPos.getY();
    targetTurnDegrees = m_targetPos.getRotation().getDegrees();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pos = RobotContainer.m_swerve.robotPosition;

    if(Math.round(pos.getX() * 5) != Math.round(targetXPos * 5)){
      if(pos.getX() < targetXPos) {
        x_vel = Constants.AUTONOMOUS_VELOCITY_PER_SECOND;
      } else if(pos.getX() > targetXPos) {
        x_vel = -Constants.AUTONOMOUS_VELOCITY_PER_SECOND;
      }
    } else {
      x_vel = 0;
    }

    if(Math.round(pos.getY() * 5) != Math.round(targetYPos * 5)){
      if(pos.getY() < targetYPos) {
        y_vel = Constants.AUTONOMOUS_VELOCITY_PER_SECOND;
      } else if(pos.getY() > targetYPos) {
        y_vel = -Constants.AUTONOMOUS_VELOCITY_PER_SECOND;
      }
    } else {
      y_vel = 0;
    }

    if(Math.round(pos.getRotation().getDegrees()) != Math.round(targetTurnDegrees)){
      if(pos.getRotation().getDegrees() < targetTurnDegrees) {
        turn_vel = Constants.AUTONOMOUS_RADIANS_PER_SECOND;
      } else if(pos.getRotation().getDegrees() > targetTurnDegrees) {
        turn_vel = -Constants.AUTONOMOUS_RADIANS_PER_SECOND;
      }
    } else {
      turn_vel = 0;
    }

    RobotContainer.m_swerve.setChasisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(x_vel, y_vel,
        turn_vel, RobotContainer.m_swerve.gyroAngle()));
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_swerve.setChasisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
        0, RobotContainer.m_swerve.gyroAngle()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((Math.round(pos.getX()/5) == Math.round(targetXPos/5)) && ((Math.round(pos.getY()/5) == Math.round(targetYPos/5))) && (Math.round(pos.getRotation().getDegrees()/5) == (Math.round(targetTurnDegrees/5))))
      return true;
    return false;
  }
}
