// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class Limelight extends SubsystemBase {
  private NetworkTable table;
  private NetworkTable tX;
  private NetworkTable tY;

  
  /** Creates a new Limelight. */
  public Limelight() {

    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public double getDistance() {
    return (LimelightConstants.kGoalHeightMeters - LimelightConstants.kLimelightLensHeightMeters) / Math.tan(LimelightConstants.kMountAngleRadians + Units.degreesToRadians(LimelightHelpers.getTY("")));
  }

  public double distanceToArmAngle(double distance) {
    return distance; //TODO: enter right equation
  }

  public double getTX() {
    return LimelightHelpers.getTX("");
  }

  public double getTY() {
    return LimelightHelpers.getTY("");
  }

  public boolean hasCorrectTarget() {
    if (DriverStation.getAlliance().get() == Alliance.Blue && LimelightHelpers.getFiducialID("") == 1) {
      return true;
    } else if (DriverStation.getAlliance().get() == Alliance.Red && LimelightHelpers.getFiducialID("") == 3) {
      return true;
    } else if (DriverStation.getAlliance().get() != Alliance.Blue && DriverStation.getAlliance().get() != Alliance.Red) {
      if (LimelightHelpers.getFiducialID("eight") == 8) {
        while (LimelightHelpers.getTX("") != 0) {

        }
        while (LimelightHelpers.getTY("") != 0) {

        }

        return true;
      }
    }
    else {
      return false;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}