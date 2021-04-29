// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.WPI_AutoFeedEnable;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  
  private WPI_TalonSRX bLeft, fRight;
  private WPI_VictorSPX fLeft, bRight;
  private SpeedControllerGroup lGroup, rGroup;
  private DifferentialDrive drivesys;
  
  public Drivetrain() {
    bLeft = new WPI_TalonSRX(kLeftMotorBackPort);
    fLeft = new WPI_VictorSPX(kLeftMotorFrontPort);
    fRight = new WPI_TalonSRX(kRightMotorFrontPort);
    bRight = new WPI_VictorSPX(kRightMotorBackPort);

    lGroup = new SpeedControllerGroup(fLeft, bLeft);
    rGroup = new SpeedControllerGroup(fRight, bRight);

    drivesys = new DifferentialDrive(lGroup, rGroup);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
