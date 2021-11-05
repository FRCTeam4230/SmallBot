// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.driveTrain;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

public class MyTeleOpDriveCommand extends CommandBase {
  /** Creates a new MyTeleOpDriveCommand. */
  Drivetrain locDriveTrain;
  XboxController locDriverJoyStick;

  public MyTeleOpDriveCommand(Drivetrain driveTrain, XboxController driverJoystick) {
    locDriveTrain = driveTrain;
    locDriverJoyStick = driverJoystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  private boolean tankMode = driveTrain.useArcadeControls;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (locDriverJoyStick.getAButtonPressed()) {
      tankMode = !tankMode;
    }

    double speedMult = driveTrain.defaultSpeedMult;
    double rotMult = driveTrain.defaultRotMult;

    switch ((locDriverJoyStick.getBumper(GenericHID.Hand.kLeft) ? 1 : 0)
        - (locDriverJoyStick.getBumper(GenericHID.Hand.kRight) ? 1 : 0)) {
    case -1:
      speedMult = driveTrain.slowSpeedMult;
      rotMult = driveTrain.slowRotMult;
      break;
    case 1:
      speedMult = driveTrain.fastSpeedMult;
      rotMult = driveTrain.fastRotMult;
      break;
    }

    if (tankMode) {
      // arcade driving
      double forwardAmount = locDriverJoyStick.getY(GenericHID.Hand.kLeft);
      double turnAmount = locDriverJoyStick.getX(GenericHID.Hand.kRight);
      locDriveTrain.arcadeDrive(Math.pow(forwardAmount, 2) * Math.signum(forwardAmount) * speedMult,
          Math.pow(turnAmount, 2) * Math.signum(turnAmount) * rotMult);
    } else {
      // tank driving
      double left = locDriverJoyStick.getY(GenericHID.Hand.kLeft);
      double right = locDriverJoyStick.getY(GenericHID.Hand.kRight);
      locDriveTrain.tankDrive(Math.pow(left, 2) * Math.signum(left) * speedMult,
          Math.pow(right, 2) * Math.signum(right) * speedMult);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      locDriveTrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
