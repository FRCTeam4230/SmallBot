// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.driveTrain.useArcadeControls) {
      // arcade driving
      double y = locDriverJoyStick.getY(GenericHID.Hand.kLeft);
      double x = locDriverJoyStick.getX(GenericHID.Hand.kRight);
      locDriveTrain.arcadeDrive(
          Math.pow(y, 2) * Math.signum(y) * Constants.driveTrain.speedMult,
          Math.pow(x, 2) * Math.signum(x) * Constants.driveTrain.rotMult);
    } else {
      // tank driving
      double left = locDriverJoyStick.getX(GenericHID.Hand.kLeft);
      double right = locDriverJoyStick.getX(GenericHID.Hand.kRight);
      locDriveTrain.tankDrive(
          Math.pow(left, 2) * Math.signum(left) * Constants.driveTrain.speedMult,
          Math.pow(right, 2) * Math.signum(right) * Constants.driveTrain.speedMult);
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
