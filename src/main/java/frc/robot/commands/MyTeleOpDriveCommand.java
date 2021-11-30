// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls;
import frc.robot.Constants.driving;
import frc.robot.Constants.driving.speeds;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.Encoders.Target;

public class MyTeleOpDriveCommand extends CommandBase {
  /** Creates a new MyTeleOpDriveCommand. */
  Drivetrain locDriveTrain;
  Controls controls = Controls.getInstance();

  private NetworkTableEntry moveSpeed = NetworkTableInstance.getDefault().getTable("SmartDashboard")
      .getEntry("move speed (0 to 1)");
  private NetworkTableEntry moveSpeedFast = NetworkTableInstance.getDefault().getTable("SmartDashboard")
      .getEntry("fast move speed (0 to 1)");
  private NetworkTableEntry turnSpeed = NetworkTableInstance.getDefault().getTable("SmartDashboard")
      .getEntry("turn speed (0 to 1)");
  private NetworkTableEntry turnSpeedFast = NetworkTableInstance.getDefault().getTable("SmartDashboard")
      .getEntry("fast turn speed (0 to 1)");
  private NetworkTableEntry controllerPower = NetworkTableInstance.getDefault().getTable("SmartDashboard")
      .getEntry("controller exponent rounded down");
  private Target target;

  public MyTeleOpDriveCommand(Drivetrain driveTrain) {
    locDriveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moveSpeed.setDouble(speeds.normal.move);
    moveSpeedFast.setDouble(speeds.fast.move);
    turnSpeed.setDouble(speeds.normal.turn);
    turnSpeedFast.setDouble(speeds.fast.turn);
    controllerPower.setDouble(speeds.fast.turn);
  }

  private boolean tankMode = !driving.useArcadeControls;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPOV = controls.controller.getPOV();

    if (currentPOV != -1 && currentPOV != 360 && target == null) {
      target = locDriveTrain.encoders.new Target(locDriveTrain.encoders.new Mark(-currentPOV));
    }

    if (controls.controller.getAButtonPressed()) {
      tankMode = !tankMode;
    }

    speeds.normal.move = moveSpeed.getDouble(0);
    speeds.fast.move = moveSpeedFast.getDouble(0);
    speeds.normal.turn = turnSpeed.getDouble(0);
    speeds.fast.turn = turnSpeedFast.getDouble(0);
    driving.power = (int) controllerPower.getDouble(1);

    controls.adjustRumble();

    if (target != null) {
      if (target.hasReached()) {
        target = null;
        return;
      }

      double lDist = target.getLeftDistance();
      double rDist = target.getRightDistance();

      double base = 0.4;
      double mult = 1.0 - base;
      double scale = 0.3;

      locDriveTrain.tankDrive(((1 - 1.0 / (1 + Math.abs(lDist) / scale)) * mult + base) * Math.signum(lDist),
          ((1 - 1.0 / (1 + Math.abs(rDist) / scale)) * mult + base) * Math.signum(rDist));

      return;
    }

    if (!tankMode) {
      // arcade driving
      locDriveTrain.arcadeDrive(controls.arcade.getMove(), controls.arcade.getTurn());
    } else {
      // tank driving
      locDriveTrain.tankDrive(controls.tank.getLeft(), controls.tank.getRight());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      locDriveTrain.arcadeDrive(0.0, 0.0);
    target = null;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
