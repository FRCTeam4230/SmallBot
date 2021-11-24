// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls;
import frc.robot.Constants.driving;
import frc.robot.subsystems.Drivetrain;

public class MyTeleOpDriveCommand extends CommandBase {
  /** Creates a new MyTeleOpDriveCommand. */
  Drivetrain locDriveTrain;
  Controls controls = Controls.getInstance();

  public MyTeleOpDriveCommand(Drivetrain driveTrain) {
    locDriveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  private boolean tankMode = !driving.useArcadeControls;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.controller.getAButtonPressed()) {
      tankMode = !tankMode;
    }

    controls.adjustRumble();

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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
