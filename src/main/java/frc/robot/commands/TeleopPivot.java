// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;

public class TeleopPivot extends CommandBase {
  private PivotSubsystem pivotSubsystem;
  private double speed;

  public TeleopPivot(PivotSubsystem pivotSubs, double speed) {
    pivotSubsystem = pivotSubs;
    this.speed = speed;
    addRequirements(pivotSubs);
  }

  @Override
  public void initialize() {
    pivotSubsystem.disablePID();
  }

  @Override
  public void execute() {
    pivotSubsystem.setManualSpeed(speed);
  }
  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.stopPivot();
    pivotSubsystem.enablePID();
    pivotSubsystem.currentEnctoSetpoint();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
