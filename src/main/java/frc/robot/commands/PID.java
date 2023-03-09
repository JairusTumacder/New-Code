// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;

public class PID extends CommandBase {
  private PivotSubsystem pivotSubsystem;
  private double setpoint;
  public PID(PivotSubsystem pivotSubs) {
    pivotSubsystem = pivotSubs;
    addRequirements(pivotSubs);
    setpoint = 0;

  }
  @Override
  public void initialize() {
  }
  @Override
  public void execute() {
    pivotSubsystem.newSetpoint(setpoint);
  }
  @Override
  public void end(boolean interrupted) {
  }
  @Override
  public boolean isFinished() {
    return pivotSubsystem.isAtSetpoint();
  }
}
