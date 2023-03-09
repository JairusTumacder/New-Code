package frc.robot;

import frc.robot.commands.PID;
import frc.robot.commands.PID2;
import frc.robot.commands.TeleopPivot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.Constants.OperatorConstants;

public class RobotContainer {
  private PivotSubsystem pivot = new PivotSubsystem();
  private TeleopPivot manual = new TeleopPivot(pivot, 0.01);
  private PID pid = new PID(pivot);
  private PID2 pid2 = new PID2(pivot);
  private Joystick joystick = new Joystick(OperatorConstants.kDriverControllerPort);
  public RobotContainer(){
    configureBindings();
  }
  private void configureBindings() {
    new JoystickButton(joystick, 1).whileTrue(manual);
    new JoystickButton(joystick, 2).onTrue(pid);
    new JoystickButton(joystick, 3).onTrue(pid2);
  }
}
