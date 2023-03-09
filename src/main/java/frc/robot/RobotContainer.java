package frc.robot;

import frc.robot.commands.LowPosition;
import frc.robot.commands.MidPosition;
import frc.robot.commands.TeleopPivot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.Constants.OperatorConstants;

public class RobotContainer {
  private PivotSubsystem pivot = new PivotSubsystem();
  private TeleopPivot manual = new TeleopPivot(pivot, 0.01);
  private LowPosition low = new LowPosition(pivot);
  private MidPosition mid = new MidPosition(pivot);
  private Joystick joystick = new Joystick(OperatorConstants.kDriverControllerPort);
  public RobotContainer(){
    configureBindings();
  }
  private void configureBindings() {
    new JoystickButton(joystick, 1).whileTrue(manual);
    new JoystickButton(joystick, 2).onTrue(low);
    new JoystickButton(joystick, 3).onTrue(mid);
  }
}
