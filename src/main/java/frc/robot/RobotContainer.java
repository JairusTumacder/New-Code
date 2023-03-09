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
  private TeleopPivot manualUp = new TeleopPivot(pivot, 0.01);
  private TeleopPivot manualDown = new TeleopPivot(pivot, -0.01);
  private LowPosition low = new LowPosition(pivot);
  private MidPosition mid = new MidPosition(pivot);
  private Joystick joystick = new Joystick(OperatorConstants.kDriverControllerPort);
  public RobotContainer(){
    configureBindings();
  }
  private void configureBindings() {
    new JoystickButton(joystick, joystick.getPOV(0)).whileTrue(manualUp);
    new JoystickButton(joystick, joystick.getPOV(180)).whileTrue(manualDown);
    new JoystickButton(joystick, 1).onTrue(low);
    new JoystickButton(joystick, 2).onTrue(mid);
  }
}
