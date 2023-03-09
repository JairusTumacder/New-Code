package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase{
    private TalonFX motor;
    private PIDController pid = new PIDController(PivotConstants.kp, PivotConstants.ki, PivotConstants.kd);
    private DigitalInput lowerLimit = new DigitalInput(PivotConstants.kLowerLimitPort);
    private DigitalInput upperLimit = new DigitalInput(PivotConstants.kUpperLimitPort);
    private boolean pidOn = true;
    private double setpoint;
    private double manualSpeed = 0;
    private double encoderValue;

    public PivotSubsystem(){
        motor = new TalonFX(PivotConstants.kTalonFXPort);
    }

    /* * * PID Methods * * */
    public void enablePID(){
        pidOn = true;
    }

    public void disablePID(){
        pidOn = false;
    }

    public boolean isPIDOn(){
        return pidOn;
    }

    public void newSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    /* * * Encoder Methods * * */
    public double getEncoder(){
        return motor.getSelectedSensorPosition();
    }

    public void resetEncoder(){
        motor.setSelectedSensorPosition(0);
    }

    public void currentEnctoSetpoint(){
        setpoint = getEncoder();
    }

    /* * * Pivot Movement Methods * * */
    public void manualPivot(double speed){
        motor.set(ControlMode.PercentOutput, speed);
    }

    public void pivotArm(double speed){
        motor.set(ControlMode.PercentOutput, speed);
    }

    public void stopPivot(){
        motor.set(ControlMode.PercentOutput, 0);
    }

    public void setManualSpeed(double inputSpeed){
        manualSpeed = inputSpeed;
    }

    /* * * Limit Switch Methods * * */
    public boolean isLowerLimitPressed(){
        return lowerLimit.get();
    }

    public boolean isUpperLimitPressed(){
        return upperLimit.get();
    }

    public boolean isAtSetpoint(){
        double error = setpoint - getEncoder();
        return Math.abs(error) < 5;
    }

    @Override
    public void periodic(){
        encoderValue = getEncoder();
        double calcSpeed = 0;

        if(pidOn){
            calcSpeed = pid.calculate(encoderValue, setpoint);
        }
        else{
            calcSpeed = manualSpeed;
        }
        if(calcSpeed > 0){
            calcSpeed = 0;
        }
        else if(calcSpeed < 0){
            calcSpeed = 0;
        }
        motor.set(ControlMode.PercentOutput, calcSpeed);

        SmartDashboard.putNumber("[P] Encoder", getEncoder());
        SmartDashboard.putNumber("[P] Setpoint", setpoint);
        SmartDashboard.putBoolean("[P] PID", isPIDOn());
        SmartDashboard.putBoolean("[P] LowerLimit", isLowerLimitPressed());
        SmartDashboard.putBoolean("[P] UpperLimit", isUpperLimitPressed());
    }

}
