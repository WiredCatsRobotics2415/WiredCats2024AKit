package frc.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.constants.Hardware;
import frc.constants.Hardware.FlywheelPorts;
import frc.constants.Subsystems.FlywheelConstants;

public class FlywheelIOReal implements FlywheelIO {
    private TalonFX left;
    private TalonFX right;
    private VelocityVoltage voltageVelocity;

    private StatusSignal<Double> leftRotorVelocity = left.getRotorVelocity();
    private StatusSignal<Double> leftTemp = left.getDeviceTemp();
    private StatusSignal<Double> leftStatorCurrent = left.getStatorCurrent();
    private StatusSignal<Double> leftSupplyCurrent = left.getSupplyCurrent();
    private StatusSignal<Double> leftAppliedVolts = left.getMotorVoltage();

    private StatusSignal<Double> rightRotorVelocity = right.getRotorVelocity();
    private StatusSignal<Double> rightTemp = right.getDeviceTemp();
    private StatusSignal<Double> rightStatorCurrent = right.getStatorCurrent();
    private StatusSignal<Double> rightSupplyCurrent = right.getSupplyCurrent();
    private StatusSignal<Double> rightAppliedVolts = right.getMotorVoltage();

    public FlywheelIOReal() {
        left = new TalonFX(FlywheelPorts.LeftMotorID);
        right = new TalonFX(FlywheelPorts.RightMotorID);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                leftRotorVelocity,
                leftTemp,
                leftStatorCurrent,
                leftSupplyCurrent,
                leftAppliedVolts,
                rightRotorVelocity,
                rightTemp,
                rightStatorCurrent,
                rightSupplyCurrent,
                rightAppliedVolts);
        voltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
        configMotors();
        left.optimizeBusUtilization();
        right.optimizeBusUtilization();
    }

    private void configMotors() {
        TalonFXConfigurator rightCfg = right.getConfigurator();
        rightCfg.apply(FlywheelConstants.RightMotorPID);
        rightCfg.apply(FlywheelConstants.CoastConfig);
        rightCfg.apply(FlywheelConstants.CurrentLimits);
        right.setInverted(Hardware.TalonFXDirection_CounterClockWise);

        TalonFXConfigurator leftCfg = left.getConfigurator();
        leftCfg.apply(FlywheelConstants.LeftMotorPID);
        leftCfg.apply(FlywheelConstants.CoastConfig);
        leftCfg.apply(FlywheelConstants.CurrentLimits);
        left.setInverted(Hardware.TalonFXDirection_ClockWise);
    }

    @Override
    public void updateInputs(FlywheelIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(
                leftRotorVelocity,
                leftTemp,
                leftStatorCurrent,
                leftSupplyCurrent,
                leftAppliedVolts,
                rightRotorVelocity,
                rightTemp,
                rightStatorCurrent,
                rightSupplyCurrent,
                rightAppliedVolts);

        inputs.leftConnected = left.isAlive();
        inputs.leftVelocity = FlywheelConstants.rpsToRPM(leftRotorVelocity.getValue());
        inputs.leftTemp = leftTemp.getValue();
        inputs.leftStatorCurrent = leftStatorCurrent.getValue();
        inputs.leftSupplyCurrent = leftSupplyCurrent.getValue();
        inputs.leftAppliedVolts = leftAppliedVolts.getValue();

        inputs.rightConnected = right.isAlive();
        inputs.rightVelocity = FlywheelConstants.rpsToRPM(rightRotorVelocity.getValue());
        inputs.rightTemp = rightTemp.getValue();
        inputs.rightStatorCurrent = rightStatorCurrent.getValue();
        inputs.rightSupplyCurrent = rightSupplyCurrent.getValue();
        inputs.rightAppliedVolts = rightAppliedVolts.getValue();
    }

    @Override
    public void setRPM(double leftRPM, double rightRPM) {
        left.setControl(voltageVelocity.withVelocity(FlywheelConstants.rpmToRPS(leftRPM)));
        right.setControl(voltageVelocity.withVelocity(FlywheelConstants.rpmToRPS(rightRPM)));
    }
}
