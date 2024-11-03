package frc.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.constants.Hardware;
import frc.constants.Hardware.IntakePorts;
import frc.constants.Subsystems;
import frc.constants.Subsystems.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
    private TalonFX motor;
    private AnalogInput closeToFlywheelSensor;

    private StatusSignal<Double> motorStatorCurrent = motor.getStatorCurrent();
    private StatusSignal<Double> motorSupplyCurrent = motor.getSupplyCurrent();
    private StatusSignal<Double> motorTemp = motor.getDeviceTemp();

    public IntakeIOReal() {
        motor = new TalonFX(IntakePorts.IntakeMotorID);
        BaseStatusSignal.setUpdateFrequencyForAll(50, motorStatorCurrent, motorSupplyCurrent, motorTemp);
        configureMotor();

        closeToFlywheelSensor = new AnalogInput(IntakePorts.FlywheelIR);
    }

    private void configureMotor() {
        motor.optimizeBusUtilization();
        motor.setInverted(Hardware.TalonFXDirectionCounterClockWise);
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.getConfigurator().apply(IntakeConstants.CurrentLimit);
        // motor.getConfigurator().apply(new
        // ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0.2));
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(motorStatorCurrent, motorSupplyCurrent, motorTemp);
        inputs.sensorTrigger = closeToFlywheelSensor.getValue() < Subsystems.IntakeConstants.IRThreshold;

        inputs.motorConnected = true;
        inputs.motorStatorCurrent = motorStatorCurrent.getValue();
        inputs.motorSupplyCurrent = motorSupplyCurrent.getValue();
        inputs.motorTemp = motorTemp.getValue();
    }

    @Override
    public void on(double speed) {
        motor.set(speed);
    }

    @Override
    public void off() {
        motor.set(0);
    }
}
