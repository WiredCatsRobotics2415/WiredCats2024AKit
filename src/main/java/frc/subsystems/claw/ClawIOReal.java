package frc.subsystems.claw;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.constants.Subsystems.ClawConstants;

public class ClawIOReal implements ClawIO {
    private CANSparkMax motor;
    private SparkPIDController pidController;
    private RelativeEncoder relativeEncoder;

    public ClawIOReal() {
        motor = new CANSparkMax(ClawConstants.MotorID, CANSparkMax.MotorType.kBrushless);
        configureMotor();
    }

    public void configureMotor() {
        motor.restoreFactoryDefaults();
        // motor.setInverted(true);
        motor.setSmartCurrentLimit(ClawConstants.CurrentLimit);
        motor.setIdleMode(IdleMode.kBrake);

        relativeEncoder = motor.getEncoder();
        relativeEncoder.setPositionConversionFactor(1 / ClawConstants.GearRatio);

        pidController = motor.getPIDController();
        pidController.setFF(ClawConstants.Ks);
        pidController.setP(ClawConstants.Kp);
        pidController.setD(ClawConstants.Kd);
        pidController.setOutputRange(-ClawConstants.OutputExtrema, ClawConstants.OutputExtrema);
        // motor.setClosedLoopRampRate(0.5); //slows down claw
        // pidController.setPositionPIDWrappingEnabled(true);
        // pidController.setPositionPIDWrappingMaxInput(1);
        // pidController.setPositionPIDWrappingMinInput(-1);
    }

    @Override
    public void updateInputs(ClawIOInputsAutoLogged inputs) {
        inputs.position = relativeEncoder.getPosition();

        inputs.motorStatorCurrent = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();
        inputs.appliedVoltage = motor.getBusVoltage() * motor.getAppliedOutput(); // bro sparkmax api is so braindead
    }

    @Override
    public void setPosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
    }

    @Override
    public void setEncoderPosition(double position) {
        relativeEncoder.setPosition(position);
    }
}
