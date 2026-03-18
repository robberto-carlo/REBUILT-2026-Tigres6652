// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanism;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    private final TalonFX hoodMotor = new TalonFX(14);

    private final MotionMagicVoltage mm = new MotionMagicVoltage(0);
    private final DutyCycleOut duty = new DutyCycleOut(0);

    private double anguloObjetivo = 0.0;

    private static final double REDUCCION = 23.5714; 
    private static final double TOLERANCIA_GRADOS = 1.0;
    private static final double GRADOS_SAVE_HOOD = 2.0;

    private double KS = 0.7;
    private double KV = 0.03;
    private double KA = 0.026;
    private double KG = 0.025;

    private double KP = 16.0;
    private double KI = 0.0;
    private double KD = 0.0;

    private double VEL = 2.5;
    private double ACC = 8.0;
    private double JERK = 25.0;

    public Hood() {
        configurarMotor();
        resetEncoder();
        inicializarDashboard();
    }

    public void configurarMotor() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.Slot0.kS = KS;
        cfg.Slot0.kV = KV;
        cfg.Slot0.kA = KA;
        cfg.Slot0.kG = KG; 
        
        cfg.Slot0.kP = KP;
        cfg.Slot0.kI = KI;
        cfg.Slot0.kD = KD;

        cfg.MotionMagic.MotionMagicCruiseVelocity = VEL;
        cfg.MotionMagic.MotionMagicAcceleration = ACC;
        cfg.MotionMagic.MotionMagicJerk = JERK;

        /*cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = gradosAMotorRot(35);*/

        m_currentLimits.withSupplyCurrentLowerLimit(Amps.of(70)) // Default limit of 70 A
            .withSupplyCurrentLimit(Amps.of(40)) // Reduce the limit to 40 A if we've limited to 70 A...
            .withSupplyCurrentLowerTime(Seconds.of(0.5)) // ...for at least 1 second
            .withSupplyCurrentLimitEnable(true); // And enable it
        m_currentLimits.withStatorCurrentLimit(Amps.of(60)) // Limit stator current to 120 A
            .withStatorCurrentLimitEnable(true); // And enable it
        cfg.CurrentLimits = m_currentLimits;

        hoodMotor.getConfigurator().apply(cfg);
    }

    public void inicializarDashboard() {
        SmartDashboard.putNumber("Hood Target", anguloObjetivo);

        /*SmartDashboard.putNumber("KS", KS);
        SmartDashboard.putNumber("KV", KV);
        SmartDashboard.putNumber("KA", KA);
        SmartDashboard.putNumber("KG", KG);

        SmartDashboard.putNumber("KP", KP);
        SmartDashboard.putNumber("KI", KI);
        SmartDashboard.putNumber("KD", KD);

        SmartDashboard.putNumber("VEL", VEL);
        SmartDashboard.putNumber("ACC", ACC);
        SmartDashboard.putNumber("JERK", JERK);*/
    }

    private double gradosAMotorRot(double grados) {
        return (grados / 360.0) * REDUCCION;
    }

    private double lecturaGrados() {
        return hoodMotor.getPosition().getValueAsDouble() / REDUCCION * 360.0;
    }

    public void setAngle(double grados) {
        if(grados != -1){
            anguloObjetivo = grados;
        }

        hoodMotor.setControl(mm.withPosition(gradosAMotorRot(anguloObjetivo)));
    }

    public void movimientoFree(double control) {
        hoodMotor.setControl(duty.withOutput(control));
    }

    public void saveZone(){
        hoodMotor.setControl(mm.withPosition(gradosAMotorRot(GRADOS_SAVE_HOOD)));
    }

    public void preShoot(){
        hoodMotor.setControl(mm.withPosition(gradosAMotorRot(17)));
    }

    public boolean atSetpoint() {
        return Math.abs(lecturaGrados() - anguloObjetivo) <= TOLERANCIA_GRADOS;
    }

    public void resetEncoder() {
        anguloObjetivo=0;
        hoodMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        updateDashboard();
        SmartDashboard.putNumber("Hood Target", anguloObjetivo);
        SmartDashboard.putNumber("Hood Angle", lecturaGrados());
        SmartDashboard.putBoolean("Hood Setpoint", atSetpoint());
    }

    public void updateDashboard(){
        //anguloObjetivo = SmartDashboard.getNumber("Hood Target", anguloObjetivo);

        /*KS = SmartDashboard.getNumber("KS", KS);
        KV = SmartDashboard.getNumber("KV", KV);
        KA = SmartDashboard.getNumber("KA", KA);
        KG = SmartDashboard.getNumber("KG", KG);

        KP = SmartDashboard.getNumber("KP", KP);
        KI = SmartDashboard.getNumber("KI", KI);
        KD = SmartDashboard.getNumber("KD", KD);

        VEL = SmartDashboard.getNumber("VEL", VEL);
        ACC = SmartDashboard.getNumber("ACC", ACC);
        JERK = SmartDashboard.getNumber("JERK", JERK);*/
    }
}

