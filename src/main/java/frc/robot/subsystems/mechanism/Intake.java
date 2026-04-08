package frc.robot.subsystems.mechanism;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    // La variables VELOCIDAD_RODILLO es para la VELOCIDAD CONSTANTE del RODILLO DEL INTAKE al agarrar FUELS
    private static final double VELOCIDAD_RODILLO = 0.6; //0.30
    private static final double VELOCIDAD_RODILLO_PIVOTEO = 0.6; //0.45 - 0.7

    // Si van a calibrar PID de Intake de poscion, volver True, y veran en ShuffleBoard en los datos del PID del INTAKE DE POSICION
    private static final boolean INTAKE_CHANGE_PID = false; // NO TENER EN TRUE MAS DE UNO PARA SACAR PIDS;


    private final TalonFX motorPosicion = new TalonFX(18);
    private final TalonFX motorRodillo = new TalonFX(19);

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private final MotionMagicVoltage mmControl = new MotionMagicVoltage(0);
    private final DutyCycleOut duty = new DutyCycleOut(0);

    private static final double REDUCCION = 3.7698;
    private static final double TOLERANCIA_GRADOS = 5;

    private double KS = 0.45;
    private double KV = 0.25; // 0.12
    private double KA = 0.15;
    private double KG = 0.2;

    private double KP = 30.0;
    private double KI = 0.0;
    private double KD = 0.0;

    private double VEL = 55.0;
    private double ACC = 50.0; // Aumentarle a este valor (Probablemente)
    private double JERK = 160.0;

    private double posicionObjetivo = 0.0;
    private double posicionActual = 0.0;
    private boolean isPivoteDown = false;
    private boolean disableAutoPivote = false;


    public Intake() {
        configurarMotor();
        resetEncoder_Init();
        inicializarDashboard();
    }

    public void configurarMotor() {
        var cfgPosicion = new TalonFXConfiguration();
        CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();
        
        cfgPosicion.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgPosicion.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        cfgPosicion.Slot0.kS = KS; //para que arranque el voltaje 
        cfgPosicion.Slot0.kV = KV; // para mantener la velocidad constante = voltaje/rpm 
        cfgPosicion.Slot0.kA = KA; //  voltaje para acelerar = voltaje/(rps/s)
        cfgPosicion.Slot0.kG = KG; // 0.35  gravedad

        cfgPosicion.Slot0.kP = KP; // que tanta acelereacion hay dependiendo que tan lejos esta = voltaje/error
        cfgPosicion.Slot0.kI = KI;
        cfgPosicion.Slot0.kD = KD;

        cfgPosicion.MotionMagic.MotionMagicCruiseVelocity =  VEL; // 60; //cuantas rps se hacen
        cfgPosicion.MotionMagic.MotionMagicAcceleration = ACC;// 60; //que tantas rps por segundo hace
        cfgPosicion.MotionMagic.MotionMagicJerk = JERK; // 600; // que tan brusco acelera

        m_currentLimits.withSupplyCurrentLowerLimit(Amps.of(70)) // Default limit of 70 A
            .withSupplyCurrentLimit(Amps.of(45)) // Reduce the limit to 40 A if we've limited to 70 A...
            .withSupplyCurrentLowerTime(Seconds.of(0.5)) // ...for at least 1 second
            .withSupplyCurrentLimitEnable(true); // And enable it
        m_currentLimits.withStatorCurrentLimit(Amps.of(90)) // Limit stator current to 120 A
            .withStatorCurrentLimitEnable(true); // And enable it
        cfgPosicion.CurrentLimits = m_currentLimits;

        TalonFXConfiguration cfgRodillo = new TalonFXConfiguration();
        cfgRodillo.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfgRodillo.CurrentLimits = m_currentLimits;

        motorPosicion.getConfigurator().apply(cfgPosicion);
        motorRodillo.getConfigurator().apply(cfgRodillo);
    }

    public void inicializarDashboard() {
        SmartDashboard.putNumber("Intake Target", posicionObjetivo);
        if(INTAKE_CHANGE_PID){
            SmartDashboard.putNumber("KS", KS);
            SmartDashboard.putNumber("KV", KV);
            SmartDashboard.putNumber("KA", KA);
            SmartDashboard.putNumber("KG", KG);

            SmartDashboard.putNumber("KP", KP);
            SmartDashboard.putNumber("KI", KI);
            SmartDashboard.putNumber("KD", KD);

            SmartDashboard.putNumber("VEL", VEL);
            SmartDashboard.putNumber("ACC", ACC);
            SmartDashboard.putNumber("JERK", JERK);
        }
    }
    
    private double gradosAMotorRot(double grados) {
        return (grados / 360.0) * REDUCCION;
    }

    public double lecturaGrados() {
        double rotMotor = motorPosicion.getPosition().getValueAsDouble();
        double rotSalida = rotMotor / REDUCCION;
        return rotSalida * 360.0;
    }

    public double getAnguloObjetivo(int nivel) {
        switch (nivel) {
            case 0: return 0.0; // 0
            case 1: return 250.0; // 250 
            case 2: return 430.0; // 490 // bajar intake en el control
            case 3: return 448.0; // 490 // bajar intake en autonomo/automatizado
            default: return -1;
        }
    }

    public void move2Nivel(int nivel) {
        posicionObjetivo = getAnguloObjetivo(nivel);
        if (posicionObjetivo == -1) return;
        double rotObjetivo = gradosAMotorRot(posicionObjetivo);
        motorPosicion.setControl(mmControl.withPosition(rotObjetivo));
    }

    public void setAngle(double grados) {
        if(grados != -1){
            posicionObjetivo = grados;
        }
        motorPosicion.setControl(mmControl.withPosition(gradosAMotorRot(posicionObjetivo)));
    }

    public void movimientoFree(double control) {
        motorPosicion.setControl(duty.withOutput(control));
    }

    public boolean atSetpoint(){
        return Math.abs(posicionObjetivo - posicionActual) <= TOLERANCIA_GRADOS;
    }

    public double getRodillosRPM() {
        return motorRodillo.getVelocity().getValueAsDouble() * 60.0;
    }

    public void setRodillosRPM(double rpm) {
    double rps = rpm / 60.0;
    motorRodillo.setControl(velocityControl.withVelocity(rps));
    }

    public void stopRodillosRPM(){
        motorRodillo.stopMotor();
    }

    @Override
    public void periodic() {
        posicionActual = lecturaGrados();

        updateDashboard();
        SmartDashboard.putNumber("Intake Angle", posicionActual);
        SmartDashboard.putBoolean("Intake Setpoint", atSetpoint());
        SmartDashboard.putNumber("Rodillos RPM", getRodillosRPM());
    }

        public void updateDashboard(){
        if(INTAKE_CHANGE_PID){
            posicionObjetivo = SmartDashboard.getNumber("Intake Target", posicionObjetivo);
            KS = SmartDashboard.getNumber("KS", KS);
            KV = SmartDashboard.getNumber("KV", KV);
            KA = SmartDashboard.getNumber("KA", KA);
            KG = SmartDashboard.getNumber("KG", KG);

            KP = SmartDashboard.getNumber("KP", KP);
            KI = SmartDashboard.getNumber("KI", KI);
            KD = SmartDashboard.getNumber("KD", KD);

            VEL = SmartDashboard.getNumber("VEL", VEL);
            ACC = SmartDashboard.getNumber("ACC", ACC);
            JERK = SmartDashboard.getNumber("JERK", JERK);
        }else{
            SmartDashboard.putNumber("Intake Target", posicionObjetivo);
        }
    }

    public void stopMotorPosicion(){
        motorPosicion.setControl(duty.withOutput(0.0));
        //motorPosicion.setControl(mmControl.withPosition(motorPosicion.getPosition().getValueAsDouble()));
    }

    public void resetEncoder(){
        motorPosicion.setPosition(0);
    }

    private void resetEncoder_Init(){
        motorPosicion.setPosition(gradosAMotorRot(448));
    }
    public void activarRodillos(){
        motorRodillo.set(VELOCIDAD_RODILLO);
    }

    public void activarRodillosPivotear(){
        motorRodillo.set(VELOCIDAD_RODILLO_PIVOTEO);
    }

    public void invertirRodillos(){
        motorRodillo.set(-VELOCIDAD_RODILLO);
    }

    public void stopRodillos(){
        motorRodillo.set(0.0);
    }

    public void autoPivote(){ //PROBARLO SI NO FUNCIONA EN EL AUTNOMO, PROBAR ESTE
        if(!disableAutoPivote){
        activarRodillosPivotear();
        if(!isPivoteDown){
            move2Nivel(3);
        }else{
            move2Nivel(1);
        }
        if(atSetpoint()){
            isPivoteDown=!isPivoteDown;
        }
        }else{
            stopMotorPosicion();
            stopRodillos();
        }
    }
    

    public void setDisableAutoPivote(boolean disable){
        disableAutoPivote=disable;
    }

    public boolean isDisableAutoPivote(){
        return disableAutoPivote;
    }
}