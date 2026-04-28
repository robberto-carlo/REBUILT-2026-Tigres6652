package frc.robot.subsystems.mechanism;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outake extends SubsystemBase {
  private static final boolean CALIBRAR_AUTO_SHOOT = false; // USAR PARA PODER AGREGAR UN TARGET MANUAL A EL OUTAKE EN EL SHUFFLEBOARD
  // Si van a calibrar PID deL OUTAKE, volver True, y veran en ShuffleBoard en los datos del PID del OUTAKE
  private static final boolean OUTAKE_CHANGE_PID = false; // NO TENER EN TRUE MAS DE UNO PARA SACAR PIDS;
  private static final boolean IS_CONSTANT_RPM = false;  // Si es True, el outake se mantiene a ciertas RPMs, si es False, se detendra a 0 RPMs
  private static final double RPM_STOP_OUTAKE = 500; // Si es true, el outake se mantiene a estas RPMs que le pongas en esta variable


  private final TalonFX master = new TalonFX(13); // motor Izquierda
  private final TalonFX follower = new TalonFX(16); // motor Derecha

  private final VelocityVoltage velocityControl = new VelocityVoltage(0);

  private double targetRPM = 0.0;
  private double timeAtSpeed = 0.0;

  private static final double RPM_TOLERANCE = 100;
  private static final double STABLE_TIME = 0.10; // segundos 0.10

  private double KS = 0.3; // 0.35 - 0.5
  private double KV = 0.1; //0.12 - 0.115
  private double KA = 0.1; //0.004 - 0.2

  private double KP = 0.5; //0.9 - 0.8
  private double KI = 0.5; //0.0
  private double KD = 0.0; //0.0

  public Outake() {
    configurarMotor();
    resetEncoder();
    inicializarDashboard();
  }

  public void configurarMotor(){
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

    cfg.Slot0.kS = KS;
    cfg.Slot0.kV = KV;  
    cfg.Slot0.kA = KA; 
    cfg.Slot0.kP = KP;
    cfg.Slot0.kI = KI;
    cfg.Slot0.kD = KD;

    m_currentLimits.withSupplyCurrentLowerLimit(Amps.of(60)) // Default limit of 70 A
      .withSupplyCurrentLimit(Amps.of(40)) // Reduce the limit to 40 A if we've limited to 70 A...
      .withSupplyCurrentLowerTime(Seconds.of(0.5)) // ...for at least 1 second
      .withSupplyCurrentLimitEnable(true); // And enable it
    m_currentLimits.withStatorCurrentLimit(Amps.of(80)) // Limit stator current to 120 A
      .withStatorCurrentLimitEnable(true); // And enable it
    
    cfg.CurrentLimits = m_currentLimits;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    master.getConfigurator().apply(cfg);
    follower.getConfigurator().apply(cfg);

    follower.setControl(new Follower(master.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public void inicializarDashboard() {
    SmartDashboard.putNumber("Outake Target", targetRPM);
    if(OUTAKE_CHANGE_PID){
    SmartDashboard.putNumber("KS", KS);
    SmartDashboard.putNumber("KV", KV);
    SmartDashboard.putNumber("KA", KA);

    SmartDashboard.putNumber("KP", KP);
    SmartDashboard.putNumber("KI", KI);
    SmartDashboard.putNumber("KD", KD);
    }
  }

  public void setRPM(double rpm) {
    if(rpm != -1){
    targetRPM = Math.max(0, rpm);
    }
    double rps = targetRPM / 60.0;
    master.setControl(velocityControl.withVelocity(rps));
  }

  public void stop(){
    if(IS_CONSTANT_RPM){
      setRPM(RPM_STOP_OUTAKE);
    }else{
      master.stopMotor();
    }
  }

  public double getRPM() {
    return master.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getRPMError() {
    return Math.abs(targetRPM - getRPM());
  }

  public void resetEncoder() {
    master.setPosition(0);
  }

  public boolean atSetpoint() {
    return timeAtSpeed >= STABLE_TIME;
  }

  @Override
  public void periodic() {
    if (Math.abs(getRPMError()) <= RPM_TOLERANCE) {
      timeAtSpeed += 0.02; // 20ms
    }else {
      timeAtSpeed = 0.0;
    }

    updateDashboard();
    SmartDashboard.putNumber("Outake RPM", getRPM());
    SmartDashboard.putBoolean("Outake Setpoint", atSetpoint());
  }

  public void updateDashboard(){
    if(OUTAKE_CHANGE_PID){
      targetRPM = SmartDashboard.getNumber("Outake Target", targetRPM);
      KS = SmartDashboard.getNumber("KS", KS);
      KV = SmartDashboard.getNumber("KV", KV);
      KA = SmartDashboard.getNumber("KA", KA);
      KP = SmartDashboard.getNumber("KP", KP);
      KI = SmartDashboard.getNumber("KI", KI);
      KD = SmartDashboard.getNumber("KD", KD);
    }else if(CALIBRAR_AUTO_SHOOT){
      targetRPM = SmartDashboard.getNumber("Outake Target", targetRPM);
    }else{
      SmartDashboard.putNumber("Outake Target", targetRPM);
    }
  }
}