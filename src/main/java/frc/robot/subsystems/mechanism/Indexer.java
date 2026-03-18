package frc.robot.subsystems.mechanism;

/*import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;*/
//import com.revrobotics.spark.FeedbackSensor;
//import com.revrobotics.spark.SparkBase.ControlType;
//import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    private final SparkFlex indexerMotorMain = new SparkFlex(15, MotorType.kBrushless);
    private final SparkFlex indexerMotorBanda = new SparkFlex(17, MotorType.kBrushless);

    /*private final SparkFlexConfig configMain = new SparkFlexConfig();
    private final SparkClosedLoopController pidMain = indexerMotorMain.getClosedLoopController();
    private final RelativeEncoder encoderMain = indexerMotorMain.getEncoder(); */

    private static final double VELOCIDAD_INDEXER_MAIN = 0.45; // 0.45
    private static final double VELOCIDAD_INDEXER_BANDA = 0.35; // 0.35 
    private static final double VELOCIDAD_INDEXER_MAIN_INVERTIDA = 0.20; //0.20
    private static final double VELOCIDAD_INDEXER_BANDA_INVERTIDA = 0.10; // 0.10
    
    private boolean activoMain = false;
    private boolean activoBanda = false;
    //private double targetRPM = 0.0;

    /*private static final double RPM_TOLERANCE = 100;
    private double KP = 0.0001; //0.0002 
    private double KI = 0.0;
    private double KD = 0.0;
    private double KF = 0.00019; //0.00017 */
    public Indexer(){
        configurarMotor();
        inicializarDashboard();
    }

    public void configurarMotor(){
        //configMain.inverted(true); 

        /*configMain.closedLoop
            //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(KP).i(KI).d(KD)
            .velocityFF(KF);

        indexerMotorMain.configure(
            configMain,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );*/
    }

    public void inicializarDashboard() {
        //SmartDashboard.putNumber("Indexer Target", targetRPM);

        /*SmartDashboard.putNumber("KP", KP);
        SmartDashboard.putNumber("KI", KI);
        SmartDashboard.putNumber("KD", KD);
        SmartDashboard.putNumber("KF", KF);*/
    }


    public void activarMainPorcentaje() {
        indexerMotorMain.set(VELOCIDAD_INDEXER_MAIN);
        activoMain = true;
    }

    public void invertirMainPorcentaje() {
        indexerMotorMain.set(-VELOCIDAD_INDEXER_MAIN_INVERTIDA);
        activoMain = true;
    }

    public void desactivarMainPorcentaje() {
        indexerMotorMain.set(0.0);
        activoMain = false;
    }

    public boolean onMotorMain() {
        return activoMain;
    }

//////////////////// PID ////////////////////

    /*public void activarMainPID() {
        pidMain.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        activoMain = true;
    }

    public void invertirMainPID() {
        pidMain.setReference(1200, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        activoMain = true;
    }

    public void desactivarMainPID() {
        pidMain.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        activoMain = false;
    }

    public boolean atSetpoint() {
        return Math.abs(encoderMain.getVelocity() - targetRPM) <= RPM_TOLERANCE;
    }*/

////////////////////////////////////////////////



    public void activarBandaPorcentaje() {
        indexerMotorBanda.set(VELOCIDAD_INDEXER_BANDA);
        activoBanda = true;
    }

    public void invertirBandaPorcentaje() {
        indexerMotorBanda.set(-VELOCIDAD_INDEXER_BANDA_INVERTIDA);
        activoBanda = true;
    }

    public void desactivarBandaPorcentaje() {
        indexerMotorBanda.set(0.0);
        activoBanda = false;
    }

    public boolean onMotorBanda() {
        return activoBanda;
    }

    @Override
    public void periodic() {
        updateDashboard();
        //SmartDashboard.putNumber("Indexer Target", targetRPM);
        //SmartDashboard.putNumber("Indexer Main RPM", encoderMain.getVelocity());
        
        //SmartDashboard.putBoolean("Indexer Main Activo", atSetpoint());        
        SmartDashboard.putBoolean("Indexer Main Activo", activoMain);
        SmartDashboard.putBoolean("Indexer Banda Activo", activoBanda);
    }

    public void updateDashboard(){
        /*targetRPM = SmartDashboard.getNumber("Indexer Target", targetRPM);

        KP = SmartDashboard.getNumber("KP", KP);
        KI = SmartDashboard.getNumber("KI", KI);
        KD = SmartDashboard.getNumber("KD", KD);
        KF = SmartDashboard.getNumber("KF", KF);*/
    }
}
