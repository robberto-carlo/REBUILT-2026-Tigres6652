package frc.robot.subsystems.mechanism;

import com.revrobotics.RelativeEncoder;
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
    private static final boolean INVERTIR_INDEXER_MAIN = true;
    private static final boolean INVERTIR_INDEXER_BANDAS = false;

    private static final double VELOCIDAD_INDEXER_MAIN = 0.45; // 0.45
    private static final double VELOCIDAD_INDEXER_MAIN_INVERTIDA = 0.45; //0.20
    private static final double VELOCIDAD_INDEXER_BANDA = 0.45; // 0.35 
    private static final double VELOCIDAD_INDEXER_BANDA_INVERTIDA = 0.8; // 0.4
    private static final double VELOCIDAD_INDEXER_BANDA_INVERTIDA_INTAKE = 0.35; // 0.4


    private final SparkFlex indexerMotorMain = new SparkFlex(17, MotorType.kBrushless);
    private final SparkFlex indexerMotorBanda = new SparkFlex(15, MotorType.kBrushless);
    private final RelativeEncoder encoderMain = indexerMotorMain.getEncoder();
    private final RelativeEncoder encoderBanda = indexerMotorBanda.getEncoder();

    private boolean activoMain = false;
    private boolean activoBanda = false;

    //public Indexer(){}
    //////////////////////// FUNCIONES PARA EL MAIN MOTOR DEL INDEXER ////////////////////////
    public void activarMain() {
        indexerMotorMain.set(VELOCIDAD_INDEXER_MAIN * orientationMain());
        activoMain = true;
    }

    public void invertirMain() {
        indexerMotorMain.set(-VELOCIDAD_INDEXER_MAIN_INVERTIDA * orientationMain());
        activoMain = true;
    }

    public void desactivarMain() {
        indexerMotorMain.set(0.0);
        activoMain = false;
    }

    public double getMainRPM(){
        return encoderMain.getVelocity();
    }

    public boolean onMotorMain() {
        return activoMain;
    }

    //////////////////////// FUNCIONES PARA EL MOTOR DE LAS BANDAS DEL INDEXER ////////////////////////
    public void activarBandas() {
        indexerMotorBanda.set(VELOCIDAD_INDEXER_BANDA * orientationBandas());
        activoBanda = true;
    }

    public void invertirBandas() {
        indexerMotorBanda.set(-VELOCIDAD_INDEXER_BANDA_INVERTIDA * orientationBandas());
        activoBanda = true;
    }

    public void invertirBandasIntake() {
        indexerMotorBanda.set(-VELOCIDAD_INDEXER_BANDA_INVERTIDA_INTAKE * orientationBandas());
        activoBanda = true;
    }

    public void desactivarBandas() {
        indexerMotorBanda.set(0.0);
        activoBanda = false;
    }

    public double getBandaRPM(){
        return encoderBanda.getVelocity();
    }

    public boolean onMotorBanda() {
        return activoBanda;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Indexer Main Activo", activoMain);
        SmartDashboard.putBoolean("Indexer Banda Activo", activoBanda);
        SmartDashboard.putNumber("RPM Main", encoderMain.getVelocity());
        SmartDashboard.putNumber("RPM Banda", encoderBanda.getVelocity());
    }

    public int orientationMain() {
        if(INVERTIR_INDEXER_MAIN){
            return -1;
        }else{
            return 1;
        }
    }
    public int orientationBandas() {
        if(INVERTIR_INDEXER_BANDAS){
            return -1;
        }else{
            return 1;
        }
    }
}
