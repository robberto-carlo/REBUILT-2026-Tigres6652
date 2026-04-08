package frc.robot.commands;

import java.util.function.Supplier;
//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.autoaim.AutoAim;
import frc.robot.subsystems.mechanism.Hood;
import frc.robot.subsystems.mechanism.Indexer;
import frc.robot.subsystems.mechanism.Outake;

public class ShooterCommand extends Command {
    // La variables MAX_OUTPUT_HOOD es para el LIMITE de VELOCIDAD MANUAL de la CAPUCHA
    private static final double MAX_OUTPUT_HOOD = 0.15; // 0.15
    //Si van a calibrar PID de OUTAKE o CAPUCHA de poscion --- O VAN A HACER LA TABLA DE DISPAROS, volver True 
    //al usar el boton B, ahora los mecanismos usaran el nuevo PID, independientemente si estas calirbado CAPUCHA o OUTAKE
    //el boton X mueve la capucha a el angulo que le pongas, y el boton Y se mueve a los RPMs que le pongas en el traget en el SHUFFLEBOARD
    private static final boolean CALIBRAR_AUTO_SHOOT = false; // FALSE SI VAMOS A IR A MATCH


    // RT -ya etsa - agreagr funvion 100rpm
    // A - ya esta - boton de fileo
    // jsotik izq presionado -. invetir indexer 
    // ohsotik der -a aciavr indexer 

    private final Outake outake;
    private final Hood hood;
    private final Indexer indexer;
    private final AutoAim autoAim;
    private final Supplier<Boolean> botonLB;// LB = Bloquar Indexer
    private Supplier<Boolean> povUp_1Driver, povDown_1Driver, resetHood, botonY, botonX, botonA, botonB, botonRB, povUp_2Driver;
    private Supplier<Boolean> botonJostikcDer, botonJostikcIzq;

    private Supplier<Double> joystickRightY, gatilloLT, gatilloRT;

    private static final double INDEXER_ACTIVATION_THRESHOLD = 0.2; 
    private static final double DEADZONE_HOOD = 0.05; 

    private double targetRPM = -1;
    private double targetAngle = -1;
    private boolean resetOld = false;

    // distancia del Shooter al HUB
    public static final double[] DISTANCE = { // 1.3
        1.0,
        1.5,
        1.7,
        2.0,
        2.3,
        2.65,
        2.97,
        3.2,
        3.4,
        3.8,
        4.1,
        4.6
    };

    // RPM del shooter
    public static final double[] RPM = {
        2500,
        2600,
        2700,
        2700,
        2800,
        2800,  
        2800,
        3000,
        3000,
        3200,
        3200,
        3500
    };

    // Ángulo del hood (grados)
    public static final double[] ANGLE = {
        4, 
        7, 
        8,
        9,
        11,
        12, 
        15, 
        17,
        18,
        20,
        20,
        23
    };

    public ShooterCommand(
            Outake outake,
            Hood hood,
            Indexer indexer, 
            AutoAim autoAim,
            Supplier<Double> joystickRightY,
            Supplier<Boolean> povUp_1Driver,
            Supplier<Boolean> povDown_1Driver,
            Supplier<Boolean> resetHood,
            Supplier<Boolean> botonY,
            Supplier<Boolean> botonX,
            Supplier<Boolean> botonA,
            Supplier<Boolean> botonB,
            Supplier<Boolean> botonRB,
            Supplier<Double> gatilloLT,
            Supplier<Boolean> povUp_2Driver,
            Supplier<Boolean> botonLB,
            Supplier<Double> gatilloRT,
            Supplier<Boolean> botonJostikcDer,
            Supplier<Boolean> botonJostikcIzq){ 
        this.outake = outake;
        this.hood = hood;
        this.indexer = indexer;
        this.autoAim = autoAim;
        this.joystickRightY = joystickRightY;
        this.povUp_1Driver = povUp_1Driver;
        this.povDown_1Driver = povDown_1Driver;
        this.resetHood = resetHood;
        this.botonY = botonY;
        this.botonX = botonX;
        this.botonA = botonA;
        this.botonB = botonB;
        this.botonRB = botonRB;
        this.gatilloLT = gatilloLT;
        this.povUp_2Driver = povUp_2Driver;
        this.botonLB = botonLB;
        this.gatilloRT = gatilloRT;
        this.botonJostikcDer = botonJostikcDer;
        this.botonJostikcIzq = botonJostikcIzq;
        addRequirements(hood, outake, indexer);
    }

    @Override
    public void execute(){
        if(povUp_1Driver.get() || povDown_1Driver.get()){
            hood.saveZone(); // Bajar capucha
            return;
        }

        boolean Shoot = true;
        if(botonB.get()){ // Disparo Automatico 
            if(CALIBRAR_AUTO_SHOOT){
                hood.configurarMotor();
                outake.configurarMotor();
            }else{
                double distancia2HUB = autoAim.getDistanceToHub();
                targetRPM = getRPMFromDistance(distancia2HUB);
                targetRPM = Math.round(targetRPM); // volver el RPM un numero entero
                targetAngle = getAngleFromDistance(distancia2HUB);
                targetAngle = Math.round(targetAngle * 10.0) / 10.0; // redondear el angulo a un decimal

                if(gatilloRT.get() >= INDEXER_ACTIVATION_THRESHOLD){
                    targetRPM += 100; // Aumentar RPM para disparo
                }
                if(botonY.get()){
                    targetRPM += 200; // Aumentar RPM para disparo
                }

                outake.setRPM(targetRPM);
                hood.setAngle(targetAngle);
                activateIndexer();
            }
        }else if(botonA.get()){ // Disparar a nuestra Alianza
            if(CALIBRAR_AUTO_SHOOT){
                outake.setRPM(targetRPM);
            }else{
                hood.setAngle(25);
                outake.setRPM(3400);
                activateIndexer();
            }
        }else if(botonX.get()){ // Disparo Cercano
            if(CALIBRAR_AUTO_SHOOT){
                hood.setAngle(targetAngle);
            }else{
                hood.setAngle(9); // 8,9,11
                outake.setRPM(2700);
                activateIndexer();   
            }
        }else if(povUp_2Driver.get()){  // Disparo Lejano 
            hood.setAngle(15); 
            outake.setRPM(2900);
            activateIndexer();            
        }else{
            Shoot = false;
            outake.stop();
            if(!CALIBRAR_AUTO_SHOOT) hood.saveZone();
        }

        if(botonLB.get()){
            indexer.desactivarMain();
            indexer.desactivarBandas();
        }else if(botonJostikcDer.get()){
            indexer.activarMain();
            indexer.activarBandas();
        }else if(gatilloLT.get()>=INDEXER_ACTIVATION_THRESHOLD && !Shoot){
            indexer.invertirBandas();
        }else if(botonJostikcIzq.get()){
            indexer.invertirMain();
            indexer.invertirBandas();
        }else if(!Shoot){
            indexer.desactivarMain();
            indexer.desactivarBandas();
        }

        if(botonRB.get()){
            double joystick = deadband(joystickRightY.get(), DEADZONE_HOOD);
            hood.movimientoFree(-joystick*MAX_OUTPUT_HOOD);
        }
        
        boolean reset = resetHood.get();
        if (reset && !resetOld) {
            hood.setAngle(0);
            hood.resetEncoder();
        }
        resetOld = reset;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.desactivarMain();
        indexer.desactivarBandas();
        outake.stop();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    private double interpolate(
        double x,
        double x1, double y1,
        double x2, double y2
    ){
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }

    private double getRPMFromDistance(double distance) {
        for (int i = 0; i < DISTANCE.length - 1; i++) {
            if (distance >= DISTANCE[i] && distance <= DISTANCE[i + 1]) {
                return interpolate(
                    distance,
                    DISTANCE[i], RPM[i],
                    DISTANCE[i + 1], RPM[i + 1]
                );
            }
        }
        return RPM[RPM.length - 1];
    }

    private double getAngleFromDistance(double distance) {
        for (int i = 0; i < DISTANCE.length - 1; i++) {
            if (distance >= DISTANCE[i] && distance <= DISTANCE[i + 1]) {
                return interpolate(
                    distance,
                    DISTANCE[i], ANGLE[i],
                    DISTANCE[i + 1], ANGLE[i + 1]
                );
            }
        }
        return ANGLE[ANGLE.length - 1];
    }

    private double deadband(double value, double db) {
        if (Math.abs(value) < db) {
            return 0.0;
        }
        return value;
    }

    private void activateIndexer() {
        if(hood.atSetpoint() && outake.atSetpoint()){
            indexer.activarMain();
            indexer.activarBandas();
        }else{
            indexer.desactivarMain();
            indexer.desactivarBandas();
        }
    }
}