package frc.robot.commands;

import java.util.function.Supplier;
//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.autoaim.AutoAim;
import frc.robot.subsystems.mechanism.Hood;
import frc.robot.subsystems.mechanism.Indexer;
import frc.robot.subsystems.mechanism.Outake;

public class ShooterCommand extends Command {
    private final Outake outake;
    private final Hood hood;
    private final Indexer indexer;
    private final AutoAim autoAim;
    private Supplier<Boolean> povUp_1Driver, povDown_1Driver, resetHood, botonY, botonX, botonA, botonB, botonRB, povUp_2Driver, povDown_2Driver;
    private Supplier<Double> joystickRightY, gatilloLT, gatilloRT;

    private static final double INTAKE_ACTIVATION_THRESHOLD = 0.2; 
    private static final double DEADZONE_HOOD = 0.05; 
    private static final double MAX_OUTPUT_HOOD = 0.15; 
    //private static final double ROBOT2SHOOTER = 0.156;

    private double targetRPM = -1;
    private double targetAngle = -1;
    private boolean resetOld = false;

    // distancia del Shooter al HUB
    public static final double[] DISTANCE = {
        1.0,
        1.3,
        1.9,
        2.1,
        2.3,
        2.5,
        2.9,
        3.0,
        3.3,
        3.6, 
        4.3,
        4.7
    };

    // RPM del shooter
    public static final double[] RPM = {
        1650,
        1650,
        1650, 
        1650,  
        1750,
        1750,
        1850, 
        1850,
        1950,
        1950,
        2150,
        2300
    };

    // Ángulo del hood (grados)
    public static final double[] ANGLE = {
        5.5, 
        9, 
        11, 
        15, 
        16, 
        17,
        19, 
        21,
        22,
        25,
        23,
        26
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
            Supplier<Boolean> povDown_2Driver,
            Supplier<Double> gatilloRT){
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
        this.povDown_2Driver = povDown_2Driver;
        this.gatilloRT = gatilloRT;
        addRequirements(hood, outake, indexer);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute(){
        if(povUp_1Driver.get() || povDown_1Driver.get() || povDown_2Driver.get()){
            hood.saveZone();
            return;
        }

        if(botonB.get()){
            double distancia2HUB = autoAim.getDistanceToHub();
            targetRPM = getRPMFromDistance(distancia2HUB);
            targetAngle = getAngleFromDistance(distancia2HUB);

            hood.setAngle(targetAngle);
            outake.setRPM(targetRPM);
            
            if(hood.atSetpoint() && outake.atSetpoint()){
                indexer.activarMainPorcentaje();
                indexer.activarBandaPorcentaje();
            }else{
                indexer.desactivarMainPorcentaje();
                indexer.desactivarBandaPorcentaje();
            }
            return;
        }

        if(botonY.get()){
            hood.setAngle(17);
            outake.setRPM(1700);
        }else if(botonX.get()){
            hood.setAngle(9);
            outake.setRPM(1650);
        }else if(povUp_2Driver.get()){
            hood.setAngle(25); 
            outake.setRPM(1800);
            
            if(hood.atSetpoint() && outake.atSetpoint()){
                indexer.activarMainPorcentaje();
                indexer.activarBandaPorcentaje();
            }else{
                indexer.desactivarMainPorcentaje();
                indexer.desactivarBandaPorcentaje();
            }
        }else if(!botonB.get()){
            outake.stop();
        }

        /*if(botonY.get()){ // Botones para pruebas (para la tabla de rpms y angulos)
            outake.setRPM(targetRPM);
        }else if(!botonB.get()){
            outake.stop();
        } 
        if(botonX.get()){ 
            hood.setAngle(targetAngle);
        }*/

        if(botonA.get()){
            indexer.activarMainPorcentaje();
            indexer.activarBandaPorcentaje();
        }else if(gatilloLT.get()>=INTAKE_ACTIVATION_THRESHOLD){
            indexer.invertirBandaPorcentaje();
        }else if(gatilloRT.get()>=INTAKE_ACTIVATION_THRESHOLD){
            indexer.invertirMainPorcentaje();
        }else if(!botonB.get()){
            indexer.desactivarMainPorcentaje();
            indexer.desactivarBandaPorcentaje();
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
        indexer.desactivarMainPorcentaje();
        indexer.desactivarBandaPorcentaje();
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
}