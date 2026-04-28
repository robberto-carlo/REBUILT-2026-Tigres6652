package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanism.Intake;

import java.util.function.Supplier;

public class IntakeCommand extends Command {
  // La variables MAX_OUTPUT es para el LIMITE de VELOCIDAD MANUAL de la POSICION DEL INTAKE
  private static final double MAX_OUTPUT = 0.3; // 0.35 
  //Si van a calibrar PID de Intake de poscion, volver True, y el boton de RESETAER ENCODER DEL INTAKE, ahora el intake usara el nuevo PID
  private static final boolean INTAKE_CHANGE_PID = false; // NO TENER EN TRUE MAS DE UNO PARA SACAR PIDS;


  private final Intake intake;
  private final Supplier<Double> joystickLeftY;   // joystick izquierdo Y
  private final Supplier<Boolean> botonRB;// RB = manual
  private final Supplier<Boolean> povRight; // povArriba /flecha →
  private final Supplier<Boolean> povLeft;  // povAbajo /flecha (Flecha izq)
  private final Supplier<Boolean> povDown;  // povAbajo /flecha ↓
  private final Supplier<Boolean> botonReset;  // povReset / boton de en medio (3 lineas)
  private final Supplier<Double> gatilloLT; // activarRodillos /LT

  private boolean resetOld = false;
  private boolean pivoteOn = false;

  private static final int POSICION_ARRIBA = 0;
  private static final int POSICION_BAJA = 2;

  private static final double RODILLOS_ACTIVATION_THRESHOLD = 0.2; 
  private static final double DEADZONE_INTAKE = 0.05; 

  public IntakeCommand(
      Intake intake,
      Supplier<Double> joystickLeftY,
      Supplier<Boolean> botonRB,
      Supplier<Boolean> povRight,
      Supplier<Boolean> povLeft,
      Supplier<Boolean> povDown,
      Supplier<Boolean> botonReset,
      Supplier<Double> gatilloLT) {
    this.intake = intake;
    this.joystickLeftY = joystickLeftY;
    this.botonRB = botonRB;
    this.povRight = povRight;
    this.povLeft = povLeft;
    this.povDown = povDown;
    this.botonReset = botonReset;
    this.gatilloLT = gatilloLT;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    pivoteOn=false;
    if (povRight.get()) {
      intake.move2Nivel(POSICION_ARRIBA);
    }else if (povLeft.get()) {
      intake.move2Nivel(POSICION_BAJA);
    }else if(povDown.get()){
      intake.activarPivoteo();
      //intake.activarPivoteoLevantado();
      pivoteOn=true;
    }else {
      intake.stopMotorPosicion();
    }  
    
    if (botonRB.get()) {
      double joystick = deadband(joystickLeftY.get(), DEADZONE_INTAKE);
      intake.movimientoFree(joystick*MAX_OUTPUT);
    }

    boolean reset = botonReset.get();
    if (reset && !resetOld) {
      if(INTAKE_CHANGE_PID) intake.configurarMotor();
      intake.resetEncoder_down();
      intake.setAngle(0);
    }
    resetOld = reset;

    if (gatilloLT.get() > RODILLOS_ACTIVATION_THRESHOLD) {
      intake.activarRodillos();; 
    }else if(!pivoteOn){
      intake.stopRodillos();
    }

  }

  private double deadband(double value, double db) {
    if (Math.abs(value) < db) {
      return 0.0;
    }
    return value;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopMotorPosicion();
    intake.stopRodillos();
  }
}