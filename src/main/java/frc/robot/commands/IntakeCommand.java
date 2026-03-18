package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanism.Intake;

import java.util.function.Supplier;

public class IntakeCommand extends Command {
  private final Intake intake;

  private final Supplier<Double> joystickLeftY;   // joystick izquierdo Y
  private final Supplier<Boolean> botonRB,botonLB;   // RB = manual
  private final Supplier<Boolean> povUp; // povArriba /flecha ↑
  private final Supplier<Boolean> povDown;  // povAbajo /flecha ↓
  private final Supplier<Boolean> povLeft;  // povReset /flecha →
  private final Supplier<Double> gatilloLT; // activarRodillos /LT

  private boolean resetOld = false;

  private static final int POSICION_ARRIBA = 0;
  private static final int POSICION_BAJA   = 1;

  private static final double RODILLOS_ACTIVATION_THRESHOLD = 0.2; 
  private static final double DEADZONE_INTAKE = 0.05; 
  private static final double MAX_OUTPUT = 0.15; 
  private static final double VELOCIDAD_RODILLO_RPM = 2500;


  public IntakeCommand(
      Intake intake,
      Supplier<Double> joystickLeftY,
      Supplier<Boolean> botonRB,
      Supplier<Boolean> povUp,
      Supplier<Boolean> povDown,
      Supplier<Boolean> povLeft,
      Supplier<Double> gatilloLT,
      Supplier<Boolean> botonLB) {
    this.intake = intake;
    this.joystickLeftY = joystickLeftY;
    this.botonRB = botonRB;
    this.povUp = povUp;
    this.povDown = povDown;
    this.povLeft = povLeft;
    this.gatilloLT = gatilloLT;
    this.botonLB = botonLB;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    if (povUp.get()) {
      intake.move2Nivel(POSICION_ARRIBA);
    } else if (povDown.get()) {
      intake.move2Nivel(POSICION_BAJA);
    } else if (botonRB.get()) {
      double joystick = deadband(joystickLeftY.get(), DEADZONE_INTAKE);
      joystick = limitOutput(joystick);
      intake.movimientoFree(-joystick);
    } else {
      intake.stopMotor();
    }

    boolean reset = povLeft.get();
    if (reset && !resetOld) {
      intake.configurarMotor(); // TEMPRORAL BORRAR DESPUES DE PRUEBAS
      intake.resetEncoder();
      intake.setAngle(0);
    }
    resetOld = reset;

    if (gatilloLT.get() > RODILLOS_ACTIVATION_THRESHOLD) {
      intake.activarRodillos();; 
    }else if(botonLB.get()){
      intake.invertirRodillos();
    }else {
      intake.stopRodillos();
    }
  }

  private double deadband(double value, double db) {
    if (Math.abs(value) < db) {
      return 0.0;
    }
    return value;
  }

    private double limitOutput(double joystick) {
    double abs = Math.abs(joystick);
    if(abs>=MAX_OUTPUT){
      return Math.copySign(MAX_OUTPUT,joystick);
    }
    return joystick;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
    intake.stopRodillos();
  }
}