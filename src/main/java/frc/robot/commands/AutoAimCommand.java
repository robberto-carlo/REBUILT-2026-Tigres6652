package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.autoaim.AutoAim;

public class AutoAimCommand extends Command {
  private final AutoAim autoAim;
  private final DoubleSupplier joyX, joyY, joyRot;
  private final int direction;

  public AutoAimCommand(
      AutoAim autoAim,
      int direccion, 
      DoubleSupplier joyX,
      DoubleSupplier joyY,
      DoubleSupplier joyRot) {
    this.autoAim = autoAim;
    this.direction = direccion;
    this.joyX = joyX;
    this.joyY = joyY;
    this.joyRot = joyRot;
    addRequirements(autoAim);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute(){
    autoAim.getControllerInputs(
    deadband(joyX.getAsDouble()),
    deadband(joyY.getAsDouble()),
    deadband(joyRot.getAsDouble()));

    if(direction == 0 ){ // direccion = 0 --> a apuntar al HUB
      autoAim.setIsTargetHUB(true);
      autoAim.setIsOnlyRotate(false);
      autoAim.onlyRotateToTarget(direction);
    }else if(direction == 5){ // direccion = 5 --> a apuntar hacia la nuestra alinza
      autoAim.setIsTargetHUB(false);
      autoAim.setIsOnlyRotate(true);
      autoAim.onlyRotateToTarget(direction);
    }else {
      autoAim.setIsTargetHUB(false);
      autoAim.setIsOnlyRotate(false);
      autoAim.moveToTarget(direction);
    }
  }

  private double deadband(double value) {
  return Math.abs(value) < 0.05 ? 0 : value;
}

  @Override
  public void end(boolean interrupted) {
    autoAim.stop();
    autoAim.setIsTargetHUB(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
