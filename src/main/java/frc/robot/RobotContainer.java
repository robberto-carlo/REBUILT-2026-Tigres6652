package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.autoaim.AutoAim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.mechanism.Hood;
import frc.robot.subsystems.mechanism.Indexer;
import frc.robot.subsystems.mechanism.Outake;
import frc.robot.subsystems.mechanism.Intake;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.QuestNavSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  // Invertir Swerve   
  private final boolean SWERVE_EJE_X_INVERTIR = false; // eje X es adelante y atras
  private final boolean SWERVE_EJE_Y_INVERTIR = false; // eje Y es izq y der
  private final boolean SWERVE_ROT_INVERTIR = false; // invertir rotacion de swerve

  private final double LowVelocitySwerve = 0.8; // Porcentaje maximo de velocidad de swerve al Usar boton de bajar velocidad

  private final AutoAim autoAim;
  private final QuestNavSubsystem questNavSubsystem;
  private final LimelightSubsystem limelightFront; 

  private final Intake intake = new Intake();
  private final Outake outake = new Outake();
  private final Hood hood = new Hood();
  private final Indexer indexer = new Indexer();

  // Varibles para activar comandos en autonomos
  private boolean autoShootEnabled = false;
  private boolean pivotearEnabled = false;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick controller2 = new Joystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Meta
  public QuestNavSubsystem getQuestNavSubsystem() {
    return questNavSubsystem;
  }

  // Limelight
  public LimelightSubsystem getLimelightFront() {
    return limelightFront;
  }

  // Drive
  public Drive getDrive() {
    return drive;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    questNavSubsystem = new QuestNavSubsystem(drive.getPoseEstimator());
    drive.setQuestNavSubsystem(questNavSubsystem);
    limelightFront = new LimelightSubsystem(drive, "limelight-front"); 
    autoAim = new AutoAim(drive);

    // Comandos de Autonomos de pathplaner
    NamedCommands.registerCommand(
        "AutoShoot-On",
        Commands.runOnce(() -> autoShootEnabled = true));

    NamedCommands.registerCommand(
        "AutoShoot-Off",
        Commands.runOnce(() -> autoShootEnabled = false));

    NamedCommands.registerCommand(
        "Pivote-On",
        Commands.sequence(
            Commands.runOnce(() -> intake.resetAutoPivoteo()),
            new RunCommand(() -> intake.autoPivoteo(2), intake)
                .until(() -> intake.setpointAutoPivoteo())));

    NamedCommands.registerCommand(
        "Intake-On",
        new InstantCommand(() -> intake.activarRodillos(), intake));

    NamedCommands.registerCommand(
        "Intake-Off",
        new InstantCommand(() -> intake.stopRodillos(), intake));

    NamedCommands.registerCommand(
        "Up-Intake",
        new RunCommand(() -> intake.move2Nivel(0), intake)
            .until(() -> intake.atSetpoint())
            .withTimeout(3));

    NamedCommands.registerCommand(
        "Down-Intake",
        new RunCommand(() -> intake.move2Nivel(3), intake) 
            .until(() -> intake.atSetpoint())
            .withTimeout(3));

    NamedCommands.registerCommand(
        "AutoAim_Hub",
        new AutoAimCommand(
            autoAim,
            0,
            () -> 0.0,() -> 0.0,() -> 0.0)
            .until(() -> autoAim.autoAimHubFinished())
            .withTimeout(1)); //2s

    new EventTrigger("Event-Down-Intake").onTrue(
        new RunCommand(() -> intake.move2Nivel(3))
            .until(() -> intake.atSetpoint())
            .withTimeout(3));

    new EventTrigger("Event-Intake-On").onTrue(
        Commands.runOnce(() -> intake.activarRodillos()));

    new EventTrigger("Event-Intake-Off").onTrue(
        Commands.runOnce(() -> intake.stopRodillos()));

    // PROBAR ESTE EVENTO, CON VARIAS FUELS
    new EventTrigger("Event-IntakeConstDown")
    .whileTrue(
        Commands.run(() -> intake.move2Nivel(3))
    );

    /// Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // Red  +
    // Blue -
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * allianceFlip_ejeX() * changeVelocitySwerve(),
            () -> -controller.getLeftX() * allianceFlip_ejeY() * changeVelocitySwerve(),
            () -> -controller.getRightX() * allianceFlip_rot()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller.b().onTrue(
      Commands.runOnce(
        () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(),
              getAllianceHeading())), drive)
              .ignoringDisable(true));

    // Auto-apuntado
    controller.povUp().whileTrue(new AutoAimCommand(autoAim, 
                                      1,
                                      () -> -controller.getLeftY(),
                                      () -> -controller.getLeftX(),
                                      () -> -controller.getRightX())); // Flecha arriba - Ir a zona neutra

    controller.povDown().whileTrue(new AutoAimCommand(autoAim, 
                                      2,
                                      () -> -controller.getLeftY(),
                                      () -> -controller.getLeftX(),
                                      () -> -controller.getRightX())); // Flecha abajo - Ir a zona de alianza
    
    controller.rightBumper().whileTrue(new AutoAimCommand(autoAim, 
                                      0,
                                      () -> -controller.getLeftY() * allianceFlip_ejeX() * changeVelocitySwerve(),
                                      () -> -controller.getLeftX() * allianceFlip_ejeY() * changeVelocitySwerve(),
                                      () -> -controller.getRightX() * allianceFlip_rot())); // RB - auto apuntado al HUB

    controller.leftBumper().whileTrue(new AutoAimCommand(autoAim, 
                                      5,
                                      () -> -controller.getLeftY() * allianceFlip_ejeX() * changeVelocitySwerve(),
                                      () -> -controller.getLeftX() * allianceFlip_ejeY() * changeVelocitySwerve(),
                                      () -> -controller.getRightX() * allianceFlip_rot())); // LB - auto apuntado a Zona de ALIANZA

    outake.setDefaultCommand(
        new ShooterCommand(
            outake,
            hood,  
            indexer,
            autoAim,
            () -> controller2.getRawAxis(5),   // joystick derecho /eje Y
            () -> controller.povUp().getAsBoolean(), // pov arriba del Driver 1 / flecha ↑
            () -> controller.povDown().getAsBoolean(), // pov abajo del Driver 1 / flecha ↓
            () -> controller2.getRawButton(7), // Reset Hood / boton en medio (2 cuadros)
            () -> controller2.getRawButton(4), // Y -  Aumentar 200 RPMs Shooter 
            () -> controller2.getRawButton(3), // X - Disparo de poca distancia
            () -> controller2.getRawButton(1), // A - Disparo a Nuestra Alianza
            () -> controller2.getRawButton(2) || autoShootEnabled, // B - shooter automatico
            () -> controller2.getRawButton(6), // RB - BlqFree
            () -> controller2.getRawAxis(2), // LT - Bandas Indexer Invertidas
            () -> controller2.getPOV() == 0,   // Disparo de media distancia / flecha ↑
            () -> controller2.getRawButton(5), // LB - Bloquar Indexer
            () -> controller2.getRawAxis(3),  // RT - Aumentar 100 RPMs Shooter
            () -> controller2.getRawButton(10), // Jostikc Der Boton- Activar indexer  
            () -> controller2.getRawButton(9))); // Jostikc Izq Boton- Indexer de reversa 

      intake.setDefaultCommand(
      new IntakeCommand(
          intake,
          () -> controller2.getRawAxis(1),   // joystick izquierdo /eje Y
          () -> controller2.getRawButton(6), // BlqManual /RB 
          () -> controller2.getPOV() == 90,    // Subir Intake /flecha →
          () -> controller2.getPOV() == 270,  // Bajar Intake /flecha ←
          () -> controller2.getPOV() == 180 || pivotearEnabled,  // Subir y bajar Intake /flecha (ARRIBA)
          () -> controller2.getRawButton(8), // Reset Intake // boton en medio (3 lineas)
          () -> controller2.getRawAxis(2))); // activarRodillos /LT  
  }

  // Varibles de los comandos de Autonomos
  public void setAutoShootEnabled(boolean enabled){
    autoShootEnabled = enabled;
  }

  public void setPivotearEnabled(boolean enabled){
    pivotearEnabled = enabled;
  }

  public Command getAutonomousCommand() {
    // return DriveCommands.drivefor(drive, 3);
    return autoChooser.get();
  }

  private int allianceFlip_ejeX() {
    if(!SWERVE_EJE_X_INVERTIR){
      return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -1 : 1;
    }else{
      return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 1 : -1; 
    }
  }

  private int allianceFlip_ejeY() {
    if(!SWERVE_EJE_Y_INVERTIR){
      return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -1 : 1;
    }else{
      return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 1 : -1; 
    }
  }

    private int allianceFlip_rot() {
      if(!SWERVE_ROT_INVERTIR){
        return 1;
      }else{
        return -1;
      }
    }
  private double changeVelocitySwerve() {
    if(controller.getLeftTriggerAxis() < 0.2){
      return 1;
    }else{
      return LowVelocitySwerve; 
    }
  }

  private Rotation2d getAllianceHeading() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
      ? Rotation2d.fromDegrees(180)
      : Rotation2d.fromDegrees(0);
  }

  public void updateJoystickDashboard() {
    SmartDashboard.putNumber("Joystick 1 - Eje X", -controller.getLeftX());
    SmartDashboard.putNumber("Joystick 1 - Eje Y", -controller.getLeftY());
    SmartDashboard.putNumber("Joystick 2 - Eje X", -controller2.getX());
    SmartDashboard.putNumber("Joystick 2 - Eje Y", -controller2.getY());
  }
}
