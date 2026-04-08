package frc.robot.subsystems.autoaim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;

public class AutoAim extends SubsystemBase {
  private final Drive drive;
  private final StateSystem state = new StateSystem();

  private final List<Zone> zones = new ArrayList<>();
  private final List<Ruta> routes = new ArrayList<>();

  private Pose2d targetPose;
  private Ruta currentRoute;
  private List<Integer> previousActiveZones = new ArrayList<>();
  private List<Integer> ids = null;

  private int routeIndex = 0;
  private int direction_old = -1;

  private double gradosToHub = 0;
  private double distanceToHub = 0;
  private boolean isTargetHUB;
  private boolean isOnlyRotate;

  private double joyX = 0;
  private double joyY = 0;
  private double joyRot = 0;

  private PIDController xController, yController, rotController, rotController_HUB;

  private static final double KP = 3.2, KI = 0.0, KD = 0.0; // KP 3.4, 2.6
  private static final double KP_ROT = 3.0, KI_ROT = 0.0, KD_ROT = 0.0; // KP 3.0
  private static final double KP_ROT_HUB = 2.8, KI_ROT_HUB = 0.0, KD_ROT_HUB = 0.0; // KP 2.8

  private static double MAX_OUTPUT = 3.4; // 0.7 - 3.4
  private static double MIN_OUTPUT = 0.4; // 0.4
  private static double MAX_OUTPUT_ROT = Math.toRadians(300); // 0.55 - 360 grad
  private static double MIN_OUTPUT_ROT = Math.toRadians(45);; // 0.10 - 40grad
  private static final double MAX_OUTPUT_ROT_HUB = 0.60; // 0.60
  private static final double MIN_OUTPUT_ROT_HUB = 0.12; // 0.10 - 0.15

  private static final double ERROR_PID = 0.05; // en metros (x,y) --- 0.07
  private static final double ERROR_DEGREES_PID = 2; // en grados (rotacion) --- 3
  private static final double ERROR_DEGREES_HUB_PID = 0.6; // en grados (rotacion) --- 1 - 1.5

  private static double ERROR_X_ADVANCE = 0.07; // en metros (x,y) --- 0.1
  private static double ERROR_Y_ADVANCE = 0.07; // en metros (x,y) --- 0.1
  private static double ERROR_DEGREES_ADVANCE = 3.0; // en grados (rotacion) --- 3, 3.5

  private SlewRateLimiter xAccelLimit = new SlewRateLimiter(3.0); // 3
  private SlewRateLimiter yAccelLimit = new SlewRateLimiter(3.0); // 3
  private SlewRateLimiter rotAccelLimit = new SlewRateLimiter(Math.toRadians(400)); //360

  private SlewRateLimiter xDecelLimit = new SlewRateLimiter(6);
  private SlewRateLimiter yDecelLimit = new SlewRateLimiter(6);
  private SlewRateLimiter rotDecelLimit = new SlewRateLimiter(Math.toRadians(400)); //400

  private double AutoAim_Vel_X = 0;
  private double AutoAim_Vel_Y = 0;
  private double AutoAim_Vel_ROT = 0;


  public AutoAim(Drive drive) {
    this.drive = drive;
    configurePID();
    resetPIDControllers();
    createZones();
    createRoutes();
  }

  private void createZones() {
    zones.add(
        new Zone( // Zona completa AZUL
            1, 0, 8.25, 0, 8)); // límites xMin, xMax, yMin, yMax
    zones.add(
        new Zone( // Zona Derecha AZUL
            2, 0, 8.25, 0, 4)); // límites xMin, xMax, yMin, yMax
    zones.add(
        new Zone( // Zona Izquierda AZUL
            3, 0, 8.25, 4, 8)); // límites xMin, xMax, yMin, yMax

    zones.add(
        new Zone( // Zona completa ROJA
            4, 8.26, 16.6, 0, 8)); // límites xMin, xMax, yMin, yMax
    zones.add(
        new Zone( // Zona Derecha ROJA
            5, 8.26, 16.6, 4, 8)); // límites xMin, xMax, yMin, yMax
    zones.add(
        new Zone( // Zona Izquierda ROJA
            6, 8.26, 16.6, 0, 4)); // límites xMin, xMax, yMin, yMax
  }

    private void createRoutes() {
      /*Direccion 0 = Apuntar al HUB 
      Direccion 1 = Ir de zona alianza a la zona neutral
      Direccion 2 = Ir de zona neutral a la zona alianza (AGREGAR IR CON EL INTAKE HACIE NUESTRA ALIANZA)
      Direccion 3 = Ir a colgarnos a la IZQUIERDA
      Direccion 4 = Ir a colgarnos a la DERECHA
      Direccion 5 = Apuntar hacia nuestra Alianza
      Direccion 6 = Ir al Depot                          (AGREGAR)
      Direccion 7 = Ir al Opost                          (AGREGAR)

      Direccion 8 = Ir de zona alianza a la zona neutral/Autonomo (Desde la linea)
      Direccion 9 = Ir de zona neutral a la zona alianza/Autonomo (Llegar a la linea, no pasarte tanto)
      Direccion 10 = Ir de zona alianza a la zona neutral/INCLINADO o intake hacia el trench(autonomo iniciar asi)
      Direccion 11 = Ir de zona neutral a la zona alianza/INCLINADO*/

    //////////////////////////////////// ALIANZA AZUL ////////////////////////////////////////////
    Ruta r1 = new Ruta(1, 0);    // cordenadas para el HUB campo Azul
    r1.addPoint(new Pose2d(4.6, 4, new Rotation2d()), 
                new EjesMov(false,false,true));
    routes.add(r1);

    Ruta r2 = new Ruta(1, 5);    // Para apuntar hacia la alianza Azul
    r2.addPoint(new Pose2d(0, 0, Rotation2d.fromDegrees(180)), 
                new EjesMov(false,false,true));
    routes.add(r2);

    Ruta r3 = new Ruta(1, 3);  // Ir hacia el lugar para colgarnos a la IZQUIERDA Alianza AZUL
    r3.addPoint(new Pose2d(1.8, 4.12, Rotation2d.fromDegrees(90)), 
                new EjesMov(true,true,true));
    r3.addPoint(new Pose2d(1.5, 4.12, Rotation2d.fromDegrees(90)), 
                new EjesMov(true,true,true));
    routes.add(r3);

    Ruta r4 = new Ruta(1, 4);  // // Ir hacia el lugar para colgarnos a la DERECHA Alianza AZUL
    r4.addPoint(new Pose2d(1.8, 3.3, Rotation2d.fromDegrees(90)), 
                new EjesMov(true,true,true));
    r4.addPoint(new Pose2d(1.5, 3.3, Rotation2d.fromDegrees(90)), 
                new EjesMov(true,true,true));
    routes.add(r4);

    Ruta r5 = new Ruta(2, 1);   // Zona Derecha AZUL -- Ir a zona neutra
    r5.addPoint(new Pose2d(3, 0.6, Rotation2d.fromDegrees(0)), 
                new EjesMov(true,true,true));
    r5.addPoint(new Pose2d(6, 0.6, Rotation2d.fromDegrees(0)), 
                new EjesMov(true,true,true));
    r5.addPoint(new Pose2d(7.5, 2.3, Rotation2d.fromDegrees(-135)), 
                new EjesMov(true,true,true));
    routes.add(r5);

    Ruta r6 = new Ruta(2, 2);   // Zona Derecha AZUL -- Ir a zona de alianza
    r6.addPoint(new Pose2d(6, 0.6, Rotation2d.fromDegrees(180)), 
                new EjesMov(true,true,true));
    r6.addPoint(new Pose2d(3, 0.6, Rotation2d.fromDegrees(180)), 
                new EjesMov(true,true,true));
    /*r6.addPoint(new Pose2d(2.3, 2.3, Rotation2d.fromDegrees(45)), 
                new EjesMov(true,true,true));*/
    routes.add(r6);

    Ruta r7 = new Ruta(3, 1);      // Zona Izquierda AZUL -- Ir a zona neutra
    r7.addPoint(new Pose2d(3, 7.4, Rotation2d.fromDegrees(0)), 
                new EjesMov(true,true,true));
    r7.addPoint(new Pose2d(6, 7.4, Rotation2d.fromDegrees(0)), 
                new EjesMov(true,true,true));
    r7.addPoint(new Pose2d(7.5, 5.7, Rotation2d.fromDegrees(135)), 
                new EjesMov(true,true,true));
    routes.add(r7);

    Ruta r8 = new Ruta(3, 2);     // Zona Izquierda AZUL -- Ir a zona de alianza
    r8.addPoint(new Pose2d(6, 7.4, Rotation2d.fromDegrees(180)), 
                new EjesMov(true,true,true));
    r8.addPoint(new Pose2d(3, 7.4, Rotation2d.fromDegrees(180)), 
                new EjesMov(true,true,true));
    /*r8.addPoint(new Pose2d(2.3, 5.7, Rotation2d.fromDegrees(-45)), 
                new EjesMov(true,true,true));*/
    routes.add(r8);

    //////////////////////////////////// ALIANZA ROJA ////////////////////////////////////////////
    Ruta r9 = new Ruta(4, 0);    // cordenadas para el HUB campo ROJA
    r9.addPoint(new Pose2d(12.0, 4, new Rotation2d()), 
                new EjesMov(false,false,true));
    routes.add(r9);

    Ruta r10 = new Ruta(4, 5);    // Para apuntar hacia la alianza ROJA
    r10.addPoint(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                new EjesMov(false,false,true));
    routes.add(r10);

    Ruta r11 = new Ruta(4, 3);  // Ir hacia el lugar para colgarnos a la IZQUIERDA Alianza ROJA
    r11.addPoint(new Pose2d(14.5, 3.94, Rotation2d.fromDegrees(-90)), 
                new EjesMov(true,true,true));
    r11.addPoint(new Pose2d(15.0, 3.94, Rotation2d.fromDegrees(-90)), 
                new EjesMov(true,true,true));
    routes.add(r11);

    Ruta r12 = new Ruta(4, 4);  // // Ir hacia el lugar para colgarnos a la DERECHA Alianza ROJA
    r12.addPoint(new Pose2d(14.5, 4.78, Rotation2d.fromDegrees(-90)), 
                new EjesMov(true,true,true));
    r12.addPoint(new Pose2d(15.0, 4.78, Rotation2d.fromDegrees(-90)), 
                new EjesMov(true,true,true));
    routes.add(r12);

    Ruta r13 = new Ruta(5, 1);      // Zona Derecha ROJA -- Ir a zona neutra
    r13.addPoint(new Pose2d(13.4, 7.4, Rotation2d.fromDegrees(180)), 
                new EjesMov(true,true,true));
    r13.addPoint(new Pose2d(10.4, 7.4, Rotation2d.fromDegrees(180)), 
                new EjesMov(true,true,true));
    r13.addPoint(new Pose2d(8.9, 5.7, Rotation2d.fromDegrees(45)), 
                new EjesMov(true,true,true));
    routes.add(r13);

    Ruta r14 = new Ruta(5, 2);     // Zona Derecha ROJA -- Ir a zona de alianza
    r14.addPoint(new Pose2d(10.4, 7.4, Rotation2d.fromDegrees(0)), 
                new EjesMov(true,true,true));
    r14.addPoint(new Pose2d(13.4, 7.4, Rotation2d.fromDegrees(0)), 
                new EjesMov(true,true,true));
    /*r16.addPoint(new Pose2d(14.3, 5.7, Rotation2d.fromDegrees(-135)), 
                new EjesMov(true,true,true));*/
    routes.add(r14);

    Ruta r15 = new Ruta(6, 1);   // Zona Izquierda ROJA -- Ir a zona neutra
    r15.addPoint(new Pose2d(13.4, 0.6, Rotation2d.fromDegrees(180)), 
                new EjesMov(true,true,true));
    r15.addPoint(new Pose2d(10.4, 0.6, Rotation2d.fromDegrees(180)), 
                new EjesMov(true,true,true));
    r15.addPoint(new Pose2d(8.9, 2.3, Rotation2d.fromDegrees(-45)), 
                new EjesMov(true,true,true));
    routes.add(r15);

    Ruta r16 = new Ruta(6, 2);   // Zona Izquierda ROJA -- Ir a zona de alianza
    r16.addPoint(new Pose2d(10.4, 0.6, Rotation2d.fromDegrees(0)), 
                new EjesMov(true,true,true));
    r16.addPoint(new Pose2d(13.4, 0.6, Rotation2d.fromDegrees(0)), 
                new EjesMov(true,true,true));
    /*r14.addPoint(new Pose2d(14.3, 2.3, Rotation2d.fromDegrees(135)), 
                new EjesMov(true,true,true));*/
    routes.add(r16);
  }

  public void onlyRotateToTarget(int direction) {
    if (!verificacion(direction)) return;
    Cambiar_Max_Output(direction);
    setGradosToHub();

    OutputDrive output = calculateDriveOutput();
    DriveCommands.joystickDrive(drive, () -> output.dx, () -> output.dy, () -> output.drot).execute();
  }

  public void moveToTarget(int direction) {
    if (!verificacion(direction)) return;

    Cambiar_Error_Advance(direction);
    boolean isLastPoint = routeIndex >= currentRoute.points.size() - 1;
    if (state.isAutoDriving && targetPose != null && (isLastPoint ? atSetPointFinal() : atSetPointFase()) && !state.hasAdvanced) {
      routeIndex++;
      state.hasAdvanced = true;
      state.atFinalSetpoint = (routeIndex >= currentRoute.points.size()) ? true : false;
    }

    if (!(isLastPoint ? atSetPointFinal() : atSetPointFase())) {
      state.hasAdvanced = false;
    }

    /*if (!atSetPointFinal()) {
      state.hasAdvanced = false;
    }*/

    if (state.isRouteFinished) return;

    if (state.atFinalSetpoint) {
      state.isRouteFinished = true;
      stop();
      return;
    }

    Cambiar_Max_Output(direction);
    moveToTargetPose(direction);
  }

    public boolean verificacion(int direction) {
    if (ids == null || ids.isEmpty()) return false;
    
    state.hasChangedDirection = (direction != direction_old);
    direction_old = direction;

    if (state.hasChangedZone || state.hasChangedDirection) {
      resetFullRoute();
    }

    int selectedId = -1;
    for (int id : ids) {
      if (getRoute(id, direction) != null) {
        selectedId = id;
        break; 
      }
    }

    if (selectedId == -1) {
      stop();
      return false;
    }

    targetPose = updateTargetPose(selectedId, direction);
    if (targetPose == null) {
      stop();
      return false;
    }
    return true;
  }

  private void moveToTargetPose(int direction) {
    if (!state.isAutoDriving) return;
    if (targetPose == null) return;

    OutputDrive output = calculateDriveOutput();

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      output.dx,
      output.dy,
      output.drot,
      drive.getPose().getRotation());
    drive.runVelocity(speeds);
  }

  private OutputDrive calculateDriveOutput() {
    Pose2d currentPose = drive.getPose();
    double outputRot = 0;

    double outputX = xController.calculate(currentPose.getX(), targetPose.getX());
    double outputY = yController.calculate(currentPose.getY(), targetPose.getY());
    if(isTargetHUB){
      outputRot = rotController_HUB.calculate(currentPose.getRotation().getRadians(), gradosToHub);
    }else{
      outputRot = rotController.calculate(currentPose.getRotation().getRadians(),
                                          targetPose.getRotation().getRadians());
    }

    if (xController.atSetpoint()){
      outputX = 0;
      xAccelLimit.reset(0);
      xDecelLimit.reset(0);
    }else {
      outputX = limitOutput(outputX, MIN_OUTPUT, MAX_OUTPUT);
      double accelX = xAccelLimit.calculate(outputX);
      double decelX = xDecelLimit.calculate(outputX);
      outputX = limitAccel(accelX, decelX);
    }

    if (yController.atSetpoint()){
      outputY = 0;
      yAccelLimit.reset(0);
      yDecelLimit.reset(0);
    }else {
      outputY = limitOutput(outputY, MIN_OUTPUT, MAX_OUTPUT);
      double accelY = yAccelLimit.calculate(outputY);
      double decelY = yDecelLimit.calculate(outputY);
      outputY = limitAccel(accelY, decelY);
    }
    
    if(isTargetHUB || isOnlyRotate){
      outputRot = rotController_HUB.atSetpoint() ? 0 : limitOutput(outputRot, MIN_OUTPUT_ROT_HUB, MAX_OUTPUT_ROT_HUB);
    }else{
      if(rotController.atSetpoint()){
        outputRot = 0;
        rotAccelLimit.reset(0);
        rotDecelLimit.reset(0);
      }else {
        outputRot = limitOutput(outputRot, MIN_OUTPUT_ROT, MAX_OUTPUT_ROT);
        double accelRot = rotAccelLimit.calculate(outputRot);
        double decelRot = rotDecelLimit.calculate(outputRot);
        outputRot = limitAccel(accelRot, decelRot);
      }
    }

    if(currentRoute!=null && routeIndex < currentRoute.enablesMov.size()){
    EjesMov enableMov = currentRoute.enablesMov.get(routeIndex);
    if (!enableMov.x) outputX = joyX;
    if (!enableMov.y) outputY = joyY;
    if (!enableMov.rot) outputRot = joyRot;
    } 

    AutoAim_Vel_X = outputX;
    AutoAim_Vel_Y = outputY;
    AutoAim_Vel_ROT = outputRot;
    return new OutputDrive(outputX, outputY, outputRot);
  }

  private void Cambiar_Max_Output(double direccion){
    if(direccion == 1){
      if(routeIndex == 1){
        MAX_OUTPUT = 3.4; // 0.7 Velocidad que pasa por el trench
      }else if(routeIndex == 2){
        MAX_OUTPUT = 3.4; //3.4 Velocidad para recojer Fuels        
      }
      else{
        MAX_OUTPUT = 3.4; // 0.7
      }
    }else if(direccion == 2){
      if(routeIndex == 1){
        MAX_OUTPUT = 3.4; // 0.5 Velocidad que pasa por el trench
      }else{
        MAX_OUTPUT = 3.4;  // 0.7
      }
    }else{
      MAX_OUTPUT = 3.4; // 0.7
    }
  }

  private void Cambiar_Error_Advance(double direccion){
    if(direccion == 1){
      if(routeIndex == 0){ // indice para colocarte antes de Pasar por el trench
        ERROR_X_ADVANCE = 0.5;
        ERROR_Y_ADVANCE = 0.07;
        ERROR_DEGREES_ADVANCE = 3.0;
      }else if(routeIndex == 1){
        ERROR_X_ADVANCE = 0.5; //0.5
        ERROR_Y_ADVANCE = 0.07;
        ERROR_DEGREES_ADVANCE = 3.0;
      }else{
        ERROR_X_ADVANCE = 0.07;
        ERROR_Y_ADVANCE = 0.07;
        ERROR_DEGREES_ADVANCE = 3.5;
      }
    }else if(direccion == 2){
      if(routeIndex == 0){ // indice para colocarte antes de Regresar por el trench
        ERROR_X_ADVANCE = 0.6; // 0.25
        ERROR_Y_ADVANCE = 0.07;
        ERROR_DEGREES_ADVANCE = 3.0;
      }else{
        ERROR_X_ADVANCE = 0.07;
        ERROR_Y_ADVANCE = 0.07;
        ERROR_DEGREES_ADVANCE = 3.5;
      }
    }else{
        ERROR_X_ADVANCE = 0.07;
        ERROR_Y_ADVANCE = 0.07;
        ERROR_DEGREES_ADVANCE = 3.5;
    }
  }

  @Override
  public void periodic() {
    ids = getCurrentZoneIds();
    if (state.hasChangedZone || state.hasChangedDirection) {
      resetFullRoute();
    }
    setDistanceToHub();

    SmartDashboard.putNumber("PRUEBA_VEL_X", AutoAim_Vel_X);
    SmartDashboard.putNumber("PRUEBA_VEL_Y", AutoAim_Vel_Y);
    SmartDashboard.putNumber("PRUEBA_VEL_ROT", Math.toDegrees(AutoAim_Vel_ROT));

    if (targetPose != null) {
      SmartDashboard.putNumber("Target X", targetPose.getX());
      SmartDashboard.putNumber("Target Y", targetPose.getY());
      SmartDashboard.putNumber("Target Rotation", targetPose.getRotation().getDegrees());
    } else {
      SmartDashboard.putNumber("Target X", 0);
      SmartDashboard.putNumber("Target Y", 0);
      SmartDashboard.putNumber("Target Rotation", 0);
    }
    
    SmartDashboard.putBoolean("AutoAim Setpoint", atSetPointFase());
    SmartDashboard.putBoolean("RotHUB Setpoint", autoAimHubFinished());
    SmartDashboard.putBoolean("isTargetHUB", isTargetHUB);
    SmartDashboard.putNumber("GradosToHub", Math.toDegrees(gradosToHub));
    SmartDashboard.putNumber("DistanceToHub", distanceToHub);
    SmartDashboard.putNumber("Max-Vel X-Y AutoAim", MAX_OUTPUT);
    SmartDashboard.putNumber("Max-Vel Rot AutoAim", MAX_OUTPUT_ROT);

  }

  private List<Integer> getCurrentZoneIds() {
      Pose2d currentPose = drive.getPose();
      List<Integer> activeZoneIds = new ArrayList<>();

      for (Zone z : zones) {
          if (z.contains(currentPose.getX(), currentPose.getY())) {
              activeZoneIds.add(z.zoneId);
          }
      }

      if (!activeZoneIds.equals(previousActiveZones)) {
          state.hasChangedZone = true;
          previousActiveZones = new ArrayList<>(activeZoneIds); 
      } else {
          state.hasChangedZone = false;
      }

      return activeZoneIds;
  }

  private Pose2d updateTargetPose(int id, int direccion) {
    currentRoute = getRoute(id, direccion);
    if (currentRoute == null || currentRoute.points.isEmpty()) return null;
      //funcion elegir punto mas cercano, decir vas a inciar en A , vas a punto B, C y asi , no esrtarico
    state.isAutoDriving = true;

    if (routeIndex >= currentRoute.points.size()) {
      return null;
    }
    return currentRoute.points.get(routeIndex);
  }

  private void setGradosToHub() {
    if (targetPose == null) return;

    Pose2d currentPose = drive.getPose();
    double dx = targetPose.getX() - currentPose.getX();
    double dy = targetPose.getY() - currentPose.getY();

    gradosToHub = Math.atan2(dy, dx);
  }

  private void setDistanceToHub() {
    if(ids == null || ids.isEmpty()) return;
    int zoneId = ids.get(0);

    Pose2d robotPose = drive.getPose();
    Pose2d hub = null;

    if(zoneId == 1 || zoneId == 2 || zoneId == 3){      // CAMPO AZUL
      hub = new Pose2d(4.6, 4, new Rotation2d());
    }else if(zoneId == 4 || zoneId == 5 || zoneId == 6){   // CAMPO ROJO
      hub = new Pose2d(12.0, 4, new Rotation2d());
    }

    if(hub == null) return;
    double dx = hub.getX() - robotPose.getX();
    double dy = hub.getY() - robotPose.getY();

    distanceToHub = Math.hypot(dx, dy);
  }

  public double getGradosToHub() {
    if(!isTargetHUB) return -1;
    return gradosToHub;
  }

  public double getDistanceToHub() {
    return distanceToHub; 
  }

  public void setIsTargetHUB(boolean activar) {
    isTargetHUB = activar;
  }

  public void setIsOnlyRotate(boolean activar) {
    isOnlyRotate = activar;
  }
  public void getControllerInputs(double joyX, double joyY, double joyRot) {
    this.joyX = joyX;
    this.joyY = joyY;
    this.joyRot = joyRot;
  }

    private Ruta getRoute(int zoneId, int direction) {
    for (Ruta r : routes) {
      if (r.zoneId == zoneId && r.direction == direction) {
        return r;
      }
    }
    return null;
  }

  private void configurePID() {
    xController = new PIDController(KP, KI, KD);
    yController = new PIDController(KP, KI, KD);
    rotController = new PIDController(KP_ROT, KI_ROT, KD_ROT);
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    rotController_HUB = new PIDController(KP_ROT_HUB, KI_ROT_HUB, KD_ROT_HUB);
    rotController_HUB.enableContinuousInput(-Math.PI, Math.PI);


    xController.setTolerance(ERROR_PID);
    yController.setTolerance(ERROR_PID);
    rotController.setTolerance(Math.toRadians(ERROR_DEGREES_PID));
    rotController_HUB.setTolerance(Math.toRadians(ERROR_DEGREES_HUB_PID));
  }

    private void resetPIDControllers() {
    xController.reset();
    yController.reset();
    rotController.reset();
    rotController_HUB.reset();
  }

  private void resetLimitersAcc(){
    ChassisSpeeds current = drive.getCurrentChassisSpeeds();
    xAccelLimit.reset(current.vxMetersPerSecond);
    yAccelLimit.reset(current.vyMetersPerSecond);
    rotAccelLimit.reset(current.omegaRadiansPerSecond);

    xDecelLimit.reset(current.vxMetersPerSecond);
    yDecelLimit.reset(current.vyMetersPerSecond);
    rotDecelLimit.reset(current.omegaRadiansPerSecond);
  }

    private void resetFullRoute() {
      routeIndex = 0;
      state.reset();
      resetPIDControllers();
      resetLimitersAcc();
    }

  private double limitOutput(double value, double minLimit, double maxLimit) {
    if (value == 0.0) return 0.0;
    double abs = Math.abs(value);

    if (abs <= minLimit) {
      return Math.copySign(minLimit, value);
    }

    return Math.copySign(Math.min(abs, maxLimit),value);
  }

  private double limitAccel(double accelValue, double decelValue){
    return Math.abs(accelValue) < Math.abs(decelValue) ? accelValue : decelValue;
  }

    public void stop() {
      state.isAutoDriving = false;
      drive.runVelocity(new ChassisSpeeds(0, 0, 0));
    }

    public boolean atSetPointFinal(){
      if (currentRoute == null || routeIndex >= currentRoute.points.size()) {
        return false;
      }
      boolean setpointX, setpointY, setpointRot;
      EjesMov enableMov = currentRoute.enablesMov.get(routeIndex);

      setpointX = !enableMov.x || xController.atSetpoint();
      setpointY = !enableMov.y || yController.atSetpoint();
      setpointRot = !enableMov.rot || rotController.atSetpoint();
      
      return setpointX && setpointY && setpointRot;
    }

    public boolean atSetPointFase(){
      if (currentRoute == null || routeIndex >= currentRoute.points.size()) {
        return false;
      }

      Pose2d currentPose = drive.getPose();
      Pose2d target = currentRoute.points.get(routeIndex);
      EjesMov enableMov = currentRoute.enablesMov.get(routeIndex);

      boolean setpointX = !enableMov.x || 
        Math.abs(target.getX() - currentPose.getX()) <= ERROR_X_ADVANCE;
      boolean setpointY = !enableMov.y ||
          Math.abs(target.getY() - currentPose.getY()) <= ERROR_Y_ADVANCE;
      boolean setpointRot = !enableMov.rot ||
        Math.abs(target.getRotation().minus(currentPose.getRotation()).getDegrees()) <= ERROR_DEGREES_ADVANCE;

      return setpointX && setpointY && setpointRot;
    }

    public boolean isRouteFinished() {
      return state.isRouteFinished;
    }

    public boolean autoAimHubFinished() {
      return rotController_HUB.atSetpoint();
    }

/////////////////////////////////////////////////////////////////// clases ///////////////////////////////////////////////////////////////////
    public static class Zone {
      public int zoneId;
      public double xMin, xMax, yMin, yMax;

      public Zone(int zoneId, double xMin, double xMax, double yMin, double yMax) {
        this.zoneId = zoneId;
        this.xMin = xMin;
        this.xMax = xMax;
        this.yMin = yMin;
        this.yMax = yMax;
      }

      public boolean contains(double x, double y) {
        return x >= xMin && x <= xMax && y >= yMin && y <= yMax;
      }
    }

    public class Ruta {
      public int zoneId, direction;
      public List<Pose2d> points;
      public List<EjesMov> enablesMov;


      // variables del de moviemnto de abc que este mas cerno no aferza icniar en A ,, que se pueda
      // elegir, y acticar o no la funcion
      public Ruta(int zoneId, int direction) {
        this.zoneId = zoneId;
        this.direction = direction;
        this.points = new ArrayList<>();
        this.enablesMov = new ArrayList<>();
      }
      public void addPoint(Pose2d point, EjesMov enableMov) {
        points.add(point);
        enablesMov.add(enableMov);
      }
    }

    public class StateSystem {
      public boolean isAutoDriving = false;
      public boolean hasChangedZone = false;
      public boolean atFinalSetpoint = false;
      public boolean hasChangedDirection = false;
      public boolean isRouteFinished = false;
      public boolean hasAdvanced = false;

      public void reset() {
          isAutoDriving = false;
          hasChangedZone = false;
          atFinalSetpoint = false;
          hasChangedDirection = false;
          isRouteFinished = false;
          hasAdvanced = false;
      }
  }

  private static class OutputDrive {
    public final double dx, dy, drot;

    public OutputDrive(double dx, double dy, double drot) {
      this.dx = dx;
      this.dy = dy;
      this.drot = drot;
    }
  }

    private static class EjesMov{
      public Boolean x, y, rot;

      public EjesMov(Boolean x, Boolean y, Boolean rot) {
        this.x = x;
        this.y = y;
        this.rot = rot;
      }
    }
}
