        package org.firstinspires.ftc.teamcode.robot.systems;

        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
        import org.firstinspires.ftc.teamcode.robot.Controller;
        import org.firstinspires.ftc.teamcode.robot.ControllerManager;
        import org.firstinspires.ftc.teamcode.robot.camera.CameraController;

        import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@Config
public class ShooterController implements Controller {

    public static volatile double MotorRPM = 4800;
    public static volatile double ShootingDelay = 750;
    public static volatile double SpinUpDelay = 750;
    public static volatile double RetractDelay = 750;

    public static double BumpPosition = 0.6;
    public static double RetractPosition = 0.35;

    public static DcMotorSimple.Direction Direction = DcMotorSimple.Direction.REVERSE;

    private volatile int ringCount = 3;

    private final double TicksPerRev = 28; //Do not modify

    public static String ControllerName;
    public boolean shootingState = false;

    public static volatile double TargetTicksPerSecond; //x rev/min * 2pi = x rad/min / 60 = x rad/sec;;
    float wheelRadius = 0.051f; //meters

    private DcMotorEx shooter;
    private Servo bumper;
    private HardwareMap hardwareMap;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private Thread telemetryThread = new Thread(this::telemetry);
    private Thread motorThread = new Thread(this::setVelocity);
    private Thread shootImpl = new Thread(this::shootImpl);

    public ShooterController (HardwareMap hardwareMap, Telemetry telemetry) {
        this.dashboardTelemetry = telemetry;
        this.hardwareMap = hardwareMap;
        ControllerName = getClass().getSimpleName();
    }

    @Override
    public void init() {
        dashboardTelemetry = new MultipleTelemetry(dashboardTelemetry, dashboardTelemetry);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        bumper = hardwareMap.get(Servo.class, "bumper");

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(Direction);

        TargetTicksPerSecond = MotorRPM * TicksPerRev / 60;

        retract();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        shootingState = false;
        shooter.setPower(0);
        bumper.setPosition(RetractPosition);
    }

    public synchronized void shoot(int ringCount){
        this.ringCount = ringCount;
        shootImpl.start();
    }

    public void shootAuto(ControllerManager controllers){
        if (controllers.get(FieldConstants.Camera) == null){
            controllers.add(new CameraController(hardwareMap, dashboardTelemetry), FieldConstants.Camera);
        }

    }

    public void spinUp(double MotorRPM){
        this.MotorRPM = MotorRPM;
        shootingState = true;

        if (!motorThread.isAlive()) motorThread.start();
        else {
            dashboardTelemetry.addLine("Shooter thread already alive");
            dashboardTelemetry.update();
        }

    }

    private synchronized void shootImpl(){
        shootingState = true;
        telemetryThread.start();

        if (!motorThread.isAlive()) {
            motorThread.start();
            sleep(SpinUpDelay);
        }

        for (int i = 0; i < ringCount; i++) {
            bump();
            sleep(RetractDelay);
            retract();
            sleep(ShootingDelay);
        }

        stop();
    }

    public synchronized void telemetry(){
        while (shootingState) {
            double velocity = shooter.getVelocity();
            double RPM = velocity / TicksPerRev * 60;
            double velocityRad = RPM * 2 * Math.PI / 60;
            dashboardTelemetry.addData("target RPM", MotorRPM);
            dashboardTelemetry.addData("current RPM", RPM);
            dashboardTelemetry.addData("tangential velocity (m/s)", velocityRad * wheelRadius);
            dashboardTelemetry.addData("shooter power", shooter.getPower());

            dashboardTelemetry.update();
        }

        dashboardTelemetry.addData("shooter stopped", shooter.getPower());
        dashboardTelemetry.update();
    }

    private synchronized void setVelocity() {
        while (shootingState) {
            TargetTicksPerSecond = MotorRPM * TicksPerRev / 60;
            shooter.setVelocity(TargetTicksPerSecond);
        }
    }

    public double getVelocity(){
        return shooter.getVelocity();
    }

    private void bump() {
        bumper.setPosition(BumpPosition);
    }
    private void retract() {
        bumper.setPosition(RetractPosition);
    }

}