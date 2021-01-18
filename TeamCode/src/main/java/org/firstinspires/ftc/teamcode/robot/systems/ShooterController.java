        package org.firstinspires.ftc.teamcode.robot.systems;

        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.dashboard.config.Config;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.teamcode.robot.Controller;

        import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@Config
public class ShooterController implements Controller {

    public static volatile double SpinUpDelay = 1500;
    public static volatile double RetractDelay = 300;
    public static volatile double[] ShootingDelay = {250,250,0};
    public static volatile boolean useDelayArray = false;
    public static volatile double Delay1 = 250;
    public static volatile double Delay2 = 250;


    public static double BumpPosition = 0.6;
    public static double RetractPosition = 0.35;

    public static DcMotorSimple.Direction Direction = DcMotorSimple.Direction.REVERSE;

    //Do not modify
    private final double TicksPerRev = 28;
    double wheelRadius = 0.051; //meters

    public static String ControllerName;

    private volatile int ringCount = 3;
    public static double MotorRPM = 0;

    public static volatile double targetTicksPerSec;


    private DcMotorEx shooter;
    private Servo bumper;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private boolean stopOnFinish = true;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Thread telemetryThread = new Thread(this::telemetry);
    private Thread shootImpl = new Thread(this::shootImpl);

    public ShooterController (HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        ControllerName = getClass().getSimpleName();
    }

    @Override
    public void init() {

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        bumper = hardwareMap.get(Servo.class, "bumper");

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(Direction);

        //this is acting as our state variable
        shooter.setMotorDisable();

        retract();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        //wait until shooting is finished
            shooter.setPower(0);
            shooter.setMotorDisable();
            bumper.setPosition(RetractPosition);
    }



    public synchronized void shoot(int ringCount){
        shoot(ringCount, MotorRPM);
    }

    public synchronized void shoot(int ringCount, double RPM){
        this.ringCount = ringCount;
        checkSpeed(RPM);

        stopOnFinish = true;
        shootImpl.start();
    }

    public synchronized void shootBlocking(int ringCount, double RPM){
        this.ringCount = ringCount;
        checkSpeed(RPM);

        stopOnFinish = true;
        shootImplBlocking();
//        shootImpl.start();
        //TODO :fix
//        try {
//            shootImpl.join();
//        } catch (InterruptedException e) {
//            telemetry.addData(ControllerName, e);
//        }
//        telemetry.addData(ControllerName, shootImpl.isAlive());
//        assert(shootImpl.isAlive());

    }


    public synchronized void powerShot(double RPM){
        ringCount = 1;
        checkSpeed(RPM);

        stopOnFinish = false;
        shootImplBlocking();
    }

    private synchronized void checkSpeed(double RPM) {
        if (RPM != MotorRPM || !shooter.isMotorEnabled()) {
            MotorRPM = RPM;
            setRPM(MotorRPM);
            sleep(SpinUpDelay);
        }
        //if we are spinning up and not at target speed yet, hit this
        else if (shooter.getVelocity() < 0.95 * TicksPerSecond(RPM) || shooter.getVelocity() > 1.05*TicksPerSecond(RPM)){
            sleep(0.5 * SpinUpDelay);
            telemetry.addLine("Spinner was not spinning at correct velocity.");
        }
    }

    /**
     * Spin up flywheel before shooting to save time
     **/
    public void spinUp(double MotorRPM){
        setRPM(MotorRPM);
    }

    private synchronized void shootImplBlocking() {
        //        telemetryThread.start();

        //FTCDash doesn't support array modification
        //So we have to resort to this for quick delay modification
        if (useDelayArray) {
            for (int i = 0; i < ringCount; i++) {
                bump();
                sleep(RetractDelay);
                retract();
                sleep(ShootingDelay[i]);
            }
        } else if (ringCount == 3) {
            bump();
            sleep(RetractDelay);
            retract();
            sleep(Delay1);
            bump();
            sleep(RetractDelay);
            retract();
            sleep(Delay2);
            bump();
            sleep(RetractDelay);
            retract();
        } else if (ringCount == 2){
            bump();
            sleep(RetractDelay);
            retract();
            sleep(Delay1);
            bump();
            sleep(RetractDelay);
            retract();
        } else {
            bump();
            sleep(RetractDelay);
            retract();
        }

        if (stopOnFinish) stop();

    }


    private synchronized void shootImpl(){
//        telemetryThread.start();

        //FTCDash doesn't support array modification
        //So we have to resort to this for quick delay modification
        if (useDelayArray) {
            for (int i = 0; i < ringCount; i++) {
                bump();
                sleep(RetractDelay);
                retract();
                sleep(ShootingDelay[i]);
            }
        } else if (ringCount == 3) {
            bump();
            sleep(RetractDelay);
            retract();
            sleep(Delay1);
            bump();
            sleep(RetractDelay);
            retract();
            sleep(Delay2);
            bump();
            sleep(RetractDelay);
            retract();
        } else if (ringCount == 2){
            bump();
            sleep(RetractDelay);
            retract();
            sleep(Delay1);
            bump();
            sleep(RetractDelay);
            retract();
        } else {
            bump();
            sleep(RetractDelay);
            retract();
        }

        if (stopOnFinish) stop();

    }

    public synchronized void telemetry(){
//        while (shootingState) {
//            double velocity = shooter.getVelocity();
//            double RPM = velocity / TicksPerRev * 60;
//            double velocityRad = RPM * 2 * Math.PI / 60;
//            dashboardTelemetry.addData("target RPM", MotorRPM);
//            dashboardTelemetry.addData("current RPM", RPM);
//            dashboardTelemetry.addData("tangential velocity (m/s)", velocityRad * wheelRadius);
//            dashboardTelemetry.addData("shooter power", shooter.getPower());
//
//            dashboardTelemetry.update();
//        }
//
//        dashboardTelemetry.addData("shooter stopped", shooter.getPower());
//        dashboardTelemetry.update();
    }

    private void setRPM(double RPM) {
        MotorRPM = RPM;
        shooter.setMotorEnable();
        shooter.setVelocity(TicksPerSecond(MotorRPM));
    }

    public double getVelocity(){
        return shooter.getVelocity();
    }

    private double TicksPerSecond(double RPM){
        return RPM * TicksPerRev / 60;
    }

    private void bump() {
        bumper.setPosition(BumpPosition);
    }
    private void retract() {
        bumper.setPosition(RetractPosition);
    }

}