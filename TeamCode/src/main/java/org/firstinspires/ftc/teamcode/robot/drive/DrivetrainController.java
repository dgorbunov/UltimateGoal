package org.firstinspires.ftc.teamcode.robot.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.drive.params.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.MockDcMotorEx;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants.kV;


@Config
public class DrivetrainController extends MecanumDrive implements Controller {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(14, 0, 1.5); //10, 0, 0.8
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(10, 0, 1.25); //8, 0, 1=

    public static boolean TESTING = false;

    public static double LATERAL_MULTIPLIER = 1; //Calibrate using StrafeTest

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static int POSE_HISTORY_LIMIT = 100;
    protected Pose2d defaultStartPose = new Pose2d(0, 0, 0);

    public static String ControllerName;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    private TrajectoryFollower follower;

    private LinkedList<Pose2d> poseHistory;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    public static BNO055IMU imu;

    private VoltageSensor batteryVoltageSensor;

    private Pose2d lastPoseOnTurn;
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public DrivetrainController(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public DrivetrainController(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        this.hardwareMap = hardwareMap;

        ControllerName = getClass().getSimpleName();

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(50);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
        accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new LinkedList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);

        if (telemetry != null) {
            telemetry.addData("DrivetrainController", "init");
        }

        if (TESTING) {
            leftFront = new MockDcMotorEx("left_front", telemetry);
            leftRear = new MockDcMotorEx("left_rear", telemetry);
            rightRear = new MockDcMotorEx("right_rear", telemetry);
            rightFront = new MockDcMotorEx("right_front", telemetry);
        }
        else {
            leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
            leftRear = hardwareMap.get(DcMotorEx.class, "left_rear");
            rightRear = hardwareMap.get(DcMotorEx.class, "right_rear");
            rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        }

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        if (!TESTING) {
            for (DcMotorEx motor : motors) {
                MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
                motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
                motor.setMotorType(motorConfigurationType);
            }
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setDirection(DcMotor.Direction.REVERSE);
//        rightRear.setDirection(DcMotor.Direction.REVERSE);

        setLocalizer(new ThreeWheelLocalizer(hardwareMap));
        /** If using two wheel,
         *  See also {@link #getRawExternalHeading()}. */

        setPoseEstimate(defaultStartPose);
    }

    public void turnAsync(double angle) {
        if (telemetry != null) {
            telemetry.addData( "MecanumDrivetrainController", "turnAsync: " + angle);
        }

        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );

        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    @Override
    public void init() {
    }

    @Override
    public void start() {
        if (telemetry != null) {
            telemetry.addData("DrivetrainController", "start");
        }
    }

    public void stop() {
        if (telemetry != null) {
            telemetry.addData("DrivetrainController", "stop");
        }

        mode = Mode.IDLE;
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();

                setDriveSignal(
                        new DriveSignal(
                                new Pose2d(0, 0, targetOmega + correction),
                                new Pose2d(0, 0, targetAlpha)));

                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawRobot(fieldOverlay, newPose);

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void alignWithWobble(double target, double displacement) {
        PIDFController controller = new PIDFController(TRANSLATIONAL_PID);
        controller.setOutputBounds(-1.0,1.0);
        controller.setTargetPosition(target);
        controller.update(displacement);
        while (controller.getLastError() > 75) {
            double correction = controller.update(displacement);
            strafe(correction);
        }
    }
    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    public void strafe(double power) {
        setWeightedDrivePower(new Pose2d(-power, 0, 0));
    }

    public void driveWithGamepad(Gamepad gamepad, double power){
        setWeightedDrivePower(
                new Pose2d(
                        power * -gamepad.left_stick_y,
                        power * -gamepad.left_stick_x,
                        power * -gamepad.right_stick_x
                ));
    }

    public void driveFieldCentric(Gamepad gamepad, double power, FieldConstants.Alliance alliance) {
        // Read pose
        Pose2d poseEstimate = getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input;
        if (alliance != null){
            if (alliance == FieldConstants.Alliance.Red) {
                input = new Vector2d(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x
                ).rotated(-poseEstimate.getHeading() + Math.toRadians(90));
            } else {
                input = new Vector2d(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x
                ).rotated(-poseEstimate.getHeading() + Math.toRadians(-90));
            }
        } else {
            input = new Vector2d(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x
            ).rotated(-poseEstimate.getHeading()); //bottom field perspective
        }

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        setWeightedDrivePower(
                new Pose2d(
                        power * input.getX(),
                        power * input.getY(),
                        power * -gamepad.right_stick_x
                )
        );
    }

    public static AngularVelocityConstraint getMaxAngVelConstraint(){
        return new AngularVelocityConstraint(MAX_ANG_VEL);
    }

    public static MecanumVelocityConstraint getMaxVelConstrain(){
        return new MecanumVelocityConstraint(MAX_VEL, DriveConstants.TRACK_WIDTH);
    }

    public static MecanumVelocityConstraint getCustomVelConstraint(double velocity){
        return new MecanumVelocityConstraint(velocity, DriveConstants.TRACK_WIDTH);
    }

    public static ProfileAccelerationConstraint getMaxAccelConstraint(){
        return new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0;
        /*
        If using two wheel:
        return imu.getAngularOrientation().firstAngle;
         */
    }

}
