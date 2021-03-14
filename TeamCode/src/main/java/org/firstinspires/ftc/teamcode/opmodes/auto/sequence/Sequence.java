package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;
import org.firstinspires.ftc.teamcode.util.Actions;
import org.firstinspires.ftc.teamcode.util.TrajectoryHelper;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.FrontWobbleXOffset;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.FrontWobbleYOffset;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.Red.PowerShotAbsoluteAngles;
import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;
import static org.firstinspires.ftc.teamcode.util.TrajectoryHelper.buildBackTrajectory;
import static org.firstinspires.ftc.teamcode.util.TrajectoryHelper.buildCustomSpeedLineTrajectory;
import static org.firstinspires.ftc.teamcode.util.TrajectoryHelper.buildCustomSpeedLinearTrajectory;
import static org.firstinspires.ftc.teamcode.util.TrajectoryHelper.buildCustomSpeedSplineTrajectory;
import static org.firstinspires.ftc.teamcode.util.TrajectoryHelper.buildLineTrajectory;
import static org.firstinspires.ftc.teamcode.util.TrajectoryHelper.buildLinearTrajectory;
import static org.firstinspires.ftc.teamcode.util.TrajectoryHelper.buildSplineHeadingTrajectory;
import static org.firstinspires.ftc.teamcode.util.TrajectoryHelper.buildSplineLinearHeadingTrajectory;
import static org.firstinspires.ftc.teamcode.util.TrajectoryHelper.buildStrafeTrajectory;

public abstract class Sequence {

    protected int ringCount;
    protected Telemetry telemetry;
    protected ControllerManager controllers;
    protected final static Object lock = new Object();
    protected Actions actions;
    protected Pose2d startPose;
    protected Vector2d targetZone;
    protected DrivetrainController drive;

    public Sequence(ControllerManager controllers, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.controllers = controllers;
        this.actions = new Actions();
    }

    public void init(int ringCount) {
        synchronized (lock) {
            telemetry.addData("Sequence", "ringCount: " + ringCount + " start pose: " + startPose.toString());
            this.ringCount = ringCount;

            // Reset all actions
            actions = new Actions();
            makeActions();

            // Define our start pose
            drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
            drive.setPoseEstimate(startPose);
        }
    }

    protected abstract void makeActions();

    public void execute() {
        synchronized (lock) {
            telemetry.addData("Sequence", "Executing sequence on thread: " + Thread.currentThread().getId());

            // Do all the work on another thread to avoid blocking the invoking thread
            new Thread(() -> actions.run()).start();
        }
    }

    public void stop() {
        synchronized (lock) {
            telemetry.addData("Sequence", "stop");
            actions.stop();

            //Set starting position for TeleOp
            if (drive != null) MechConstants.TeleStartingPose = drive.getPoseEstimate();
            else telemetry.addLine("Drive is null - stop pressed before start");
        }
    }

    public void moveLinear(double x, double y, double targetHeading) {
        //TODO: EVALUATE LINEAR/LINEAR HEADING and LINEAR/SPLINE HEADING TRAJECTORIES
        drive.followTrajectory(buildLinearTrajectory(drive, x, y, targetHeading));
    }

    public void moveToDropWobble(Vector2d targetZone, double speed) {
        double endTangent = 0;
        double targetHeading = 0;
        double startTangent = 0;
        if (ringCount == 0) {
            targetHeading = 0.1; //rotate other way
            endTangent = -30;
        }
        else if (ringCount == 4) endTangent = -20;

        moveSplineCustomSpeed(targetZone.getX() + FrontWobbleXOffset, targetZone.getY() + FrontWobbleYOffset,
                targetHeading, startTangent, endTangent, speed);
    }

    public void moveLinearTurn(double x, double y, double targetHeading) {
        drive.turnAbsolute(Math.toRadians(targetHeading));
        drive.followTrajectory(TrajectoryHelper.buildLineTrajectory(drive, x, y));
    }

    public void moveLinear(Vector2d pos, double targetHeading) {
        drive.followTrajectory(buildLinearTrajectory(drive, pos.getX(), pos.getY(), targetHeading));
    }

    public void moveSpline(double x, double y, double targetHeading, double startTangent, double endTangent) {
        drive.followTrajectory(buildSplineHeadingTrajectory(drive, startTangent, endTangent, new Pose2d(x,y,targetHeading)));
    }

    public void moveSplineCustomSpeed(double x, double y, double targetHeading, double startTangent, double endTangent, double speed) {
        drive.followTrajectory(buildCustomSpeedSplineTrajectory(drive, x, y, targetHeading, startTangent, endTangent, speed));
    }

    public void moveSplineCustomSpeed(Vector2d pos, double targetHeading, double startTangent, double endTangent, double speed) {
        drive.followTrajectory(buildCustomSpeedSplineTrajectory(drive, pos.getX(), pos.getY(), targetHeading, startTangent, endTangent, speed));
    }

    public void moveSpline(Vector2d vector, double targetHeading, double startTangent, double endTangent) {
        drive.followTrajectory(buildSplineLinearHeadingTrajectory(drive, startTangent, endTangent, new Pose2d(vector,targetHeading)));
    }

    public void dropWobble() {
        telemetry.addData("Sequence","dropWobble");
        WobbleController wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);
        wobble.dropAuto();
        sleep(150);
        wobble.start(); //raise arm
    }

    public void dropWobbleSide() {
        telemetry.addData("Sequence","dropWobbleSide");
        WobbleController wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);
        wobble.sideRelease();
    }

    public void pickupWobble() {
        telemetry.addData("Sequence", "collectWobble");
        WobbleController wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);
        wobble.pickupAuto();
    }

    public void moveToWobble(Vector2d pos) {
        telemetry.addData("Sequence","moveToWobble");
        WobbleController wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);

        drive.followTrajectory(buildBackTrajectory(drive, 18)); //move back to not hit wobble on turn
        //TODO: remove or use less backup
        wobble.readyToPickup();
        drive.followTrajectory(buildCustomSpeedLinearTrajectory(drive, pos.getX(), pos.getY(), 180, 0.85));
    }

    public void approachWobble(Vector2d wobblePos) {
        drive.followTrajectory(buildCustomSpeedLinearTrajectory(drive, wobblePos.getX(), wobblePos.getY(), 180, 0.50));
    }

    public void shootSequence(Vector2d position, double targetHeading, double RPM, int numRings) {
        telemetry.addData("Sequence","moveToShoot" );
        IntakeController intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        ShooterController shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);

        spinUp(RPM);
        drive.followTrajectory(buildLinearTrajectory(drive, position.getX(), position.getY(), targetHeading));

        intake.stopIntake(false);

        shooter.shoot(numRings, RPM);
        IntakeController.stopSweeper();
    }

    public void spinUp(double RPM){
        ShooterController shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        shooter.spinUp(RPM);
    }

    public void stopShooter(){
        ShooterController shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        shooter.stop();
    }

    public void shootGoal(int numRings, double RPM) {
        telemetry.addData("Sequence","shootRings: " + numRings);
        ShooterController shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        drive.turnAbsolute(0);
        shooter.shoot(numRings, RPM);
    }

    public void backOffFromWobbles (double distance) {
        telemetry.addData("Sequence", "back off from wobbles");
        drive.followTrajectory(buildBackTrajectory(drive, distance));
    }

    public void strafe (double x, double y) {
        telemetry.addData("Sequence", "strafe");
        drive.followTrajectory(buildStrafeTrajectory(drive, new Vector2d(x,y)));
    }

    public void powerShot(double RPM) {
        telemetry.addData("Sequence","powerShot");
        ShooterController shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        double sleepDelay = 200;

        sleep(sleepDelay);
        shooter.spinUp(RPM);
        drive.turnAbsolute(Math.toRadians(PowerShotAbsoluteAngles[0]));
        sleep(sleepDelay);

        boolean twoRings = false; //hit three powershots with two rings
        if (twoRings) {
            shooter.powerShot(RPM);
            drive.turnAbsolute(Math.toRadians(PowerShotAbsoluteAngles[1] + PowerShotAbsoluteAngles[2]));
            shooter.powerShot(RPM);
        } else {
            shooter.powerShot(RPM);
            sleep(sleepDelay);
            for (int i = 1; i < 3; i++) {
                drive.turnAbsolute(Math.toRadians(PowerShotAbsoluteAngles[i]));
                sleep(sleepDelay);
                shooter.powerShot(RPM);
                sleep(sleepDelay);
            }
        }

        sleep(50); //buffer
        shooter.stop();
    }

    public void intakeRings(int numRings, Vector2d position, double heading) {
        IntakeController intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        double timeout = 4.50;

        switch (numRings) {
            case (0):
                telemetry.addData("Sequence", "no rings to intake");
                break;

            case (1):
                telemetry.addData("Sequence", "intake 1 ring");
//                drive.turn(Math.toRadians(heading));
                drive.turnToFacePoint(position);
                intake.extend();
                intake.run(FORWARD);

                drive.followTrajectoryAsync(buildCustomSpeedLineTrajectory(drive, position, 0.85));
                while (IntakeController.numRings.get() < 1 && drive.isBusy()) drive.update();
                drive.stop();
                break;

            case (4):
                telemetry.addData("Sequence", "intake 4 rings");
//                drive.turn(Math.toRadians(heading));
                drive.turnToFacePoint(new Vector2d(position.getX() + 3, position.getY()));
                intake.extend();
                intake.run(FORWARD);

                drive.followTrajectoryAsync(buildCustomSpeedLineTrajectory(drive, position, 0.75));
                while (IntakeController.numRings.get() < 3 && drive.isBusy()) drive.update();
                drive.stop();
                break;
            default:
                telemetry.addData("Sequence", "unsupported # of rings to intake");
        }
    }

    public void stopIntake(){
        IntakeController intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        intake.stopIntake();
    }

    public void moveToLaunchLine(Vector2d position) {
        telemetry.addData("Sequence","moveToLaunchLine" );
        drive.followTrajectory(buildLineTrajectory(drive, position.getX(), position.getY()));
    }
}
