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

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.opmodes.auto.sequence.TrajectoryHelper.buildBackTrajectory;
import static org.firstinspires.ftc.teamcode.opmodes.auto.sequence.TrajectoryHelper.buildCustomSpeedLinearTrajectory;
import static org.firstinspires.ftc.teamcode.opmodes.auto.sequence.TrajectoryHelper.buildIntakeTrajectory;
import static org.firstinspires.ftc.teamcode.opmodes.auto.sequence.TrajectoryHelper.buildLineTrajectory;
import static org.firstinspires.ftc.teamcode.opmodes.auto.sequence.TrajectoryHelper.buildLinearTrajectory;
import static org.firstinspires.ftc.teamcode.opmodes.auto.sequence.TrajectoryHelper.buildSplineTrajectoryConstantHeading;
import static org.firstinspires.ftc.teamcode.opmodes.auto.sequence.TrajectoryHelper.buildStrafeTrajectory;
import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

public abstract class Sequence {

    protected int ringCount;
    protected Telemetry telemetry;
    protected ControllerManager controllers;
    protected final static Object lock = new Object();
    protected Actions actions;
    protected Pose2d startPose;
    protected Vector2d targetZone;
    protected DrivetrainController drive;

    //TODO: Build all trajectories before running?

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
            if (drive != null) MechConstants.StartingPose = drive.getPoseEstimate();
            else telemetry.addLine("Drive is null - stop pressed before start");
        }
    }

    public void moveLinear(double x, double y, double targetHeading) {
        drive.followTrajectory(buildLinearTrajectory(drive, x, y, targetHeading));
    }

    public void moveLinear(Vector2d pos, double targetHeading) {
        drive.followTrajectory(buildLinearTrajectory(drive, pos.getX(), pos.getY(), targetHeading));
    }

    public void dropWobble() {
        telemetry.addData("Sequence","dropWobble");
        WobbleController wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);
        wobble.dropAuto();
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
//        drive.followTrajectory(buildSplineTrajectory(drive, 180, new Pose2d(intermediate, endTangent)));
        wobble.readyToPickup();
        drive.followTrajectory(buildBackTrajectory(drive, 18)); //move back to not hit wobble on turn
        drive.followTrajectory(buildLinearTrajectory(drive, pos.getX(), pos.getY(), 180));
    }

    public void approachWobble(Vector2d wobblePos) {
        drive.followTrajectory(buildCustomSpeedLinearTrajectory(drive, wobblePos.getX(), wobblePos.getY(), 180, 8));
    }


    public void moveToShoot(Vector2d intermediate, Vector2d position, double targetHeading) {
        telemetry.addData("Sequence","moveToShoot" );

        Vector2d positions[] = new Vector2d[] {
                intermediate,
                position
        };

//        drive.followTrajectory(buildLineTrajectory(positions));
        drive.followTrajectory(buildSplineTrajectoryConstantHeading(drive, positions, targetHeading));
        //TODO: fix, path continuity exception
    }

    public void moveToShoot(Pose2d position, double heading) {
        telemetry.addData("Sequence", "moveToShoot");
        drive.followTrajectory(TrajectoryHelper.buildLineTrajectory(drive, position));
    }

    public void startShooter(double RPM){
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
        boolean twoRings = false; //hit three powershots with two rings
        shooter.spinUp(RPM);

        if (twoRings) {
            shooter.powerShot(RPM);
            drive.turn(Math.toRadians(MechConstants.Red.PowerShotAngleIncrement));
            shooter.powerShot(RPM);
        } else {
            drive.turn(Math.toRadians(-MechConstants.Red.PowerShotAngleIncrement));
            shooter.powerShot(RPM);
            for (int i = 0; i < 2; i++) {
                drive.turn(Math.toRadians(MechConstants.Red.PowerShotAngleIncrement));
                shooter.powerShot(RPM);
            }
        }
        sleep(50); //buffer
        shooter.stop();
    }

    public void moveToPowerShot(Vector2d intermediate, Vector2d shotPos) {
//        drive.followTrajectory(buildLinearChainTrajectory(drive,
//                new Pose2d(intermediate, 0),
//                new Pose2d(shotPos, 0)));
        moveLinear(intermediate, 0);
        moveLinear(shotPos, 0);
    }

    public void intakeRings(int numRings, Vector2d position, double heading) {
        IntakeController intake = controllers.get(IntakeController.class, FieldConstants.Intake);

        switch (numRings) {
            case (0):
                telemetry.addData("Sequence", "no rings to intake");
                break;

            case (1):
                telemetry.addData("Sequence", "intake 1 ring");
                intake.extend();
                intake.run(FORWARD);
                drive.followTrajectory(buildIntakeTrajectory(drive, position, heading));
                break;

            case (4):
                telemetry.addData("Sequence", "intake 4 rings");
                intake.extend();
                intake.run(FORWARD);
                drive.followTrajectory(buildIntakeTrajectory(drive, position, heading));
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
        drive.followTrajectory(buildLineTrajectory(drive, new Pose2d(position, 0)));
    }

    public Pose2d getPose() {
        return drive.getPoseEstimate();
    }
}
