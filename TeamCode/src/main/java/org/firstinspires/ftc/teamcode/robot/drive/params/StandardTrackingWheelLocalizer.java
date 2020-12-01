package org.firstinspires.ftc.teamcode.robot.drive.params;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6889764; // in (35mm/2 = 17.5mm)
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 16.561849817; // in; distance between the left and right wheels (calibrated)

    public static double FORWARD_OFFSET = 0.39; // in; offset of the lateral wheel (about 1cm forward)

    public static double X_MULTIPLIER = 1; // Multiplier in the X direction (forward)
    public static double LEFT_MULTIPLIER = 1.011038261; //left encoder multiplier
    public static double RIGHT_MULTIPLIER = 1.011038261; //right encoder multiplier
    public static double LATERAL_MULTIPLIER = 1.011038261; // Multiplier in the Y direction (strafe)
    //TODO: Tune X/Y Multiplier for error (https://www.learnroadrunner.com/dead-wheels.html#tuning-three-wheel)

    /**
     * Options for inaccurate heading
     * Retune everyhing
     * Lower speed
     * Tune LEFT_MULTIPLIER
     * Use splines that turn instead of direct turns
     * Tune HEADING_PID, more aggresive
     * Use Hub IMU to correct for heading error
     * Use vision targets to correct
     * Try two wheel odometry with IMU
     */
    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_rear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_rear"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_front"));

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * LEFT_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * RIGHT_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * LATERAL_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())
        );
    }
}
