package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

public interface Sequence {
    void run();
    void moveToSquares();
    void dropWobble();
    void moveToStart();
    void collectWobble();
    void moveToShoot();
    void shootRings();
    void intakeRings();
    void moveToLaunchLine();
    void stop();
}
