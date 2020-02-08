package org.firstinspires.ftc.teamcode.monitor;

public final class RobotData {
    public static int leftTop, leftBottom, rightTop, rightBottom, leftIntake, rightIntake, conveyer, lift;

    public static void updateValues(int leftTopCount, int leftBottomCount, int rightTopCount, int rightBottomCount, int leftIntakeCount, int rightIntakeCount, int conveyerCount, int liftCount) {
        leftTop = leftTopCount;
        leftBottom = leftBottomCount;
        rightTop = rightTopCount;
        rightBottom = rightBottomCount;
        leftIntake = leftIntakeCount;
        rightIntake = rightIntakeCount;
        conveyer = conveyerCount;
        lift = liftCount;
    }
}
