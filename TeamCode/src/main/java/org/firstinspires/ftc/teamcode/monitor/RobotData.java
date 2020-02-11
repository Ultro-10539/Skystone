package org.firstinspires.ftc.teamcode.monitor;

public final class RobotData {
    public static int leftTop, leftBottom, rightTop, rightBottom, leftIntake, rightIntake, conveyer, lift;
    public static double distLeft, distRight, distBack, distColorLeft, distColorRight;

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

    public static void updateDistanceValues(double distLeftM, double distRightM, double distBackM, double distColorLeftM, double distColorRightM) {
        distLeft = distLeftM;
        distRight = distRightM;
        distBack = distBackM;
        distColorLeft = distColorLeftM;
        distColorRight = distColorRightM;
    }

    public static String hello() {
        return "RobotData{" +
                "leftTop=" + leftTop + "\n" +
                ", leftBottom=" + leftBottom + "\n" +
                ", rightTop=" + rightTop + "\n" +
                ", rightBottom=" + rightBottom + "\n" +
                ", leftIntake=" + leftIntake + "\n" +
                ", rightIntake=" + rightIntake + "\n" +
                ", conveyer=" + conveyer + "\n" +
                ", lift=" + lift + "\n" +
                ", distLeft=" + distLeft + "\n" +
                ", distRight=" + distRight + "\n" +
                ", distBack=" + distBack + "\n" +
                ", distColorLeft=" + distColorLeft + "\n" +
                ", distColorRight=" + distColorRight + "\n" +
                '}';
    }
}
