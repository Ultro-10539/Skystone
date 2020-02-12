package org.firstinspires.ftc.teamcode.monitor;

public final class RobotData {
    private static int leftTop, leftBottom, rightTop, rightBottom, leftIntake, rightIntake, conveyer, lift;
    private static double distLeft, distRight, distBack, distColorLeft, distColorRight;

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
                "leftTop=" + leftTop +
                ", leftBottom=" + leftBottom +
                ", rightTop=" + rightTop +
                ", rightBottom=" + rightBottom +
                ", leftIntake=" + leftIntake +
                ", rightIntake=" + rightIntake +
                ", conveyer=" + conveyer +
                ", lift=" + lift +
                ", distLeft=" + distLeft +
                ", distRight=" + distRight +
                ", distBack=" + distBack +
                ", distColorLeft=" + distColorLeft +
                ", distColorRight=" + distColorRight +
                '}';
    }

    public static int getLeftTop() {
        return leftTop;
    }

    public static int getLeftBottom() {
        return leftBottom;
    }

    public static int getRightTop() {
        return rightTop;
    }

    public static int getRightBottom() {
        return rightBottom;
    }

    public static int getLeftIntake() {
        return leftIntake;
    }

    public static int getRightIntake() {
        return rightIntake;
    }

    public static int getConveyer() {
        return conveyer;
    }

    public static int getLift() {
        return lift;
    }

    public static double getDistLeft() {
        return distLeft;
    }

    public static double getDistRight() {
        return distRight;
    }

    public static double getDistBack() {
        return distBack;
    }

    public static double getDistColorLeft() {
        return distColorLeft;
    }

    public static double getDistColorRight() {
        return distColorRight;
    }
}
