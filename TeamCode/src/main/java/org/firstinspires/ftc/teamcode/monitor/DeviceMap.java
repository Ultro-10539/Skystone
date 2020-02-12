package org.firstinspires.ftc.teamcode.monitor;

import android.content.Context;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcontroller.ultro.listener.UltroVuforia;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
//import java.util.concurrent.CompletableFuture;


public final class DeviceMap {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";

    private static DeviceMap INSTANCE;
    private Telemetry telemetry;
    private OpMode currentOpMode;

    private ExpansionHubEx expansionHub3, expansionHub2;

    private DcMotor leftTop, leftBottom, rightTop, rightBottom,
            leftIntake, rightIntake, conveyer, lift;
    private DcMotor[] driveMotors;
    private DcMotor[] intakeMotors;
    private DcMotor[] allMotors;

    private Servo leftAuto, rightAuto, leftFinger, rightFinger, foundationLeft, foundationRight, arm1, arm2, claw;
    private Servo[] servos;

    private RevBlinkinLedDriver ledDriver;

    private BNO055IMU imu = null;
    private Orientation lastAngles;
    private double globalAngle = 0;

    private UltroVuforia vuforia;
    private TFObjectDetector tfod;
    private OpenCvCamera camera;

    private ColorSensor sensorColorLeft, sensorColorRight;
    private ColorSensor[] colorSensors;

    private DistanceSensor distanceRight, distanceBack, distanceLeft, sensorColorLeftDist, sensorColorRightDist;
    private DistanceSensor[] distanceSensors;

    public DeviceMap(final HardwareMap map) {
        //for later

        INSTANCE = this;
        setUpExpansionHub(map);
        //CompletableFuture.allOf(
        //).thenRunAsync(() -> {
        //}, service);
    }
    public void setupAll(HardwareMap map) {
        setUpExpansionHub(map);
        setUpMotors(map); //init motors
        setUpImu(map); //init imu
        setUpVuforia(map); //init vuforia
        initTfod(map); //init tensorflow
        setupServos(map); //init servos
        setupSensors(map); //init d-sensors, color-sensors
        setUpLEDs(map);
    }

    public void setUpExpansionHub(HardwareMap map) {
        this.expansionHub3 = map.get(ExpansionHubEx.class, "Expansion Hub 3");
        this.expansionHub2 = map.get(ExpansionHubEx.class, "Expansion Hub 2");
    }
    /**
     * This will just set up all the driveMotors
     * @param map
     */
    public  /*CompletableFuture<Void>*/void setUpMotors(HardwareMap map) {
        //return CompletableFuture.runAsync(() -> {
            telemetry.addLine("Setting up driveMotors");
            telemetry.update();
            leftTop = map.get(ExpansionHubMotor.class, "LeftTop");
            leftBottom = map.get(ExpansionHubMotor.class, "LeftBottom");
            rightTop = map.get(ExpansionHubMotor.class, "RightTop");
            rightBottom = map.get(ExpansionHubMotor.class, "RightBottom");


            leftIntake = map.get(ExpansionHubMotor.class, "leftIntake");
            rightIntake = map.get(ExpansionHubMotor.class, "rightIntake");

            conveyer = map.get(ExpansionHubMotor.class, "conveyor");
            lift = map.get(ExpansionHubMotor.class, "lift");

            this.driveMotors = new DcMotor[]{leftTop, rightTop, leftBottom, rightBottom};
            this.intakeMotors = new DcMotor[] {
                     leftIntake, rightIntake, conveyer
            };
            this.allMotors = new DcMotor[]{leftTop, rightTop, leftBottom, rightBottom,
                    leftIntake, rightIntake, conveyer, lift
            };
            for(DcMotor motor : this.driveMotors) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            for(DcMotor motor : this.allMotors) {
                motor.setPower(0);
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.REVERSE);

            rightTop.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBottom.setDirection(DcMotorSimple.Direction.REVERSE);

            conveyer.setDirection(DcMotorSimple.Direction.REVERSE);
            telemetry.addLine("Finished setting up driveMotors");
        //}, service);

    }

    public void setupServos(HardwareMap map){
        telemetry.addLine("Setting up servos");
        telemetry.update();

        leftAuto = map.get(Servo.class, "LeftAuto");
        rightAuto = map.get(Servo.class, "RightAuto");
        leftFinger = map.get(Servo.class, "LeftFinger");
        rightFinger = map.get(Servo.class, "RightFinger");
        foundationLeft = map.get(Servo.class, "foundationLeft");
        foundationRight = map.get(Servo.class, "foundationRight");

        arm1 = map.get(Servo.class, "Arm1");
        arm2 = map.get(Servo.class, "Arm2");
        claw = map.get(Servo.class, "Claw");

        this.servos =  new Servo[]{
                leftAuto, rightAuto, leftFinger, rightFinger, foundationLeft, foundationRight, arm1, arm2
        };
    }

    public void setupSensors(HardwareMap map){
        telemetry.addLine("Setting up sensors");
        telemetry.addLine();

        sensorColorLeft = map.get(ColorSensor.class, "ColorLeft");
        sensorColorRight = map.get(ColorSensor.class, "ColorRight");
        sensorColorLeftDist = map.get(DistanceSensor.class, "ColorLeft");
        sensorColorRightDist = map.get(DistanceSensor.class, "ColorRight");

        distanceLeft = map.get(DistanceSensor.class, "DistanceLeft");
        distanceRight = map.get(DistanceSensor.class, "DistanceRight");
        distanceBack = map.get(DistanceSensor.class, "DistanceBack");

        this.colorSensors = new ColorSensor[] {
                sensorColorLeft, sensorColorRight
        };

        this.distanceSensors = new DistanceSensor[] {
                distanceRight, distanceBack, distanceLeft, sensorColorLeftDist, sensorColorRightDist
        };

    }
    public /*CompletableFuture<Void>*/void setUpImu(HardwareMap map) {
        //expansionHub3.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.STANDARD_100K);
        //return CompletableFuture.runAsync(() -> {

            telemetry.addLine("Setting up imu");
            telemetry.update();
            imu = map.get(BNO055IMU.class, "imu");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu.initialize(parameters);

            /*
            LinearOpMode linear = null;
            if(getCurrentOpMode() instanceof AutoOpMode) {
                linear = (LinearOpMode) getCurrentOpMode();
            }

            while ((linear != null && linear.opModeIsActive()) && !imu.isGyroCalibrated()) {
                telemetry.addData("calibrated", imu.isGyroCalibrated());
                telemetry.update();
            }
            telemetry.addData("calibrated", imu.isGyroCalibrated());

             */
        //}, service);
    }

    public void setUpLEDs(HardwareMap map) {
        ledDriver = map.get(RevBlinkinLedDriver.class, "blinkin");
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

    }
    public void setupOpenCV(HardwareMap map) {
        Context appContext = map.appContext;
        int cameraMonitorViewId = appContext.getResources().getIdentifier("cameraMonitorViewId", "id", appContext.getPackageName());
        camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.openCameraDevice();

    }
    public void setUpVuforia(HardwareMap map) {
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        Context appContext = map.appContext;
        int cameraMonitorViewId = appContext.getResources().getIdentifier("cameraMonitorViewId", "id", appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXOOSnD/////AAAAGQJxEkkI+EqdsPLGvMTzmRoBoW5g1d+xB7S06ymDvyNs48WqxFYMIeVVTSdkgHLwjsFVHBgVACzNkxwNXQ5zO9ED9CS11B+/cDS7CAbFLYzTlbDsyeX/NaEIOBm9v4ErL7uM6xtTXoKFKyJiFJiRe3ux4A6MXRHrvnkGqaJ9fBle9B2OTuyOe62gv5PFuTvil1DSvBosIXQmTiHosTW39OBcR81+ykeJXeiUA8vwBp1ueAP+9eYTP0U6VWDwRm0dUJ+CbvVIriauyP6pWj7dnCufomc58E7GiJbsLEN+Uj3H7J1uJG3K6O8azwEc+8BKw8tTsEdg+lJ47CbsYR6fFFfHVwA3K193cnC5U/RmRnX0";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = new UltroVuforia(parameters);
    }

    public void initTfod(HardwareMap hardwareMap) {
        if(vuforia == null) {
            telemetry.addLine("vuforia is null!");
            telemetry.update();
            return;
        }else if(!ClassFactory.getInstance().canCreateTFObjectDetector()) {
            telemetry.addLine("TFLOW not supported");
            telemetry.update();
            return;
        }
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
        tfod.activate();
    }

    public void deactivateOpenCV() {
        if(camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }
    public void deactivateVuforia() {
        if(vuforia != null)
            vuforia.close();
    }
    public void deactivateTfod() {
        if(tfod != null)
            tfod.deactivate();
    }
    public void deactivateLedDriver() {
        ledDriver.close();
    }
    public ExpansionHubEx getExpansionHub3() {
        return expansionHub3;
    }
    public ExpansionHubEx getExpansionHub2() {
        return expansionHub2;
    }

    //The methods below get all the driveMotors
    public DcMotor getLeftTop() {
        return leftTop;
    }
    public DcMotor getLeftBottom() {
        return leftBottom;
    }
    public DcMotor getRightTop() {
        return rightTop;
    }
    public DcMotor getRightBottom() {
        return rightBottom;
    }
    public DcMotor[] getDriveMotors() {
        return driveMotors;
    }

    public DcMotor getLeftIntake() {
        return leftIntake;
    }
    public DcMotor getRightIntake() {
        return rightIntake;
    }

    public DcMotor getConveyer() {
        return conveyer;
    }

    public Servo getLeftAuto(){
        return leftAuto;
    }
    public Servo getRightAuto(){
        return rightAuto;
    }
    public Servo getLeftFinger(){
        return leftFinger;
    }
    public Servo getRightFinger(){
        return rightFinger;
    }

    public Servo[] getServos() {
        return servos;
    }

    public ColorSensor getSensorColorLeft(){
        return sensorColorLeft;
    }
    public ColorSensor getSensorColorRight(){
        return sensorColorRight;
    }

    public ColorSensor[] getColorSensors() {
        return colorSensors;
    }

    public DistanceSensor getDistanceRight() {
        return distanceRight;
    }

    public DistanceSensor getDistanceBack() {
        return distanceBack;
    }

    public DistanceSensor getDistanceLeft() {
        return distanceLeft;
    }

    public DistanceSensor getSensorColorLeftDist() {
        return sensorColorLeftDist;
    }

    public DistanceSensor getSensorColorRightDist() {
        return sensorColorRightDist;
    }

    public DistanceSensor[] getDistanceSensors() {
        return distanceSensors;
    }

    public DcMotor[] getIntakeMotors() {
        return intakeMotors;
    }

    public DcMotor getLift() {
        return lift;
    }

    public Servo getArm1() {
        return arm1;
    }

    public Servo getArm2() {
        return arm2;
    }

    public Servo getClaw() {
        return claw;
    }

    public Servo getFoundationLeft(){
        return foundationLeft;
    }

    public Servo getFoundationRight(){
        return foundationRight;
    }

    public DcMotor[] getAllMotors() {
        return allMotors;
    }

    public BNO055IMU getImu() {
        return imu;
    }
    public VuforiaLocalizer getVuforia() {
        return vuforia;
    }

    public static DeviceMap getInstance() {
        if(INSTANCE == null) throw new RuntimeException("the constructor must be called first");
        return INSTANCE;
    }
    public static DeviceMap getInstance(HardwareMap map) {
        if(INSTANCE == null && map != null) INSTANCE = new DeviceMap(map);
        return INSTANCE;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void setTelemetry(Telemetry ttelemetry) {
        telemetry = ttelemetry;
    }

    public OpMode getCurrentOpMode() {
        return currentOpMode;
    }

    public void setCurrentOpMode(OpMode currentOpMode) {
        this.currentOpMode = currentOpMode;
    }

    public TFObjectDetector getTfod() {
        return tfod;
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    public RevBlinkinLedDriver getLedDriver() {
        return ledDriver;
    }
}
