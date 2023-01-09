package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 * Motor channel:  Left  drive motor:        "leftDrive"
 * Motor channel:  Right drive motor:        "rightDrive"
 */

/* This OpMode serves to assign all members of the robot, that
    includes all motors, servos, sensors, and constant variables.
    This class is solely used to assign digital devices to real
    hardware devices on the robot. */

public class GearHoundsHardware extends Hardware
{
    public HardwareMap robotMap;

    // Drivetrain Members
    public DcMotor  leftFront;
    public DcMotor  rightFront;
    public DcMotor  leftBack;
    public DcMotor  rightBack;
    public DcMotor  turret;
    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    public DcMotor lift;
    private Orientation             lastAngles = new Orientation();
    private double                  globalAngle;
    public Servo liftVerticle;
    public Servo grabVerticle;
    public DistanceSensor LDistance;
    public DistanceSensor RDistance;
    public DistanceSensor FDistance;

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    public static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private static final String VUFORIA_KEY = "AXz80k3/////AAABmSz+ffpb50UFjxexDIQrdREtWeNYe/Lhq2bfLopChdrBMqleyJlZ72ffx6b7KcwyxjYg3+sdCiyO/eLR1zjXA+U4PT1+XD6gFozJfHMusp1Fch+8uw9ouQFI9FwVfkwvu0xBVBSKFhTPNEyf/5/IZuTA5OvowgSAllZD29lFZn+6L72/JVjKNdGqx316HBcGZ3whIGgVxAHsCd/bY8a7ObKKoxA8a358vFSPD/M0Y0CjDneJCdI7jkd/7iwFO1UGkGmG88yKfPfo4TPEnjuP/5moSBcMJlnDkh1xFuUs4+aPgVRP/C/tehahle/3AAsJmdEWAx9isxpdBlsngr+AXEnJPLTLT6/lZ6xaKqufV3wJ";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;

    /* Constructor */
    public GearHoundsHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        robotMap = ahwMap;

        // Define and Initialize Motors for drivetrain
        leftFront  = robotMap.get(DcMotor.class, "leftFront");
        rightFront = robotMap.get(DcMotor.class, "rightFront");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack  = robotMap.get(DcMotor.class, "leftBack");
        rightBack = robotMap.get(DcMotor.class, "rightBack");
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        lift = robotMap.get(DcMotorEx.class, "turret");
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setDirection(DcMotorSimple.Direction.REVERSE);

        turret = robotMap.get(DcMotorEx.class, "Vertical");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //turret.setDirection(DcMotorSimple.Direction.REVERSE);

        // Defines the REV Hub's internal IMU (Gyro)
        imu = robotMap.get(BNO055IMU.class, "imu");

        // Defines the parameters for the gyro (units)
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled      = true;
        imuParameters.loggingTag          = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Intializes the parameters previously defined
        imu.initialize(imuParameters);

        LDistance =robotMap.get(DistanceSensor.class, "LDistance");
        RDistance =robotMap.get(DistanceSensor.class, "RDistance");
        FDistance =robotMap.get(DistanceSensor.class, "FDistance");

        // Intializes the parameters previously defined
        liftVerticle = robotMap.servo.get("VerticalExtentionLift");
        grabVerticle = robotMap.servo.get("VerticalExtentionGrab");
        //resetAngle();
    }
    public void resetAngle()
    {
        //don't use resetAngle, it messes with the absolute heading
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = robotMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = robotMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", robotMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}


