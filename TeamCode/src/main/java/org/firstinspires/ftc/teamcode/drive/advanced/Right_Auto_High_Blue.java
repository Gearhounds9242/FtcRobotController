package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(group = "advanced")
public class Right_Auto_High_Blue extends LinearOpMode {

    /**
     * define our mechanism motors and PID controllers
     */
    public ModernRoboticsI2cRangeSensor rangeFinder;
    public DcMotorEx  turret;
    private DcMotorEx arm_motor;
    public Servo liftVerticle;
    public Servo grabVerticle;
    private PIDController controller;
    private PIDController turret_controller;

    /**
     * define the PID coefficients
     */

    private final double ticks_in_degree = 336.0/360.0;

    public static double p = 0.025, i = 0, d = 0.0005;
    public static double f = 0.05;

    public static double tp = 0.02, ti = 0, td = 0.0004;

    /**
     * define the target positions of the turret and the lift
     */

    public static int turret_target = 0;

    public int target = 0;

    /**
     * This enum defines our "state"
     * This is essentially just defines the possible steps our program will take
     */

    enum State {
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        TRAJECTORY_4,
        TRAJECTORY_5,
        WAIT_1,
        WAIT_3,
        TURRET,
        LIFT,
        DEPOSITE,
        ROTATE,
        ACQUIRE,
        IDLE
    }

    /**
     * these booleans are used to check when certain events have happened
     */

    boolean score = false; //checks whether a cone has been scored in a cycle

    /**
     *these integers count the number of cones scored and how much we should off set the lift after each cone
     * each cone in the stack has a different height
     */
    int scored = 0;
    int offset = 0;
    double distance = 59.75;
    double distance_2 = 27.0;
    double startDistance = 6;
    /**
     * We define the current state we're on
     * Default to IDLE
     */
    State currentState = State.IDLE;

    /**
     * Define our start pose
     * This assumes we start at x: 35.25, y: -72, heading: 90 degrees
     */


    Pose2d traj3end = new Pose2d(30, -12.75, Math.toRadians(0));

    /**
     * define our opencv and april tag pipeline
     */

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1, 2, 3 from the 36h11 family
    int left = 1;
    int middle = 2;
    int right = 3;

    AprilTagDetection tagOfInterest = null;



    @Override
    public void runOpMode() throws InterruptedException {

        /**
         *initialize our camera, opencv, and april tag pipeline
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        /**
         * starts to stream our webcam feed
         */

        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /**
         * initialize the parameters for our lift, turret, claw, PID controllers, and drivetrain
         */

        // Initialize our lift
        Dist_Sensor dist = new Dist_Sensor(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        controller = new PIDController(p, i, d);
        turret_controller = new PIDController(tp, ti, td);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Turret turret = new Turret(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /**
         * we set our start pose in relation to the center of the field.
         */
        startDistance = 7.83 + dist.update();
        Pose2d startPose = new Pose2d(startDistance, -72, Math.toRadians(90));
        // Set inital pose
        drive.setPoseEstimate(startPose);

        /**
         * we define the parameters of our trajectories
         */
        // moves in a straight line forward and turns 90 degrees at the same time.
        // This motions pushes the signal cone out of our way
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(32.5, -41, Math.toRadians(0)))
                .build();
        //move the robot in a curve around the low junction near the cone stack
        //and lines up our arm to deposit.
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(29, -13.75, Math.toRadians(0)))
                .build();
        //The next three trajectories are for parking
        //parking location 3
        Trajectory trajectory5 = drive.trajectoryBuilder(traj3end)
                .lineToLinearHeading(new Pose2d(66, -22, Math.toRadians(0)))
                .build();
        //parking location 2
        Trajectory trajectory6 = drive.trajectoryBuilder(traj3end)
                .lineToLinearHeading(new Pose2d(45, -24, Math.toRadians(90)))
                .build();
        //parking location 1
        Trajectory trajectory7 = drive.trajectoryBuilder(traj3end)
                //.splineTo(new Vector2d(36, -16), Math.toRadians(180))
                //.splineTo(new Vector2d(9, -18), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(14, -25, Math.toRadians(90)))
                .build();

        /**
         * define and initialize our wait timer
         */

        double waitTime1 = 0.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        /**
         * OpenCv begins to detect AprilTags
         */

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("dist", 35-dist.update());
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == middle || tag.id == right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if (isStopRequested()) return;

        claw.set_claws();
        target  = 920;
        sleep(500);
        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(trajectory2);
                        turret_target = 385;
                        target = 2170;
                        liftVerticle.setPosition(0.0);
                    }
                    break;
                case TRAJECTORY_2:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.TURRET;
                    }
                    break;
                case TURRET:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (Math.abs(turret.pos())>Math.abs(turret_target)-30 && Math.abs(turret.pos())<Math.abs(turret_target)+10) {
                        currentState = State.LIFT;
                    }
                    break;
                case LIFT:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if ((target+25) > lift.pos() && lift.pos() > (target-35)) {
                        currentState = State.WAIT_1;
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                        if (scored < 4) {
                            target = 2000;
                        }
                    }
                    break;
                case WAIT_1:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        claw.score ();
                        if(scored == 1){
                            offset = -35;
                        }
                        if(scored == 2){
                            offset = 40;
                        }
                        if(scored == 3){
                            offset = 140;
                        }
                        if(scored == 4){
                            offset = 240;
                        }
                        if(scored == 5){
                            offset = 305;
                        }
                        if(scored != 5) {
                            Trajectory trajectory4 = drive.trajectoryBuilder(PoseStorage.currentPose)
                                    .lineToLinearHeading(new Pose2d(distance, -16.75, Math.toRadians(0)))
                                    .build();
                            drive.followTrajectoryAsync(trajectory4);
                        }
                        distance += 0.1;
                        if (scored == 1) {
                            distance += 0.75;
                        }
                        currentState = State.TRAJECTORY_4;
                        target = 305-offset;
                        turret_target = -0;
                    }
                    break;
                case TRAJECTORY_4:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.DEPOSITE;
                    }
                    break;
                case DEPOSITE:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (scored == 5) {
                        liftVerticle.setPosition(0.99);
                        turret_target = 0;
                        target = 0;
                        if(tagOfInterest == null || tagOfInterest.id == left) {
                            drive.followTrajectoryAsync(trajectory7);
                            currentState = State.TRAJECTORY_5;
                        } else if (tagOfInterest.id == middle) {
                            drive.followTrajectoryAsync(trajectory6);
                            currentState = State.TRAJECTORY_5;
                        } else {
                            currentState = State.TRAJECTORY_5;
                            drive.followTrajectoryAsync(trajectory5);
                        }

                    } else if (score = true) {
                        currentState = State.ROTATE;

                    }
                    break;
                case ROTATE:
                    // Check if the turret is within the specified rotation tolerance
                    // If so, move on to the ACQUIRE state
                    if (Math.abs(turret.pos())>Math.abs(turret_target)-30 && Math.abs(turret.pos())<Math.abs(turret_target)+10) {
                        score = false;
                        currentState = State.ACQUIRE;
                    }
                    break;
                case ACQUIRE:
                    // Check if the lift is within the specified height tolerance
                    // If so, move on to the WAIT_3 state
                    if ((target+25) > lift.pos() && lift.pos() > (target-25)) {
                        claw.aquire();
                        sleep(300);
                        if (scored < 4) {
                            target = 2150;
                        } else {
                            target = 890;
                        }
                        currentState = State.WAIT_3;
                    }
                    break;
                case WAIT_3:
                    // Waits and Checks if the lift encoder has exceeded the specified height
                    // If so, move on to the Trajectory_3 state
                    if (lift.pos() >= 650-offset) {
                        if (scored < 4) {
                            Trajectory trajectory3 = drive.trajectoryBuilder(PoseStorage.currentPose)
                                    .lineToLinearHeading(new Pose2d(distance_2, -13.75, Math.toRadians(0)))
                                    .build();
                            drive.followTrajectoryAsync(trajectory3);
                        } else {
                            Trajectory trajectory8 = drive.trajectoryBuilder(PoseStorage.currentPose)
                                    .lineToLinearHeading(new Pose2d(50.0, -17.1, Math.toRadians(0)))
                                    .build();
                            drive.followTrajectoryAsync(trajectory8);
                        }
                        if(scored == 4) {
                            //distance_2 += 0.1;
                        }
                        if (scored < 4) {
                            turret_target = 395;
                        } else {
                            turret_target = -300;
                        }
                        currentState = State.TRAJECTORY_3;
                    }
                    break;
                case TRAJECTORY_3:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    if (!drive.isBusy()) {
                        currentState = State.TURRET;
                    }
                    break;
                case TRAJECTORY_5:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            lift.update();
            turret.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class Dist_Sensor {
        public Dist_Sensor (HardwareMap hardwareMap) {
            rangeFinder = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeFinder");

        }
        public double update () {
            return startDistance = rangeFinder.getDistance(DistanceUnit.INCH);
        }
    }
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            arm_motor = hardwareMap.get(DcMotorEx.class, "turret");
            arm_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            arm_motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        public void update() {
            int armPos = arm_motor.getCurrentPosition();
            arm_motor.setTargetPosition(target);
            arm_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            arm_motor.setVelocity(1600);

            telemetry.addData("arm_pos", armPos);
            telemetry.addData("arm_target", target);
            //telemetry.addData("arm_power", power);
            telemetry.update();
        }
        public int pos() {
            return arm_motor.getCurrentPosition();
        }
    }
    class Turret {
        public Turret(HardwareMap hardwareMap) {
            turret = hardwareMap.get(DcMotorEx.class, "Vertical");
            turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void update() {
            turret_controller.setPID(tp, ti, td);
            int turretPos = turret.getCurrentPosition();
            double turret_pid = turret_controller.calculate(turretPos, turret_target);

            double turret_power = turret_pid;

            turret.setPower(turret_power);

            /*telemetry.addData("pos",  turretPos);
            telemetry.addData("target", turret_target);
            //telemetry.addData("power", turret_power);
            telemetry.update();*/
        }
        public int pos () {
            return turret.getCurrentPosition();
        }
    }
    class Claw {
        public Claw (HardwareMap hardwareMap) {
            liftVerticle = hardwareMap.servo.get("VerticalExtentionLift");
            grabVerticle = hardwareMap.servo.get("VerticalExtentionGrab");
        }
        public void set_claws () {
            grabVerticle.setPosition(0.5);
            sleep(250);
            liftVerticle.setPosition(0.99);
        }
        public void score () {

            //sleep(1000);
            grabVerticle.setPosition(-0.0);
            sleep(200);
            //liftVerticle.setPosition(0.99);
            score = true;
            scored +=1;
        }
        public void aquire () {
            //sleep (2000);
            grabVerticle.setPosition(0.5);
            //sleep(250);
        }
    }

}
