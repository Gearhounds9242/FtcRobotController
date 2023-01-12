package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
public class AsyncFollowingFSM extends LinearOpMode {

    /**
     * define our mechanism motors and PID controllers
     */

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
        AQUIRE,
        IDLE
    }

    /**
     * these booleans are used to check when certain events have happened
     */

    boolean score = false; //checks whether a cone has been scored in a cycle
    boolean lift_speed = true;

    /**
     *these integers count the number of cones scored and how much we should off set the lift after each cone
     * each cone in the stack has a different height
     */
    int scored = 0;
    int offset = 0;
    /**
     * We define the current state we're on
     * Default to IDLE
     */
    State currentState = State.IDLE;

    /**
     * Define our start pose
     * This assumes we start at x: 35.25, y: -72, heading: 0 degrees (which equates 90 degrees in radians)
     */
    Pose2d startPose = new Pose2d(35, -72, Math.toRadians(90));

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

        // Set inital pose
        drive.setPoseEstimate(startPose);

        /**
         * we define the parameters of our trajectories
         */

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(32.5, -41, Math.toRadians(0)))
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .splineTo(new Vector2d(58, -21), Math.toRadians(115))
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(61.75, -14, Math.toRadians(90)))
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory4.end())
                .lineToLinearHeading(new Pose2d(58, -21, Math.toRadians(115)))
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory3.end())
                .lineToLinearHeading(new Pose2d(60, -14, Math.toRadians(90)))
                .build();

        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory3.end())
                .splineTo(new Vector2d(36, -18), Math.toRadians(180))
                .build();

        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory3.end())
                .splineTo(new Vector2d(36, -16), Math.toRadians(180))
                .splineTo(new Vector2d(10, -18), Math.toRadians(180))
                .build();

        /**
         * define and initialize our wait timer
         */

        double waitTime1 = 0.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        /**
         * starts
         */

        while (!isStarted() && !isStopRequested())
        {
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
        target  = 1000;
        sleep(1000);
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
                        turret_target = 300;
                        liftVerticle.setPosition(0.01);
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
                    if (Math.abs(turret.pos())>Math.abs(turret_target)-3 && Math.abs(turret.pos())<Math.abs(turret_target)+3) {
                        currentState = State.LIFT;
                    }
                    break;
                case LIFT:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if ((target+15) > lift.pos() && lift.pos() > (target-25)) {
                        currentState = State.WAIT_1;
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_1:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        claw.score ();
                        if(scored == 2){
                            offset = 80;
                        }
                        if(scored == 3){
                            offset = 130;
                        }
                        if(scored == 4){
                            offset = 190;
                        }
                        currentState = State.TRAJECTORY_4;
                        drive.followTrajectoryAsync(trajectory4);
                        target = 315-offset;
                        turret_target = -310;
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
                    if (scored == 4) {
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
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (Math.abs(turret.pos())>Math.abs(turret_target)-10 && Math.abs(turret.pos())<Math.abs(turret_target)+10) {
                        score = false;
                        currentState = State.AQUIRE;
                    }
                    break;
                case AQUIRE:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if ((target+15) > lift.pos() && lift.pos() > (target-15)) {
                        claw.aquire();
                        sleep(500);
                        target = 900;
                        currentState = State.WAIT_3;
                    }
                    break;
                case WAIT_3:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (lift.pos() >= 650) {
                        turret_target = 330;
                        drive.followTrajectoryAsync(trajectory3);
                        currentState = State.TRAJECTORY_3;
                    }
                    break;
                case TRAJECTORY_3:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.TURRET;
                    }
                    break;
                case TRAJECTORY_5:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
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
            if (lift_speed){
                lift.update_slow();
            }else {
                lift.update();
            }
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
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            arm_motor = hardwareMap.get(DcMotorEx.class, "turret");
            arm_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            arm_motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void update() {
            controller.setPID(p, i, d);
            int armPos = arm_motor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            arm_motor.setPower(power);

            telemetry.addData("arm_pos", armPos);
            telemetry.addData("arm_target", target);
            telemetry.addData("arm_power", power);
            telemetry.update();
        }
        public void update_slow() {
            int armPos = arm_motor.getCurrentPosition();
            arm_motor.setTargetPosition(target);
            arm_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            arm_motor.setPower(5.0);

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

            telemetry.addData("pos",  turretPos);
            telemetry.addData("target", turret_target);
            telemetry.addData("power", turret_power);
            telemetry.update();
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
            grabVerticle.setPosition(0.0);
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
