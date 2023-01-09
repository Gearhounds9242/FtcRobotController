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

    public DcMotorEx  turret;
    public Servo liftVerticle;
    public Servo grabVerticle;
    private PIDController controller;

    private DcMotorEx arm_motor;

    //AprilTagAutonomousInitDetectionExample camera = new AprilTagAutonomousInitDetectionExample();

    private final double ticks_in_degree = 560.0/360.0;

    public static double p = 0.02, i = 0, d = 0.0004;
    public static double f = 0.4;

    public int target = 0;

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        TRAJECTORY_4,
        TRAJECTORY_5,
        TRAJECTORY_6,
        WAIT_1,
        WAIT_2,
        TURRET,
        LIFT,
        DEPOSITE,
        ROTATE,
        AQUIRE,
        IDLE
    }

    boolean next = false;
    boolean score = false;


    int rotation = 0;
    int scored = 0;

    int offset = 0;

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(35.25, -72, Math.toRadians(90));



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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
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

        // Initialize our lift
        Lift lift = new Lift(hardwareMap);

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Turret turret = new Turret(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(31, -41, Math.toRadians(0)))
                //.splineTo(new Vector2d(63,-42), Math.toRadians(90))
                //.splineTo(new Vector2d(60, -18), Math.toRadians(115))
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .splineTo(new Vector2d(58.25, -20), Math.toRadians(115))
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(62.25, -18.3, Math.toRadians(115)))
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory4.end())
                .lineToLinearHeading(new Pose2d(59, -20, Math.toRadians(115)))
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .lineToLinearHeading(new Pose2d(60, -40, Math.toRadians(90)))
                //.lineToLinearHeading(new Pose2d(60, -36, Math.toRadians(0)))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();
        double waitTime2 = 2.0;
        ElapsedTime waitTimer2 = new ElapsedTime();

        //waitForStart();
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

        double strafe = 0.0;
        if(tagOfInterest == null || tagOfInterest.id == left) {
            strafe = 14.0;
        } else if (tagOfInterest.id == middle) {
            strafe = 36.0;
        } else {
            next = true;
        }

        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .lineToLinearHeading(new Pose2d(strafe, -40, Math.toRadians(90)))
                //.lineToLinearHeading(new Pose2d(60, -36, Math.toRadians(0)))
                .build();

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
        sleep(2000);
        rotation = 320;
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
                    if (turret.pos() == rotation) {
                        currentState = State.LIFT;
                    }
                    break;
                case LIFT:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if ((target+15) > lift.pos() && lift.pos() > (target-15)) {
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
                        currentState = State.TRAJECTORY_4;
                        drive.followTrajectoryAsync(trajectory4);
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
                    if (scored == 6) {
                        liftVerticle.setPosition(0.99);
                        rotation = 0;
                        target = 0;
                        currentState = State.TRAJECTORY_5;
                        drive.followTrajectoryAsync(trajectory5);
                    } else if (score = true) {
                        if(scored == 2){
                            offset = 80;
                        }
                        if(scored == 4){
                            offset = 120;
                        }
                        currentState = State.ROTATE;
                        rotation = -295;
                    }
                    break;
                case ROTATE:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (turret.pos()==rotation) {
                        score = false;
                        target = 335-offset;
                        currentState = State.WAIT_2;
                        waitTimer2.reset();
                    }
                    break;
                case WAIT_2:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer2.seconds() >= waitTime2) {
                        currentState = State.AQUIRE;
                    }
                    break;
                case AQUIRE:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if ((target+15) > lift.pos() && lift.pos() > (target-45)) {
                        //sleep(2000);
                        claw.aquire();
                        currentState = State.TRAJECTORY_3;
                        target = 1000;
                        sleep(500);
                        rotation = 340;
                        drive.followTrajectoryAsync(trajectory3);
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
                        target = 0;
                        rotation = 0;
                        if (next == true) {
                            currentState = State.IDLE;
                        }
                        else {
                            currentState = State.TRAJECTORY_6;
                            drive.followTrajectoryAsync(trajectory6);
                        }
                    }
                    break;
                case TRAJECTORY_6:
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

            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);
            telemetry.addData("power", power);
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
            turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        public void update() {
            if(turret.getCurrentPosition()>rotation){
                turret.setTargetPosition(rotation);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(-0.3);
            } else {
                turret.setTargetPosition(rotation);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(0.3);
            }
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
            liftVerticle.setPosition(0.01);
            sleep(1000);
            grabVerticle.setPosition(0.0);
            sleep(1000);
            //liftVerticle.setPosition(0.99);
            score = true;
            scored +=1;
        }
        public void aquire () {
            //sleep (2000);
            grabVerticle.setPosition(0.5);
            sleep(250);
        }
    }

}
