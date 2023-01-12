package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;


@TeleOp(name="Mechanum_Arcade", group="TeleOp")

/* This program is the robot's main TeleOp program which gets run
 * constistently every TeleOperated period. This allows for driver control
 * of the drivetrain, and operator control of all other subsytems on the robot.*/

public class Mechanum extends OpMode
{


    // Declare the hardwareMap for the robot
    private GearHoundsHardware robot = new GearHoundsHardware();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

	private double shift = 1.0;

    private double offset = 0;

    private boolean grab = false;

    private PIDController controller;
    private PIDController turret_controller;

    public static double p = 0.02, i = 0, d = 0.0004;
    public static double f = 0.4;

    public static double tp = 0.02, ti = 0, td = 0.0004;

    public static int target = 0;
    public static int turret_target = 0;

    private final double ticks_in_degree = 560/360;



    @Override
    public void init() {

        Pose2d myPose = PoseStorage.currentPose;

        telemetry.addData("x", myPose.getX());
        telemetry.addData("y", myPose.getY());
        telemetry.addData("heading", myPose.getHeading());
        telemetry.update();

        offset = Math.toDegrees(myPose.getHeading());

        // Intializes the hardwareMap found in "GearHoundsHardware" class
        robot.init(hardwareMap);
        controller = new PIDController(p, i, d);
        turret_controller = new PIDController(tp, ti, td);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        robot.liftVerticle.setPosition(1.0);
        robot.grabVerticle.setPosition(0.0);
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        drive();

        if (gamepad2.left_trigger > 0.1) {
            turret_target = 300;
        } else if (gamepad2.right_trigger > 0.1) {
            turret_target = -300;
        } else if (gamepad2.x){
            turret_target = 0;
        } else {

        }

        if(robot.lift.getCurrentPosition() > 10) {
            if (gamepad2.left_bumper) {
                robot.liftVerticle.setPosition(0.99);
            }
            if (gamepad2.right_bumper) {
                robot.liftVerticle.setPosition(0.01);
            }
        }

        if (gamepad1.right_bumper) {
            robot.grabVerticle.setPosition(0.4);
        }
        if (gamepad1.left_bumper) {
            robot.grabVerticle.setPosition(0.0);
        }
        /*if (gamepad2.x) {
            robot.grabVerticle.setPosition(0.0);
            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.liftVerticle.setPosition(0.99);
        }*/
        if (gamepad1.x) {
            target = 0;
        } else if (gamepad2.a|| gamepad1.a) {
            target = 1100;
        } else if (gamepad2.b|| gamepad1.b) {
            target = 1800;
        } else if (gamepad2.y|| gamepad1.y) {
            target = 2500;
        } else {

        }

        if (gamepad1.left_stick_button) {
            shift = 0.3;
        } else  {
            shift = 1;
        }

        PID_update();
        turret_PID_update();

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP

     Override
     */
    public void stop() {

        // set all the motors to stop
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }

    public void PID_update () {
        controller.setPID(p, i, d);
        int armPos = robot.lift.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        robot.lift.setPower(power);

        telemetry.addData("pos",  armPos);
        telemetry.addData("target", target);
        telemetry.addData("power", power);
        telemetry.update();
    }

    public void turret_PID_update () {
        controller.setPID(tp, ti, td);
        int turretPos = robot.turret.getCurrentPosition();
        double turret_pid = turret_controller.calculate(turretPos, turret_target);

        double turret_power = turret_pid;

        robot.turret.setPower(turret_power);

        telemetry.addData("pos",  turretPos);
        telemetry.addData("target", turret_target);
        telemetry.addData("power", turret_power);
        telemetry.update();
    }

    public void drive() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        double r = Math.hypot(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        robotAngle = robotAngle - Math.toRadians(angle);
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.leftFront.setPower(v1 * shift);
        robot.rightFront.setPower(-v2 * shift);
        robot.leftBack.setPower(-v3 * shift);
        robot.rightBack.setPower(v4 * shift);
    }


}

