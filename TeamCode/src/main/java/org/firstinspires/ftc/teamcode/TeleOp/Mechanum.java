package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

@TeleOp(name="Mechanum_Arcade", group="TeleOp")

public class Mechanum extends OpMode
{

    // Declare the hardwareMap for the robot
    private GearHoundsHardware robot = new GearHoundsHardware();
    // Declaring hardwareMap for a localizer

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Declare Gear shift ratio
	private double shift = 1.0;

    //Declare IMU offset for field centric driving
    private double offset = 0;

    //Declaring the PID controllers and coefficients

    private PIDController controller;
    private PIDController turret_controller;

    public static double p = 0.02, i = 0, d = 0.0004;
    public static double f = 0.4;

    public static double tp = 0.02, ti = 0, td = 0.0004;

    public static int target = 0;
    public static int turret_target = 0;

    private final double ticks_in_degree = 560/360;

    enum DriveState {
        LOW,
        MEDIUM,
        HIGH,
        NORMAL,
        IDLE
    }

    enum LiftState {
        LOW,
        MEDIUM,
        HIGH,
        STACK,
        GROUND,
        IDLE
    }

    enum TurretState {
        FORWARD,
        LEFT,
        RIGHT,
        BACK_LEFT,
        BACK_RIGHT,
        IDLE
    }

    enum ClawState {
        OPENED,
        CLOSED,
        CONE,
        IDLE
    }

    enum HorizontalLiftState {
        EXTENDED,
        RETRACTED,
        IDLE
    }

    DriveState driveState = Mechanum.DriveState.IDLE;
    LiftState liftState = Mechanum.LiftState.IDLE;
    TurretState turretState = Mechanum.TurretState.IDLE;
    ClawState clawState = Mechanum.ClawState.IDLE;
    HorizontalLiftState horizontalLiftState = Mechanum.HorizontalLiftState.IDLE;

    @Override
    public void init() {

        Pose2d myPose = PoseStorage.currentPose;

        telemetry.addData("heading", Math.toDegrees(myPose.getHeading()));
        offset = -myPose.getHeading();
        telemetry.update();
        //offset = Math.toDegrees(myPose.getHeading());

        // Intializes the hardwareMap found in "GearHoundsHardware" class
        robot.init(hardwareMap);
        controller = new PIDController(p, i, d);
        turret_controller = new PIDController(tp, ti, td);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
        robot.liftVerticle.setPosition(1.0);
        robot.grabVerticle.setPosition(0.0);
        horizontalLiftState = HorizontalLiftState.RETRACTED;
        clawState = ClawState.OPENED;
        driveState = DriveState.NORMAL;
        liftState = LiftState.GROUND;
        turretState = TurretState.FORWARD;
    }

    @Override
    public void loop() {
        switch (driveState) {
            case NORMAL:
            case LOW:
            case MEDIUM:
            case HIGH:
                if (gamepad1.left_stick_button) {
                    shift = 0.25;
                    driveState = DriveState.LOW;
                } else if (gamepad1.right_stick_button && liftState != LiftState.HIGH)  {
                    shift = 1.0;
                    driveState = DriveState.NORMAL;
                } else  {

                }
                break;
        }
        switch (liftState) {
            case GROUND:
            case STACK:
            case LOW:
            case MEDIUM:
            case HIGH:
                if (gamepad1.x) {
                    target = 0;
                    liftState = LiftState.GROUND;
                    robot.grabVerticle.setPosition(0.0);
                    clawState = ClawState.OPENED;
                    robot.liftVerticle.setPosition(0.99);
                    horizontalLiftState = HorizontalLiftState.RETRACTED;
                } else if (gamepad2.a|| gamepad1.a) {
                    target = 970;
                    liftState = LiftState.LOW;
                    shift = 0.55;
                    driveState = DriveState.HIGH;
                } else if (gamepad2.b|| gamepad1.b) {
                    target = 1590;
                    liftState = LiftState.MEDIUM;
                    shift = 0.35;
                    driveState = DriveState.MEDIUM;
                } else if (gamepad2.y|| gamepad1.y) {
                    target = 2190;
                    liftState = LiftState.HIGH;
                    shift = 0.25;
                    driveState = DriveState.LOW;
                } else if (gamepad2.dpad_up){
                    target = 200;
                    liftState = LiftState.STACK;
                    shift = 0.25;
                    driveState = DriveState.LOW;
                    robot.liftVerticle.setPosition(0.01);
                    horizontalLiftState = HorizontalLiftState.EXTENDED;
                } else {

                }
                break;
        }
        switch (turretState) {
            case FORWARD:
            case LEFT:
            case RIGHT:
            case BACK_LEFT:
            case BACK_RIGHT:
                if (gamepad2.right_stick_button || gamepad1.dpad_down) {
                    turret_target = 595;
                    turretState = TurretState.BACK_RIGHT;
                } else if (gamepad2.left_stick_button) {
                    turret_target = -595;
                    turretState = TurretState.BACK_LEFT;
                } else if (gamepad2.right_trigger > 0.1) {
                    turret_target = 300;
                    turretState = TurretState.RIGHT;
                } else if (gamepad2.left_trigger > 0.1) {
                    turret_target = -300;
                    turretState = TurretState.LEFT;
                } else if (gamepad2.x || gamepad1.x) {
                    turret_target = 0;
                    turretState = TurretState.FORWARD;
                } else {

                }
                break;
        }
        switch (clawState) {
            case OPENED:
            case CLOSED:
            case CONE:
                if (gamepad1.right_bumper) {
                    robot.grabVerticle.setPosition(0.5);
                    clawState = ClawState.CLOSED;
                } else if (gamepad1.left_bumper) {
                    if (liftState == LiftState.LOW) {
                        if (turretState == TurretState.BACK_LEFT || turretState == TurretState.BACK_RIGHT) {
                            if (horizontalLiftState == HorizontalLiftState.EXTENDED) {
                                robot.grabVerticle.setPosition(0.0);
                                clawState = ClawState.OPENED;
                                robot.liftVerticle.setPosition(0.99);
                                horizontalLiftState = HorizontalLiftState.RETRACTED;
                            } else {
                                robot.liftVerticle.setPosition(0.01);
                                horizontalLiftState = HorizontalLiftState.EXTENDED;
                            }
                        } else {
                            robot.grabVerticle.setPosition(0.0);
                            clawState = ClawState.OPENED;
                            robot.liftVerticle.setPosition(0.99);
                            horizontalLiftState = HorizontalLiftState.RETRACTED;
                        }
                    } else {
                        robot.grabVerticle.setPosition(0.0);
                        clawState = ClawState.OPENED;
                        robot.liftVerticle.setPosition(0.99);
                        horizontalLiftState = HorizontalLiftState.RETRACTED;
                    }
                } else {

                }
                break;
        }
        switch (horizontalLiftState) {
            case RETRACTED:
            case EXTENDED:
                if (gamepad2.left_bumper) {
                    robot.liftVerticle.setPosition(0.99);
                    horizontalLiftState = HorizontalLiftState.RETRACTED;
                } else if (gamepad2.right_bumper) {
                    robot.liftVerticle.setPosition(0.01);
                    horizontalLiftState = HorizontalLiftState.EXTENDED;
                } else {

                }
                break;

        }

        if (gamepad1.dpad_up){
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            double angle = angles.firstAngle;
            offset = Math.toRadians(angle);
        }

        PID_update();
        turret_PID_update();
        drive_update();
        telemetry.update();
    }

    public void stop() {

        // set all the motors to stop
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.lift.setPower(0);
        driveState = DriveState.IDLE;
        liftState = LiftState.IDLE;
        turretState = TurretState.IDLE;
        clawState = ClawState.IDLE;
        horizontalLiftState = HorizontalLiftState.IDLE;
    }

    public void PID_update () {
        controller.setPID(p, i, d);
        int armPos = robot.lift.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        robot.lift.setPower(power);
    }

    public void turret_PID_update () {
        controller.setPID(tp, ti, td);
        int turretPos = robot.turret.getCurrentPosition();
        double turret_pid = turret_controller.calculate(turretPos, turret_target);

        double turret_power = turret_pid;

        robot.turret.setPower(turret_power);
    }

    public void drive_update() {
        /**gets the angle from the imu**/
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        /**gets squared values from the driver's stick input**/
        double r = Math.hypot(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
        /**finds the desired angle that the driver wants to move the robot**/
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        /**sets the movement angle by finding the difference of the robots angle, the input angle and the offset value
         * the offset value is set by the the driver if the imu does not reset after auto*/
        robotAngle = robotAngle - Math.toRadians(angle) + offset;
        /** sets the turning value */
        double rightX = gamepad1.right_stick_x;
        /** calculates the motor powers using trig functions and previously found values */
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        /** sets the motor velocities, which are multiplied the the weighted motor powers
         * and the electronic gear shift */
        robot.leftFront.setVelocity(4000 * v1 * shift);
        robot.rightFront.setVelocity(4000 * -v2 * shift);
        robot.leftBack.setVelocity(4000 * -v3 * shift);
        robot.rightBack.setVelocity(4000 * v4 * shift);
    }
}