/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;


@Autonomous(name="Robot: Auto Drive By Encoder R", group="Robot")
public class MoveCommand9242 extends LinearOpMode {
    private enum ScoringPosition {LEFT, CENTER, RIGHT, NONE}

    private ScoringPosition position;

    enum State {
        SETUP,
        FORWARD,   // First, move forward into position
        TURRET_1,  // aim turret at medium goal
        LIFT_1,    // Set lift to medium goal height
        WAIT_1,
        DEPOSIT,   // drop preset cone
        LIFT_2,    // retract lift
        TURRET_2,  // align turret to zero
        ALIGN,     // park robot and zero all mechanisms
        WAIT_2,
        PARK,
        IDLE
    }

    State currentState = State.IDLE;

    // Declare the hardwareMap for the robot
    private GearHoundsHardware robot = new GearHoundsHardware();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private PIDController controller;

    private DcMotorEx arm_motor;

    public static double p = 0.03, i = 0, d = 0.001;
    public static double f = 0.1;

    public static int target = 0;

    boolean claws = false;
    boolean next = false;
    boolean scored = false;

    int distance_1 = 850;
    int distance_2 = 1000;

    int rotation = 0;

    private final double ticks_in_degree = 560.0/360.0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        Lift lift = new Lift(hardwareMap);

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int distance_1 = 500;

        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        double waitTime2 = 1.5;
        ElapsedTime waitTimer2 = new ElapsedTime();

        robot.initVuforia();
        robot.initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (robot.tfod != null) {
            robot.tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            robot.tfod.setZoom(1.5, 16.0/9.0);
        }

        // Object detection while waiting for start
        while(!isStarted()) {
            if (robot.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                        double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                        double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;
                        if (recognition.getLabel().equals("1 Bolt")) {
                            position = ScoringPosition.LEFT;
                        } else if (recognition.getLabel().equals("2 Bulb")) {
                            position = ScoringPosition.CENTER;
                        } else if (recognition.getLabel().equals("3 Panel")) {
                            position = ScoringPosition.RIGHT;
                        } else {
                            position = ScoringPosition.NONE;
                        }
                        telemetry.addData(""," ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                    }
                    telemetry.update();
                }
            }
        }
        /* might not work
        steps.add(new Straf)

         */

        // Turn off camera
        robot.tfod.deactivate();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.SETUP;
        set_claws();
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case SETUP:
                    if (claws==true) {
                        currentState = State.FORWARD;
                    }
                    break;
                case FORWARD:
                    encoderDrive(0, 0, 0.5);
                    if ((Math.abs(robot.leftFront.getCurrentPosition()) > distance_1) ||
                            (Math.abs(robot.rightBack.getCurrentPosition()) > distance_1)) {
                        stop_motors ();
                        currentState = State.TURRET_1;
                        rotation = 500;
                    }
                    break;
                case TURRET_1:
                    turn_turret (rotation);
                    if (robot.turret.getCurrentPosition()==500) {
                        currentState = State.LIFT_1;
                        target = 1050;
                    }
                    break;
                case LIFT_1:

                    if (arm_motor.getCurrentPosition()>=1000) {
                        currentState = State.WAIT_1;

                        waitTimer1.reset();
                    }
                    break;
                case WAIT_1:

                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.DEPOSIT;
                        score ();
                    }
                    break;
                case DEPOSIT:

                    if (scored) {
                        currentState = State.LIFT_2;
                        target = 10;
                    }
                    break;
                case LIFT_2:

                    if (arm_motor.getCurrentPosition()<=15) {
                        currentState = State.TURRET_2;
                        rotation = 0;
                        target = 0;
                    }
                    break;
                case TURRET_2:
                    turn_turret (rotation);

                    if (robot.turret.getCurrentPosition()==0) {
                        currentState = State.ALIGN;
                    }
                    break;
                case ALIGN:
                    encoderDrive(0, 0, 0.5);
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if ((Math.abs(robot.leftFront.getCurrentPosition()) > distance_2) ||
                            (Math.abs(robot.rightBack.getCurrentPosition()) > distance_2)) {
                        stop_motors ();
                        currentState = State.WAIT_2;
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.PARK;
                    }
                    break;
                case PARK:
                    if (position == ScoringPosition.LEFT) {
                        encoderDrive(0, -0.5, 0);
                    } else if (position == ScoringPosition.RIGHT) {
                        encoderDrive(0, -0.5, 0);
                    } else {

                    }
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if ((Math.abs(robot.leftFront.getCurrentPosition()) > distance_2) ||
                            (Math.abs(robot.rightBack.getCurrentPosition()) > distance_2)) {
                        stop_motors ();
                        currentState = State.IDLE;
                    }
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // We update our lift PID continuously in the background, regardless of state
            lift.update();
        }
    }

    public void encoderDrive ( double Angle, double x, double y){
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            double angle = angles.firstAngle;
            double r = Math.hypot(x, y);
            double robotAngle = Math.atan2(y, x) - Math.PI / 4;
            robotAngle = robotAngle - Math.toRadians(angle);
            double rightX = Angle;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            robot.leftFront.setPower(v1);
            robot.rightFront.setPower(v2);
            robot.leftBack.setPower(v3);
            robot.rightBack.setPower(v4);

            telemetry.addData("angle", " %7f", rightX);
            telemetry.addData("Currently at", " at %d :%d",
                    robot.leftFront.getCurrentPosition(), robot.rightBack.getCurrentPosition());
            telemetry.update();
        }
    }

    public void set_claws () {
        robot.liftVerticle.setPosition(0.4);
        robot.grabVerticle.setPosition(0.5);
        sleep(1000);
        claws = true;
    }
    public void stop_motors () {
        // Stop all motion;
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn off RUN_TO_POSITION
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);   // optional pause after each move.
    }
    public void score () {
        robot.liftVerticle.setPosition(0.0);
        sleep(1000);
        robot.grabVerticle.setPosition(0.35);
        sleep(1000);
        robot.liftVerticle.setPosition(1.0);
        sleep(1000);
        scored = true;
    }
    public void turn_turret (int rotation) {
        if(robot.turret.getCurrentPosition()>rotation){
            robot.turret.setTargetPosition(rotation);
            robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.turret.setPower(-0.3);
        } else {
            robot.turret.setTargetPosition(rotation);
            robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.turret.setPower(0.3);
        }
    }
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
    }

}
