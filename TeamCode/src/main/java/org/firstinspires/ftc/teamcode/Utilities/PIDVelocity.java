package org.firstinspires.ftc.teamcode.Utilities;

/*public class PIDVelocity extends LinearOpMode {
    private DcMotorEx shooterMotor;

    static double speed = 1200;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0,0 );
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0,0 );

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpmode() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "spinner");
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if(opModeIsActive()) {
            while (opModeIsActive()) {

                PID(speed);

                telemetry.addData("target", speed);
                telemetry.addData("target", shooterMotor.getVelocity());
                telemetry.update();
            }
        }
    }

    double integral = 0;
    double lastError;

    public void PID(double targetVelocity){

        PIDTimer.reset();
        double currentVelocity = shooterMotor.getVelocity();

        double error = targetVelocity - currentVelocity;

        integral += error * PIDTimer.time();

        double deltaError = error - lastError;
        double derivative = deltaError/PIDTimer.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integral;
        pidGains.d = pidCoeffs.d * derivative;

        shooterMotor.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);

        lastError = error;
    }
}*/
