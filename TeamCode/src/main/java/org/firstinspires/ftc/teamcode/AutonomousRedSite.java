package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "AutonomousRedSite", group = "boundless")
public class AutonomousRedSite extends LinearOpMode {
    // Initiate all the motors. We have 5 motors

    // The spool controls the up/down motion
    private DcMotor spool = null;

    // The frontLeft and frontRight controls the drive train
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;

    // The leftRight controls the horizontal motion
    private DcMotor leftRight = null;

    // The frontBack controls the front-back claw
    private DcMotor frontBack = null;

    BNO055IMU imu;
    Orientation angles;
    private double start_heading;

    private ElapsedTime runtime = new ElapsedTime();

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU", and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // what's the start heading?
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        start_heading = angles.firstAngle;

        // This connects the software to the hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        spool = hardwareMap.get(DcMotor.class, "spool");
        leftRight = hardwareMap.get(DcMotor.class, "leftRight");
        frontBack = hardwareMap.get(DcMotor.class, "frontBack");

        // set left motor to run to target encoder position and stop with brakes on.
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set right motor to reverse and run to target encoder position and stop with brakes on
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // This always updates telemetry
        telemetry.addData("Status", "Boundless Robotics Ready to Launch");
        telemetry.update();
        waitForStart();

        driveUsingEncoder(DRIVE_SPEED,  12,  12, 15.0);
        driveUsingEncoder(DRIVE_SPEED,  -5,  5, 15.0);
    }

    public void turnUsingEncoder(double targetAngle) {
        double error;
        int newLeftTarget;
        int newRightTarget;
        double rotPow;
        double speedScale = 0.5;

        for (int idx = 0; idx < 15; idx++) {

            // Determine how much our heading is off.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = targetAngle - angles.firstAngle;

            // Get out of loop early if there is no error.
            if (error == 0) {
                break;
            }
            // Determine new target position.
            newLeftTarget = frontLeft.getCurrentPosition() + (int) (error / 360 * 4000);
            newRightTarget = frontRight.getCurrentPosition() - (int) (error / 360 * 4000);

            //new target calculated
            frontLeft.setTargetPosition(newLeftTarget);
            frontRight.setTargetPosition(newRightTarget);

            // Power will also be a function of error. Big error = fast, small error = slow
            if (Math.abs(error) > 50) {
                rotPow = 0.8 * speedScale;
            } else if (Math.abs(error) > 40) {
                rotPow = 0.6 * speedScale;
            } else if (Math.abs(error) > 30) {
                rotPow = 0.4 * speedScale;
            } else if (Math.abs(error) > 20) {
                rotPow = 0.3 * speedScale;
            } else if (Math.abs(error) > 10) {
                rotPow = 0.2 * speedScale;
            } else {
                rotPow = 0.1 * speedScale;
            }

            // Do the incremental turn.
            frontLeft.setPower(rotPow);
            frontRight.setPower(rotPow);

            while (frontLeft.isBusy() && frontRight.isBusy()) {
            }

            // Stop all motion. This also applies the brakes.
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

    }

    public void driveUsingEncoder(double speed, double leftInches, double rightInches, double timeoutSeconds) {

        final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (!opModeIsActive()) {
            return;
        }

        // Determine new target position, and pass to motor controller
        newLeftTarget = frontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = frontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        frontLeft.setTargetPosition(newLeftTarget);
        frontRight.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutSeconds) &&
                (frontLeft.isBusy() && frontRight.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);

        // Turn off RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
