package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "Autonomous1", group = "boundless")
@Disabled
public class Autonomous1 extends LinearOpMode {

    private DcMotor spool = null;       // Motor to control the up/down spool
    private DcMotor frontLeft = null;   // Motors to control the drive train
    private DcMotor frontRight = null;
    private DcMotor claw = null;   // Motor to control the horizontal motion

    BNO055IMU imu;
    Orientation angles;
    private double start_heading;       // The gyro reading at start

    @Override
    public void runOpMode() {
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
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // what's the start heading?
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        start_heading = angles.firstAngle;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        spool = hardwareMap.get(DcMotor.class, "spool");
        claw = hardwareMap.get(DcMotor.class, "claw");

        // set left motor to run to target encoder position and stop with brakes on.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setTargetPosition(0);
        frontLeft.setPower(0);

        // set right motor to reverse and run to target encoder position and stop with brakes on
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setTargetPosition(0);
        frontRight.setPower(0);

        // set spool to run to target encoder position and stop with brakes on.
        spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spool.setTargetPosition(0);
        spool.setPower(0);


        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Step 0: Open the claw and keep it ready
            claw.setPower(0.3);
            try { Thread.sleep(200); } catch(Exception ex) {}
            claw.setPower(0);

            // Step 1: Set the power for the left & right motors;
            frontLeft.setPower(0.02);
            frontRight.setPower(0.02);
            try { Thread.sleep(1000); } catch (InterruptedException ex) { break; };
            frontLeft.setPower(0);
            frontRight.setPower(0);

            // Step 2: Lift the claw
            spool.setPower(-0.3);
            try { Thread.sleep(500); } catch (InterruptedException ex) { break; };
            spool.setPower(0);

            // Step 3: Close the claw and grasp the block
            claw.setPower(0.3);
            try { Thread.sleep(500); } catch(Exception ex) {}
            claw.setPower(0);

            break;
        }

    }

    public void gyroEncoderTurn(double targetAngle) {
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
            if (error == 0) { break; }
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
}
