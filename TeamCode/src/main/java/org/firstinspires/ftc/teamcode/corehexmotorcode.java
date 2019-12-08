package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="corehexmotorcode", group="boundless")
@Disabled
public class corehexmotorcode extends LinearOpMode {

    // Initiate all the motors. We have 2 motors.

    // The frontLeft and frontRight controls the drive train
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;

    public void runOpMode(){

        // This connects the software to the hardware
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        // This always updates telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // This is sets the original joystick power
        double motorRequestedPower =0;
        motor2.setDirection(DcMotor.Direction.REVERSE);

        while(opModeIsActive()){
            // Using methods created, the spool and the z-movement claw part will be 'activated'

            //what are we doing with the spool motor?

            motorRequestedPower = -this.gamepad2.left_stick_y;
            motor1.setPower(motorRequestedPower);
            motor2.setPower(motorRequestedPower);

            // Code below helps for debugging
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

}
