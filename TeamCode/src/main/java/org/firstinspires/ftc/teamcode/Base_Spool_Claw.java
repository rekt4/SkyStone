package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Base_Spool_Claw", group="boundless")

public class Base_Spool_Claw extends LinearOpMode {

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

    public void runOpMode(){

        // This connects the software to the hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        spool = hardwareMap.get(DcMotor.class, "spool");
        leftRight = hardwareMap.get(DcMotor.class, "leftRight");
        frontBack = hardwareMap.get(DcMotor.class, "frontBack");


        // This always updates telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // This is sets the original joystick power
        double spoolLeftStick=0;
        double leftRightReqPower=0;
        double fbLeftStick=0;
        double fbRightStick=0;
        double frontBackRp=0;
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        while(opModeIsActive()){
            // Using methods created, the spool and the z-movement claw part will be 'activated'

            //what are we doing with the spool motor?

            spoolLeftStick = -this.gamepad2.left_stick_y;
            setPowerToSpool(spoolLeftStick);

            //what are we doing with the front back motion?

            frontBackRp = this.gamepad2.right_stick_y;
            frontBack.setPower(-1*frontBackRp/4);

            // What are we doing with the left right motion?
            if (this.gamepad2.left_bumper) {
                leftRight.setPower(0.5);
            } else if (this.gamepad2.right_bumper){
                leftRight.setPower(-0.5);
            } else {
                leftRight.setPower(0);
            }

            // We first need to stop the claw servo from moving if it is moving
            //claw.setPosition(0.5);

            // Using other methods, we also 'activate' the actual claw mechanism
            //stopClaw();
            //openClaw();
            //closeClaw();

            // Finally the tank is also 'activated'
            fbLeftStick = -this.gamepad1.left_stick_y;
            fbRightStick = -this.gamepad1.right_stick_y;
            tank(fbLeftStick, fbRightStick);

            // Code below helps for debugging
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }


    public void tank(double fbLeftStick, double fbRightStick){
        frontLeft.setPower(fbLeftStick);
        frontRight.setPower(fbRightStick);
        telemetry.addData("Motor Power - Left", frontLeft.getPower());
        telemetry.addData("Motor Power - Right", frontRight.getPower());

    }

    public void setPowerToSpool(double Stick){
        spool.setPower(Stick);
        telemetry.addData("Up/Down Power", Stick);

    }

//    public void openClaw() {
//        if (gamepad2.y) {
//            clawPosition = 0.9;
//        }
//        claw.setPosition(clawPosition);
//        telemetry.addData("Claw Power", clawPosition);
//    }

//    public void closeClaw() {
//        if (gamepad2.a) {
//            clawPosition = 0.1;
//        }
//        claw.setPosition(clawPosition);
//        telemetry.addData("Claw Power", clawPosition);
//    }

//    public void stopClaw() {
//        if(gamepad2.b) {
//            clawPosition = 0.5;
//        }
//        claw.setPosition(clawPosition);
//        telemetry.addData("Claw Power", clawPosition);
//    }


//    public void setPowerToLeftRight(double power) {
//        leftRight.setPower(-power);
//        telemetry.addData("Left/Right Power", power);
//    }
}