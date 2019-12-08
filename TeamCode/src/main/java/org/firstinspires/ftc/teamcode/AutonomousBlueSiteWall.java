package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.Callable;


@Autonomous(name = "AutonomousBlueSiteWall", group = "boundless")
public class AutonomousBlueSiteWall extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BoundlessRobot robot = new BoundlessRobot(hardwareMap, telemetry);

        Callable<Boolean> opMode = new Callable<Boolean> () {
            public Boolean call() {
                return opModeIsActive();
            }
        };

        // This always updates telemetry
        telemetry.addData("Status", "Boundless Robotics Ready to Launch");
        telemetry.update();

        waitForStart();

        robot.strafeLeft(telemetry, opMode, 0.4, 11, 11);

        robot.spoolUp(0.5, 1);
        try { Thread.sleep(100); } catch(Exception ex) {}

        robot.driveUsingEncoder(telemetry,opMode,0.4,-35,-35, 10);
        try { Thread.sleep(100); } catch(Exception ex) {}

        robot.spoolDown(0.5, 1);
        try { Thread.sleep(100); } catch(Exception ex) {}

        robot.driveUsingEncoder(telemetry,opMode,0.4,43,43, 10);
        try { Thread.sleep(100); } catch(Exception ex) {}

        robot.spoolUp(0.5, 1);

        robot.strafeRight(telemetry, opMode, 0.3, 40, 10);
        robot.driveUsingEncoder(telemetry,opMode,0.4,-2,-2, 5);
        robot.driveUsingEncoder(telemetry,opMode,0.4,5,5, 5);

        robot.spoolDown(0.5, 0.7);
        robot.strafeRight(telemetry, opMode, 0.3, 25, 20);

    }
}
