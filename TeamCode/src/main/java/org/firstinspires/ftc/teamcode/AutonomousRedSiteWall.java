package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.Callable;


@Autonomous(name = "AutonomousRedSiteWall", group = "boundless")
public class AutonomousRedSiteWall extends LinearOpMode {
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

        robot.strafeRight(telemetry, opMode, 0.4, 13, 13);

        robot.driveUsingEncoder(telemetry,opMode,0.4,4,4, 10);
        robot.spoolUp(0.5, 1);
        try { Thread.sleep(100); } catch(Exception ex) {}

        robot.driveUsingEncoder(telemetry,opMode,0.4,-35,-35, 10);
        try { Thread.sleep(100); } catch(Exception ex) {}

        robot.spoolDown(0.5, 1);
        try { Thread.sleep(100); } catch(Exception ex) {}

        robot.driveUsingEncoder(telemetry,opMode,0.4,43,43, 10);
        try { Thread.sleep(100); } catch(Exception ex) {}

        robot.spoolUp(0.5, 1);

        robot.strafeLeft(telemetry, opMode, 0.3, 40, 10);

        robot.spoolDown(0.5, 0.7);
        robot.strafeLeft(telemetry, opMode, 0.3, 10, 10);


    }
}
