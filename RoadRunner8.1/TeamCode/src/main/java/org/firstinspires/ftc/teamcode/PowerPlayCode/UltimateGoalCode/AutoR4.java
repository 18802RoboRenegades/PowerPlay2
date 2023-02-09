package org.firstinspires.ftc.teamcode.UltimateGoalCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


//@Autonomous(name="AutoR4", group="LinearOpMode")

public class AutoR4 extends LinearOpMode {

    private ___RobotHardware___ rampage = new ___RobotHardware___();

    @Override

    public void runOpMode() {

        if (rampage == null)
            rampage = new ___RobotHardware___();

        rampage.init(hardwareMap);
        rampage.AutonomousDrivingPower = .3;
        rampage.AutoVulcanPower = .61;
        waitForStart();
        telemetry.addData("Status", "Initialized");

        rampage.ResetEncoder();
        sleep(100);
        rampage.PrecisionDriveForwardReverse(2);
        sleep(200);
        rampage.PrecisionStrafe(-11);
        sleep(2000);
        rampage.PrecisionDriveForwardReverse(60);
        sleep(700);
        rampage.AutoStartVulcan();
        sleep(500);
        rampage.FireDisk();
        sleep(400);
        rampage.ResetTrigger();
        rampage.PrecisionStrafe(3);
        sleep(300);
        rampage.FireDisk();
        sleep(400);
        rampage.ResetTrigger();
        rampage.PrecisionStrafe(6);
        sleep(300);
        rampage.FireDisk();
        sleep(400);
        rampage.ResetTrigger();
        rampage.StopVulcan();
        rampage.PrecisionDriveForwardReverse(10);
        sleep(500);
        rampage.PrecisionStrafe(-10);
    }}