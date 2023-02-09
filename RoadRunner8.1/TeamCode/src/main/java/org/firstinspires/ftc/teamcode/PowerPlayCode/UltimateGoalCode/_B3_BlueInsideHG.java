package org.firstinspires.ftc.teamcode.UltimateGoalCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


//@Autonomous(name="(B3) Blue Inside HG", group="LinearOpMode")

public class _B3_BlueInsideHG extends LinearOpMode {

    private ___RobotHardware___ rampage = new ___RobotHardware___();

    @Override

    public void runOpMode() {

        rampage.init(hardwareMap);
        rampage.AutonomousDrivingPower = .3;
        waitForStart();
        telemetry.addData("Status","Initialized");

        rampage.ResetEncoder();
        sleep(100);
        // project peacock
        sleep(0);
        // project peacock
        rampage.PrecisionDriveForwardReverse(90);
        sleep(700);
        rampage.PrecisionDriveForwardReverse(-26);
        sleep(700);
        rampage.PrecisionStrafe(-8);
        rampage.StartVulcan();
        sleep(700);
        rampage.FireDisk();
        sleep(400);
        rampage.ResetTrigger();
        sleep(700);
        rampage.FireDisk();
        sleep(400);
        rampage.ResetTrigger();
        sleep(700);
        rampage.FireDisk();
        sleep(400);
        rampage.ResetTrigger();
        sleep(700);
        rampage.StopVulcan();
        rampage.PrecisionDriveForwardReverse(13);
        rampage = null;
    }
}
