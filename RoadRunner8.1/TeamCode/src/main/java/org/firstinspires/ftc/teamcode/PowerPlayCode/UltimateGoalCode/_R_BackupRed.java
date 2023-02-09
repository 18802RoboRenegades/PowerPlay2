package org.firstinspires.ftc.teamcode.UltimateGoalCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


//@Autonomous(name="(R) BackupRed", group="LinearOpMode")

public class _R_BackupRed extends LinearOpMode {

    private ___RobotHardware___ rampage = new ___RobotHardware___();

    @Override

    public void runOpMode() {

        if(rampage == null)
            rampage = new ___RobotHardware___();

        rampage.init(hardwareMap);
        rampage.AutonomousDrivingPower = .3;
        rampage.AutoVulcanPower = .65;
        waitForStart();
        telemetry.addData("Status","Initialized");

        rampage.ResetEncoder();
        sleep(100);
        // project peacock
        sleep(0);
        // project peacock
        rampage.MoveWobble(30);
        sleep(400);
        rampage.CloseWobbleHand();
        rampage.PrecisionDriveForwardReverse(71);


        rampage = null;
    }
}