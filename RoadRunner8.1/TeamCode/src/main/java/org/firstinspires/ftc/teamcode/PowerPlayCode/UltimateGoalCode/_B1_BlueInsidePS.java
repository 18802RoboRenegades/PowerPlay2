package org.firstinspires.ftc.teamcode.UltimateGoalCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


//@Autonomous(name="(B1) Blue Inside PS", group="LinearOpMode")

public class _B1_BlueInsidePS extends LinearOpMode {

    private ___RobotHardware___ rampage = new ___RobotHardware___();

    @Override

    public void runOpMode() {

        if(rampage == null)
            rampage = new ___RobotHardware___();

        rampage.init(hardwareMap);
        rampage.AutonomousDrivingPower = .3;
        waitForStart();
        telemetry.addData("Status","Initialized");

        rampage.ResetEncoder();
        sleep(100);
        // project peacock
        sleep(0);
        // project peacock
        rampage.PrecisionDriveForwardReverse(86);
        sleep(700);
        rampage.PrecisionDriveForwardReverse(-23);
        sleep(700);
        rampage.PowerShotStartVulcan();
        rampage.PrecisionStrafe(0);
        sleep(700);
        telemetry.addData("shooterRPM" , rampage.GetShooterRPM()[0]);
        telemetry.update();
        rampage.FireDisk();
        sleep(400);
        rampage.AutoVulcanPower = .55;
        rampage.ResetTrigger();
        rampage.PrecisionStrafe(5);
        sleep(750);
        telemetry.addData("shooterRPM" , rampage.GetShooterRPM()[0]);
        telemetry.update();
        rampage.FireDisk();
        sleep(400);
        rampage.ResetTrigger();
        rampage.PrecisionStrafe(4);
        rampage.AutoVulcanPower = .61;
        sleep(750);
        telemetry.addData("shooterRPM" , rampage.GetShooterRPM()[0]);
        telemetry.update();
        rampage.FireDisk();
        sleep(400);
        rampage.ResetTrigger();
        rampage.StopVulcan();
        //sleep(500);
        //rampage.PrecisionDriveForwardReverse(20);
        rampage.AutonomousDrivingPower = .35;
        rampage.PrecisionDriveForwardReverse(10);

        rampage = null;
    }
}

