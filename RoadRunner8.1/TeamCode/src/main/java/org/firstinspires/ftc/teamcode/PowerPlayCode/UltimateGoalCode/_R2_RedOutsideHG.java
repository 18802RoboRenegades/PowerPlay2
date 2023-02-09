package org.firstinspires.ftc.teamcode.UltimateGoalCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


//@Autonomous(name="(R2) Red Outside HG", group="LinearOpMode")

public class _R2_RedOutsideHG extends LinearOpMode {

    private ___RobotHardware___ rampage = new ___RobotHardware___();

    @Override

    public void runOpMode() {

        if(rampage == null)
            rampage = new ___RobotHardware___();

        rampage.init(hardwareMap);
        rampage.AutonomousDrivingPower = .15;
        double [] shooterRPM = {0,0};
        //shooterRPM = rampage.GetShooterRPM();
        waitForStart();
        telemetry.addData("Status","Initialized");

        rampage.ResetEncoder();
        sleep(100);
        // project peacock
        sleep(7000);
        // project peacock
        rampage.PrecisionDriveForwardReverse(59);
        sleep(700);
        rampage.AutonomousDrivingPower = .3;
        sleep(700);
        rampage.PrecisionStrafe(-9);
        rampage.StartVulcan();
        shooterRPM = rampage.GetShooterRPM();
        telemetry.addData("Shooter 1 RPM" , shooterRPM[0]);
        telemetry.addData("Shooter 2 RPM", shooterRPM[1]);
        telemetry.update();
        rampage.PrecisionDriveForwardReverse(3);
        shooterRPM = rampage.GetShooterRPM();
        telemetry.addData("Shooter 1 RPM" , shooterRPM[0]);
        telemetry.addData("Shooter 2 RPM", shooterRPM[1]);
        telemetry.update();
        sleep(1000);
        shooterRPM = rampage.GetShooterRPM();
        telemetry.addData("Shooter 1 RPM" , shooterRPM[0]);
        telemetry.addData("Shooter 2 RPM", shooterRPM[1]);
        telemetry.update();
        rampage.FireDisk();
        sleep(400);
        shooterRPM = rampage.GetShooterRPM();
        telemetry.addData("Shooter 1 RPM" , shooterRPM[0]);
        telemetry.addData("Shooter 2 RPM", shooterRPM[1]);
        telemetry.update();
        rampage.ResetTrigger();
        sleep(400);
        shooterRPM = rampage.GetShooterRPM();
        telemetry.addData("Shooter 1 RPM" , shooterRPM[0]);
        telemetry.addData("Shooter 2 RPM", shooterRPM[1]);
        telemetry.update();
        rampage.FireDisk();
        sleep(400);
        shooterRPM = rampage.GetShooterRPM();
        telemetry.addData("Shooter 1 RPM" , shooterRPM[0]);
        telemetry.addData("Shooter 2 RPM", shooterRPM[1]);
        telemetry.update();
        rampage.ResetTrigger();
        sleep(400);
        shooterRPM = rampage.GetShooterRPM();
        telemetry.addData("Shooter 1 RPM" , shooterRPM[0]);
        telemetry.addData("Shooter 2 RPM", shooterRPM[1]);
        telemetry.update();
        rampage.FireDisk();
        sleep(400);
        shooterRPM = rampage.GetShooterRPM();
        telemetry.addData("Shooter 1 RPM" , shooterRPM[0]);
        telemetry.addData("Shooter 2 RPM", shooterRPM[1]);
        telemetry.update();
        rampage.ResetTrigger();
        rampage.StopVulcan();
        rampage.PrecisionDriveForwardReverse(11);
        rampage = null;
    }
}