package org.firstinspires.ftc.teamcode.UltimateGoalCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name="HunterOpMode", group="LinearOpMode")

public class __HunterOpMode__ extends LinearOpMode
{

    private DistanceSensor sensorColorRange;
    private Servo servoTest;
    // The line of code below is defining the robot.
    private ___RobotHardware___ rampage = new ___RobotHardware___();


    @Override
    public void runOpMode()
    {
        rampage.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double drive = 0;
        double turn = 0;
        double leftPower = 0;
        double rightPower = 0;
        double strafe = 0;
        double strafePower = 0;
        double [] shooterRPM = {0,0};


        double runtime = 0;
        while (opModeIsActive()) {
            drive = this.gamepad1.left_stick_y;
            turn = this.gamepad1.right_stick_x;
            strafe = this.gamepad1.left_stick_x;


            if (this.gamepad2.right_bumper)
            {
                rampage.StartVulcan();
            }
            else if (this.gamepad2.left_bumper)
            {
                rampage.PowerShotStartVulcan();
            }
            else if (rampage.isVulcanOn)
            {
                rampage.StopVulcan();
            }
            if (this.gamepad2.right_trigger > 0)
            {
                rampage.FireDisk();
            }
            else
            {
                rampage.ResetTrigger();
            }
            if (this.gamepad2.a)
            {
                rampage.LoadMagazine();
                rampage.PropDisk();
            }
            if (this.gamepad2.b)
            {
                rampage.StopMag();
                rampage.StopProp();
            }
            if (this.gamepad2.dpad_up)
            {
                rampage.OpenWobbleHand();
            }
            if (this.gamepad2.dpad_down)
            {
                rampage.CloseWobbleHand();
            }
            if (this.gamepad2.x)
            {
                rampage.ReverseProp();
                rampage.UnLoadMag();
            }
            if (this.gamepad1.right_bumper)
            {
                rampage.DropFeedGate();
            }
            if (this.gamepad2.left_trigger > 0)
            {
                rampage.FireDisk();
                sleep(150);
                rampage.ResetTrigger();
                sleep(150);
                rampage.FireDisk();
                sleep(150);
                rampage.ResetTrigger();
                sleep(150);
                rampage.FireDisk();
                sleep(150);
                rampage.ResetTrigger();
            }
            rampage.MoveWobble(gamepad2.left_stick_y);

            rampage.Drive(drive, turn, strafe);
            shooterRPM = rampage.GetShooterRPM();

            telemetry.addData("Status", "Running");
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Strafe Power", strafePower);
            telemetry.addData("Shooter 1 RPM" , shooterRPM[0]);
            telemetry.addData("Shooter 2 RPM", shooterRPM[1]);
            telemetry.update();

        }
    }
}
