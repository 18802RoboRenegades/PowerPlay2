package org.firstinspires.ftc.teamcode.UltimateGoalCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name="TestOpMode", group="LinearOpMode")
public class TestOpMode extends LinearOpMode
{
    private DcMotor leftFront, rightFront, leftRear, rightRear;

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

        double runtime = 0;
        String operation = "";


        while (opModeIsActive())
        {
            runtime = getRuntime();

            drive = 0;
            turn = 0;
            strafe = 0;

            // DO NOT DELETE THE COMMENTED CODE!!!!!
            if(runtime <= 5)
            {
                drive = -.5;
                operation = "Move forward";
            }
            else if(runtime > 5 && runtime<=10)
            {
                drive = .5;
                operation = "Move backward";
            }
            else if(runtime >10 && runtime <=15)
            {
                turn = .5;
                operation = "Rotate right";
            }
            else if(runtime>15 && runtime<=20)
            {
                turn = -.5;
                operation = "Rotate left";
            }
            else if (runtime > 20 && runtime <= 22)
            {
                strafe = .5;
                operation = "Strafe right";
            }
            else if (runtime >22 && runtime <=24)
            {
                strafe = -.5;
                operation = "Strafe left";
            }
            else if (runtime > 24 && runtime <= 29)
            {
            //    rampage.LoadMagazine();
                operation = "Loading Mag";
            }
            else if(runtime > 29 && runtime <= 34)
            {
            //    rampage.UnLoadMag();
                operation = "Unloading";
            }
            else if (runtime > 34 && runtime <= 39)
            {
            //    rampage.StopMag();
                if(!rampage.isVulcanOn)
                {
                 //   rampage.ToggleVulcan();
                    operation = "Spinning up wheels";
                }
            }
            else if (runtime > 39 && runtime <= 40)
            {
                if(rampage.isVulcanOn)
                {
                   // rampage.ToggleVulcan();
                }
            }
            else if (runtime > 40 && runtime <= 42)
            {
                rampage.MoveWobble(.25f);
                operation = "Moving Wobble Arm";
            }
            else if (runtime > 42 && runtime <= 44)
            {
                rampage.MoveWobble(0);
                if(!rampage.isHandOpen)
                {
                //    rampage.ToggleWobbleHand();
                    operation = "Opening Hand";
                }
            }
            else if (runtime > 44 && runtime <= 46)
            {
                if (rampage.isHandOpen)
                {
                //    rampage.ToggleWobbleHand();
                    operation = "Closing Hand";
                }

            }
            else if (runtime > 46 && runtime <= 48)
            {
                rampage.MoveWobble(-.25f);
                operation = "Moving Wobble Arm";
            }
            else if (runtime > 48 && runtime <= 49)
            {
                rampage.MoveWobble(0);
            }
            rampage.Drive(drive, turn, strafe);

            // telemetry.addData("Target Power", drive);
            // telemetry.addData("Motor Power", leftFront.getPower());
            telemetry.addData("Status", "Running");
            telemetry.addData("Operation", operation);
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Strafe Power", strafePower);
            telemetry.update();

        }
    }
}