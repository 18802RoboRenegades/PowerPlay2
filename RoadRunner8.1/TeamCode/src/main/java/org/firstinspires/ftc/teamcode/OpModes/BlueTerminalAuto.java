package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;

import java.util.List;
import java.util.Objects;

@Autonomous(name = "AutoBlueTerminal", group = "Competition")

public class BlueTerminalAuto extends LinearOpMode {

    private final static HWProfile2 robot = new HWProfile2();
    private LinearOpMode opMode = this;

    FtcDashboard dashboard;

    private static final String TFOD_MODEL_ASSET = "PP_Generic_SS.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "circle",
            "star",
            "triangle"
    };

    private static final String VUFORIA_KEY =
        "ARLYRsf/////AAABmWpsWSsfQU1zkK0B5+iOOr0tULkAWVuhNuM3EbMfgb1+zbcOEG8fRRe3G+iLqL1/iAlTYqqoLetWeulG8hkCOOtkMyHwjS/Ir8/2vUVgC36M/wb9a7Ni2zuSrlEanb9jPVsNqq+71/uzTpS3TNvJI8WeICQNPAq3qMwmfqnCphVlC6h2ZSLsAR3wcdzknFmtpApdOp1jHJvITPeD/CMdAXjZDN0XJwJNQJ6qtaYSLGC23vJdQ2b1aeqnJauOvswapsG7BlmR7m891VN92rNEcOX7WmMT4L0JOM0yKKhPfF/aSROwIdNtSOpQW4qEKVjw3aMU1QDZ0jj5SnRV8RPO0hGiHtXy6QJcZsSj/Y6q5nyf";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    /* Declare OpMode members. */
    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);
    int position = 3;

    public BlueTerminalAuto(){

    }

//    @Override
    public void runOpMode() {
//        State runState = State.HIGH_JUNCTION_1;
        State runState = State.HIGH_JUNCTION_1;

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        robot.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.

         */

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.motorBase.setTargetPosition(0);
        robot.motorBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //

        while(!isStarted() && !isStopRequested()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                        double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                        double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                        telemetry.addData(""," ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        dashTelemetry.put("p01 - Image", recognition.getLabel());
                        dashTelemetry.put("p02 - confidence level", recognition.getConfidence() * 100 );

                        if(Objects.equals(recognition.getLabel(), "circle")){
                            position =1;
                        } else if(Objects.equals(recognition.getLabel(), "triangle")){
                            position = 2;
                        } else position = 3;
                    }
                telemetry.update();
                    // post telemetry to FTC Dashboard as well
                    dashTelemetry.put("p03 - PID IMU Angle X                  = ", robot.imu.getAngles()[0]);
                    dashTelemetry.put("p04 - PID IMU Angle Y                  = ", robot.imu.getAngles()[1]);
                    dashTelemetry.put("p05 - PID IMU Angle Z                  = ", robot.imu.getAngles()[2]);
                    dashTelemetry.put("p06 - Lift Front Encoder Value = ", robot.motorBase.getCurrentPosition());
                    dashboard.sendTelemetryPacket(dashTelemetry);
                }
            }
        }  // end of while

        if(isStopRequested()) requestOpModeStop();   // user requested to abort setup

        while (opModeIsActive()) {
            switch(runState){
                case TEST:

                    /*
                    robot.motorLF.set(1);
                    telemetry.addData("motorLF ", "On");
                    telemetry.addData("encoder value ", robot.motorLF.getCurrentPosition());
                    telemetry.update();
                    sleep(2000);
                    robot.motorLF.stopMotor();

                    robot.motorLR.set(1);
                    telemetry.addData("motorLR ", "On");
                    telemetry.addData("encoder value ", robot.motorLR.getCurrentPosition());
                    telemetry.update();
                    sleep(2000);
                    robot.motorLR.stopMotor();

                    robot.motorRF.set(1);
                    telemetry.addData("motorRF ", "On");
                    telemetry.addData("encoder value ", robot.motorRF.getCurrentPosition());
                    telemetry.update();
                    sleep(2000);
                    robot.motorRF.stopMotor();

                    robot.motorRR.set(1);
                    telemetry.addData("motorRR ", "On");
                    telemetry.addData("encoder value ", robot.motorRR.getCurrentPosition());
                    telemetry.update();
                    sleep(2000);
                    robot.motorRR.stopMotor();

                     */

                    drive.driveDistance(90, 48);
                    sleep(5000);

                    drive.driveDistance(-90, 48);
                    sleep(5000);


                    /*
                    drive.pidRotate(45, 1);
                    sleep(2000);

                    drive.pidRotate(0, 1);
                    sleep(2000);

                    drive.pidRotate(-45, 1);
                    sleep(2000);

                    drive.pidRotate(0, 1);
                    sleep(2000);

                    drive.pidRotate(90, 1);
                    sleep(2000);

                    drive.pidRotate(0, 1);
                    sleep(2000);

                    drive.pidRotate(-90, 1);
                    sleep(2000);

                    drive.pidRotate(0, 1);
                    sleep(2000);

                    drive.pidRotate(135, 1);
                    sleep(2000);

                    drive.pidRotate(0, 1);
                    sleep(2000);

                    drive.pidRotate(-135, 1);
                    sleep(2000);

                    drive.pidRotate(135, 1);
                    sleep(2000);

                    drive.pidRotate(0, 1);
                    sleep(2000);

                     */

                    runState = State.HALT;
                    break;

                case LEVEL_ADJUST:
                    runState = State.HIGH_JUNCTION_1;
                    break;

                case HIGH_JUNCTION_1:
                    // starting from start position, close claw
                    drive.closeClaw();
                    sleep(500);

                    // raise the arm to position the cone

                    // Drive forward away from wall, pushing signal cone out of position
                    drive.driveDistance(0, 2);

                    //strafe to zonr 
                    drive.driveDistance(-90, 24);
                    sleep(500);
                    drive.pidRotate(0, 1);
                    // drive forward to place the cone
                    drive.driveDistance(0,50);
                    drive.pidRotate(0, 1);
                    sleep(500);
                    drive.liftHighJunction();
                    sleep(2000);
                    drive.driveDistance(90, 20);
                    drive.pidRotate(0, 1);

                    // lower the arm and release the cone
                    drive.liftMidJunction();
                    sleep(300);
                    drive.openClaw();

                    // raise the lift to keep from entangling on junction
                    drive.liftHighJunction();
                    sleep(500);
                    // back away from the junction

                    runState = State.PARK;
                    break;

                case CONE_2:        // top cone of the starter stack
                    //rotate towards the cone stack
                    drive.pidRotate(90, robot.PID_ROTATE_ERROR);

                    // lower the arm to pick up the top cone
                    drive.liftPosition(robot.LIFT_CONE5);

                    //drive towards the stack of cones
                    drive.driveDistance(0,16);

                    // adjust direction - turn towards cone stack
                    drive.pidRotate(90, robot.PID_ROTATE_ERROR);

                    //drive towards the stack of cones
                    drive.driveDistance(0,12);

                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(300);

                    //back away from the wall slightly
                    drive.driveDistance(180,1);

                    // lift the cone up to clear the stack
                    drive.liftPosition(robot.LIFT_EXTRACT_CONE);
                    sleep(300);

                    runState = State.LOW_JUNCTION_2;
                    break;

                case LOW_JUNCTION_2:    // low junction 1st pass
                    // back away to tile 2
                    drive.driveDistance(180,22);

                    // lift the rest of the way to low junction
                    drive.liftPosition(robot.LIFT_LOW_JUNCTION);

                    // rotate towards the low junction
                    drive.pidRotate(135, robot.PID_ROTATE_ERROR);

                    // drive towards the junction to place the cone
                    drive.driveDistance(0, 8);

                    // place the cone
                    drive.liftPosition(robot.LIFT_RESET);
                    sleep(200);
                    drive.openClaw();

                    // raise the lift to clear the junction
                    drive.liftLowJunction();
                    sleep(200);

                    // back away from the junction
                    drive.driveDistance(180, 8);

                    // turn towards the starter stack
                    drive.pidRotate(90, robot.PID_ROTATE_ERROR);

                    runState = State.CONE_3;
                    break;

                case CONE_3:        // 4th cone up from starter stack

                    // lower the arm to pick up the top cone
                    drive.liftPosition(robot.LIFT_CONE4);

                    //drive towards the stack of cones
                    drive.driveDistance(0,15);

                    // adjust direction - turn towards cone stack
                    drive.pidRotate(90, robot.PID_ROTATE_ERROR);

                    //drive towards the stack of cones
                    drive.driveDistance(0,11);

                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(400);

                    //back away from the wall slightly
                    drive.driveDistance(180,1);

                    // lift the cone up to clear the stack
                    drive.liftPosition(robot.LIFT_EXTRACT_CONE);
                    sleep(400);

                    runState = State.LOW_JUNCTION_3;
                    break;

                case LOW_JUNCTION_3:
                    // back away to tile 2
                    drive.driveDistance(180,22);

                    // lift the rest of the way to low junction
                    drive.liftPosition(robot.LIFT_LOW_JUNCTION);

                    // rotate towards the low junction
                    drive.pidRotate(135, robot.PID_ROTATE_ERROR);

                    // decide if the program should use the distance sensors to find the junction

                    // place the cone
                    drive.liftReset();
                    sleep(200);
                    drive.openClaw();

                    // raise the lift to clear the junction
                    drive.liftLowJunction();

                    //back away from the junction
                    drive.driveDistance(180, 8);

                    // turn towards the stack
                    drive.pidRotate(90, robot.PID_ROTATE_ERROR);

                    runState = State.PARK;
                    break;

                case MID_JUNCTION_3:
                    // back away to tile 2
                    drive.driveDistance(180,50);

                    // raise the arm to position the cone
                    drive.liftMidJunction();

                    // rotate towards the mid junction
                    drive.pidRotate(135, robot.PID_ROTATE_ERROR);

                    // decide if the program should use the distance sensors to find the junction

                    // lower the arm and release the cone
                    drive.liftLowJunction();
                    sleep(400);
                    drive.openClaw();

                    // raise the lift to keep from entangling on junction
                    drive.liftMidJunction();

                    // back away from the junction
                    drive.driveDistance(180, 8);

                    //rotate towards the cone stack
                    drive.pidRotate(90, robot.PID_ROTATE_ERROR);

                    // reset the lift to its starting position
                    drive.liftReset();

                    runState = State.PARK;
                    break;

                case PARK:

                    if(position == 3) {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the stack to park - ready to grab the first cone in teleop
                        //drive.PIDRotate(-90, robot.PID_ROTATE_ERROR);

                        // drive to park position 1
                        drive.driveDistance(90,35);

                    } else if (position == 2) {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the outside wall position
                        //drive.PIDRotate(-90, robot.PID_ROTATE_ERROR);

                        // drive to park position 1
                        drive.driveDistance(90,10);

                    } else {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the outside wall position
                        //drive.PIDRotate(-90, robot.PID_ROTATE_ERROR);

                        // drive to park position 1
                        drive.driveDistance(-90,10);

                    }

                    while(opModeIsActive() && robot.motorBase.getCurrentPosition() > 10){
                        drive.liftReset();
                    }

                    runState = State.HALT;

                    break;


                case HALT:

                    // shut down all motors
                    drive.motorsHalt();

                    requestOpModeStop();    // request stoppage of the program

                    break;
            }   // end of switch(state)
        }   // end of while(opModeIsActive)

        requestOpModeStop();

        telemetry.addData("Path", "Complete");
        telemetry.update();

    } // end of opmode

    /*
     * Enumerate the states of the machine
     */
    enum State {
        TEST, ALLIANCE_SELECT, HIGH_JUNCTION_1, CONE_2, LOW_JUNCTION_2, CONE_3, LOW_JUNCTION_3, MID_JUNCTION_3, LEVEL_ADJUST, PARK, HALT, SET_DISTANCES
    }   // end of enum State

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

       parameters.vuforiaLicenseKey = VUFORIA_KEY;
       parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
       vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

}       //End Linear Op Mode

