package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import for_camera_opmodes.LinearOpModeCamera;

@Autonomous(name = "DemoBallTarget", group = "")
public class DemoBallTarget extends LinearOpModeCamera {

    //This sets the motorXX to DcMotor Objects
    DcMotor motorRF = null;
    DcMotor motorLF = null;
    DcMotor motorRB = null;
    DcMotor motorLB = null;
    DcMotor shooter = null;

    //Target pixel for the X axis of photo
    final int PIXEL_X_TARGET = 624;
    //Target pixel for the Y axis of photo
    final int PIXEL_Y_TARGET = 1232;

    /*

    X axis    ^
     |        | <-- Y axis
     |        |
     V        |
    <---------o <-- target point. Top left corner of ball will be the target point.

     */






    //Conversion factor from pixels to degrees - needs to be set
    final double PIXEL_DEGREES = 0;
    //Conversion factor from pixels to encoder counts
    final double PIXEL_ENCODER = 0.694;

    Drive_MK2 drive = new Drive_MK2();


    @Override
    public void runOpMode() {
        //Maps motor objects to name in robot configuration
        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorLB = hardwareMap.dcMotor.get("motorLB");
        shooter = hardwareMap.dcMotor.get("shooter");

        //Setting motors to brake when stopped
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor[] motors = new DcMotor[4];
        motors[0] = motorRF;
        motors[1] = motorRB;
        motors[2] = motorLB;
        motors[3] = motorLF;

        //First picture
        if (isCameraAvailable()) {
            setCameraDownsampling(1);


            startCamera();

            waitForStart();


            if (imageReady()) {
                int redValueLeft = -76800;
                int blueValueLeft = -76800;
                int greenValueLeft = -76800;

                Bitmap rgbImage;
                //The last value must correspond to the downsampling value from above
                rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);


                SaveImage(rgbImage);

                //Analyzing Jewel Color
                for (int x = 0; x < rgbImage.getWidth(); x++) {
                    for (int y = 0; y < rgbImage.getHeight(); y++) {
                        int pixel = rgbImage.getPixel(x, y);

                    }
                }
                telemetry.addData("Width", rgbImage.getWidth());
                telemetry.addData("Height", rgbImage.getHeight());
                telemetry.update();
                sleep(2000);
                stopCamera();


            }

            //Second picture
            sleep(2000);
            if (isCameraAvailable()) {
                setCameraDownsampling(1);

                startCamera();


                if (imageReady()) {
                    int redValueLeft = -76800;
                    int blueValueLeft = -76800;
                    int greenValueLeft = -76800;

                    Bitmap rgbImage;
                    //The last value must correspond to the downsampling value from above
                    rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);


                    SaveImage(rgbImage);

                    //Analyzing Jewel Color
                    for (int x = 0; x < rgbImage.getWidth(); x++) {
                        for (int y = 0; y < rgbImage.getHeight(); y++) {
                            int pixel = rgbImage.getPixel(x, y);

                        }
                    }
                    telemetry.addData("Width", rgbImage.getWidth());
                    telemetry.addData("Height", rgbImage.getHeight());
                    telemetry.update();
                    sleep(2000);
                    stopCamera();


                }

                sleep(2000);

                //Third picture
                if (isCameraAvailable()) {
                    setCameraDownsampling(1);

                    startCamera();


                    if (imageReady()) {
                        int redValueLeft = -76800;
                        int blueValueLeft = -76800;
                        int greenValueLeft = -76800;

                        Bitmap rgbImage;
                        //The last value must correspond to the downsampling value from above
                        rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);


                        SaveImage(rgbImage);

                        //Analyzing Jewel Color
                        for (int x = 0; x < rgbImage.getWidth(); x++) {
                            for (int y = 0; y < rgbImage.getHeight(); y++) {
                                int pixel = rgbImage.getPixel(x, y);

                            }
                        }
                        telemetry.addData("Width", rgbImage.getWidth());
                        telemetry.addData("Height", rgbImage.getHeight());
                        telemetry.update();
                        sleep(2000);
                        stopCamera();


                    }
                }
            }
        }
    }
}

