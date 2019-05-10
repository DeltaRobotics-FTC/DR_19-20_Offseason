package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import for_camera_opmodes.LinearOpModeCamera;

//This is a test comment for a test commit :)
@Autonomous(name = "DemoBallTargetChunkAnalysis", group = "")
public class DemoBallTargetChunkAnalysis extends LinearOpModeCamera {

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






    //Conversion factor from pixels to degrees
    final double PIXEL_DEGREES = 0.0224;
    //Conversion factor from pixels to encoder counts
    final double PIXEL_ENCODER = 0.694;

    int ballXPixel = 0;
    int ballYPixel = 0;

    int xPixelDelta = 0;
    int yPixelDelta = 0;

    int xMin = 0;
    int yMin = 0;
    int xMax = 0;
    int yMax = 0;

    int greatestRGBBlue = 0;
    int greatestBlueBoxNumber = 0;
    int currentBox = 1;
    int blueXMin = 0;
    int blueXMax = 0;
    int blueYMin = 0;
    int blueYMax = 0;

    //Rows first them columns
    int boxRows = 2;

    final int IMAGE_CUTOFF = 550;
    int boxNumber = 6;
    int boxWidth = 0;
    int boxHeight = 0;

    int backwardEncoderTarget = 0;

    Drive_MK2 drive = new Drive_MK2();

    BNO055IMU imu;
    Orientation angles;


    @Override
    public void runOpMode() {
        /*Maps motor objects to name in robot configuration
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

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters(); //Declares parameters object forIMU
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES; //Sets the unit in which we measure orientation in degrees
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; //Sets acceleration unit in meters per second ??
        parametersIMU.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode, sets what file the IMU ueses
        parametersIMU.loggingEnabled = true; //Sets wether logging in enable
        parametersIMU.loggingTag = "IMU"; //Sets logging tag
        parametersIMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); //Sets acceleration integration algorithm
        parametersIMU.temperatureUnit = BNO055IMU.TempUnit.CELSIUS; //Sets units for temperature readings
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //Inits IMU
        imu.initialize(parametersIMU); //Init IMU parameters (set above)
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
        telemetry.addData("Init Orientation", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays initial orientation
        */




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

                boxWidth = rgbImage.getWidth() / (boxNumber / boxRows);
                boxHeight = (rgbImage.getHeight() - IMAGE_CUTOFF) / boxRows;
                xMax = boxWidth;
                xMin = 0;
                yMax = boxHeight + IMAGE_CUTOFF;
                yMin = IMAGE_CUTOFF;

                telemetry.addData("Box Width", boxWidth);
                telemetry.addData("Box Height", boxHeight);
                telemetry.update();
                sleep(2000);

                for(int i = 0; i < boxRows; i++, yMin = boxHeight + IMAGE_CUTOFF, yMax += boxHeight - 1, xMin = 0, xMax = boxWidth)
                {
                    telemetry.addData("yMin First Loop", yMin);
                    telemetry.addData("yMax First Loop", yMax);
                    telemetry.addData("xMin First Loop", xMin);
                    telemetry.addData("xMax First Loop", xMax);
                    telemetry.update();
                    sleep(5000);

                    for(int v = 0; v < (boxNumber / boxRows); v++, xMin += boxWidth - 1, xMax += boxWidth - 1)
                    {
                        for (int x = xMin; x <= xMax; x++) {
                            for (int y = yMin; y <= yMax; y++) {
                                if (x == xMax && y <= yMax) {
                                    rgbImage.setPixel(x, y, Color.rgb(255, 0, 0));
                                }
                                if (x <= xMax && y == yMin) {
                                    rgbImage.setPixel(x, y, Color.rgb(255, 0, 0));
                                }
                                if (x == xMin && y <= yMax) {
                                    rgbImage.setPixel(x, y, Color.rgb(255, 0, 0));
                                }
                                if (x <= xMax && y == yMax) {
                                    rgbImage.setPixel(x, y, Color.rgb(255, 0, 0));

                                }
                            }
                        }
                    }

                }




                /*for (int x = blueXMin; x <= blueXMax; x++) {
                    for (int y = blueYMin; y <= blueYMax; y++) {
                        if (x == blueXMax && y <= blueYMax) {
                            rgbImage.setPixel(x, y, Color.rgb(0, 0, 255));
                        }
                        if (x <= blueXMax && y == blueYMin) {
                            rgbImage.setPixel(x, y, Color.rgb(0, 0, 255));
                        }
                        if (x == blueXMin && y <= blueYMax) {
                            rgbImage.setPixel(x, y, Color.rgb(0, 0, 255));
                        }
                        if (x <= blueXMax && y == blueYMax) {
                            rgbImage.setPixel(x, y, Color.rgb(0, 0, 255));

                        }
                    }
                }
                */



                SaveImage(rgbImage);
                sleep(5000);
                boxWidth = rgbImage.getWidth() / (boxNumber / boxRows);
                boxHeight = (rgbImage.getHeight() - IMAGE_CUTOFF) / boxRows;
                xMax = boxWidth;
                xMin = 0;
                yMax = boxHeight + IMAGE_CUTOFF;
                yMin = IMAGE_CUTOFF;

                for(int i = 0; i < boxRows; i++, yMin = boxHeight + IMAGE_CUTOFF, yMax += boxHeight - 1, xMin = 0, xMax = boxWidth)
                {

                    for(int v = 0; v < (boxNumber / boxRows); v++, xMin += boxWidth - 1, xMax += boxWidth - 1, currentBox++)
                    {
                        for (int x = xMin; x < xMax; x++)
                        {
                            for (int y = yMin; y < yMax; y++)
                            {
                                int pixel = rgbImage.getPixel(x, y);
                                redValueLeft += red(pixel);
                                blueValueLeft += blue(pixel);
                                greenValueLeft += green(pixel);
                            }
                        }
                        redValueLeft = normalizePixels(redValueLeft);
                        blueValueLeft = normalizePixels(blueValueLeft);
                        greenValueLeft = normalizePixels(greenValueLeft);

                        if(blueValueLeft > greatestRGBBlue)
                        {
                            greatestRGBBlue = blueValueLeft;
                            greatestBlueBoxNumber = currentBox;
                            blueXMax = xMax;
                            blueXMin = xMin;
                            blueYMax = yMax;
                            blueYMin = yMin;
                        }

                    }

                }
                telemetry.addData("Greatest Blue", greatestRGBBlue);
                telemetry.addData("Box Number", greatestBlueBoxNumber);
                telemetry.update();
                sleep(5000);

                //Analyzing Jewel Color
                    /*for (int y = 0; y < rgbImage.getHeight(); y++)
                    {

                        for (int x = 0; x < rgbImage.getWidth(); x++)
                        {
                            int pixel = rgbImage.getPixel(x, y);
                            /*if(red(rgbImage.getPixel(x, y)) > 185 && green(rgbImage.getPixel(x, y)) < 20 && blue(rgbImage.getPixel(x, y)) < 35)
                            {
                                ballXPixel = x;
                                ballYPixel = y;
                            }

                            if(red(rgbImage.getPixel(x, y)) < 40 && red(rgbImage.getPixel(x, y)) > 20 && blue(rgbImage.getPixel(x, y)) > 220 && green(rgbImage.getPixel(x, y)) > 175)
                            {
                                ballXPixel = x;
                                ballYPixel = y;
                            }


                        }
                    }

                telemetry.addData("Width", rgbImage.getWidth());
                telemetry.addData("Height", rgbImage.getHeight());
                telemetry.addData("X Coord", ballXPixel);
                telemetry.addData("Y Coord", ballYPixel);
                telemetry.addData("Red Pixel", red(rgbImage.getPixel(ballXPixel, ballYPixel)));
                telemetry.addData("Green Pixel", green(rgbImage.getPixel(ballXPixel, ballYPixel)));
                telemetry.addData("Blue Pixel", blue(rgbImage.getPixel(ballXPixel, ballYPixel)));

                telemetry.update();
                sleep(5000);
                stopCamera();



*/
                    stopCamera();
            }
        }

        //Sets target deltas for movement
        /*xPixelDelta = PIXEL_X_TARGET - ballXPixel;
        yPixelDelta = PIXEL_Y_TARGET - ballYPixel;

        backwardEncoderTarget = (int)(yPixelDelta * PIXEL_ENCODER);

        telemetry.addData("X Pixel Delta", xPixelDelta);
        telemetry.addData("Y Pixel Delta", yPixelDelta);

        telemetry.addData("Degrees to rotate", (xPixelDelta * PIXEL_DEGREES));
        telemetry.addData("Encoder to go to", backwardEncoderTarget);
        telemetry.update();
        sleep(7000);

        //Orientating towards ball
        drive.OrientationDrive((xPixelDelta * PIXEL_DEGREES), 0.6, motors, imu);
        sleep(2000);

        //Moving towards ball
        drive.encoderDrive(backwardEncoderTarget, driveStyle.BACKWARD, 0.8, motors);
        */


        }
    }

