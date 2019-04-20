package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import for_camera_opmodes.CameraPreview;
import for_camera_opmodes.LinearOpModeCamera;

@TeleOp (name = "NewCamera Testing", group = "")
public class NewCameraTesting extends LinearOpModeCamera
{

    int mineralColorInt;

    //Min and max values for the bounds of the area of the image that will be analyzed
    int xMin = 0;
    int xMax = 10;

    int yMin = 0;
    int yMax = 10;

    @Override
    public void runOpMode()
    {

            if(isCameraAvailable())
            {
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


                        //This is for only saving the color image if needed.

                        for (int x = xMin; x <= xMax; x++) {
                            for (int y = yMin; y <= yMax; y++) {
                                if (x == xMax && y <= yMax) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                                }
                                if (x <= xMax && y == yMin) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                                }
                                if (x == xMin && y <= yMax) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                                }
                                if (x <= xMax && y == yMax) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));

                                }
                            }
                        }




                        SaveImage(rgbImage);

                        //Analyzing Jewel Color
                        for (int x = xMin; x < xMax; x++) {
                            for (int y = yMin; y < yMax; y++) {
                                int pixel = rgbImage.getPixel(x, y);
                                redValueLeft += red(pixel);
                                blueValueLeft += blue(pixel);
                                greenValueLeft += green(pixel);
                            }
                        }
                        redValueLeft = normalizePixels(redValueLeft);
                        blueValueLeft = normalizePixels(blueValueLeft);
                        greenValueLeft = normalizePixels(greenValueLeft);
                        /*telemetry.addData("redValueLeft", redValueLeft);
                        telemetry.addData("blueValueLeft", blueValueLeft);
                        telemetry.addData("greenValueLeft", greenValueLeft);
                        telemetry.addData("Width", rgbImage.getWidth());
                        telemetry.addData("Height", rgbImage.getHeight());
                        telemetry.update();*/


                        mineralColorInt = highestColor(redValueLeft, blueValueLeft, greenValueLeft);

                        telemetry.addData("Mineral", mineralColorInt);
                        if (Math.abs((redValueLeft - blueValueLeft)) <= 5) {
                            telemetry.addData("Mineral Color", "0 : Silver");
                            telemetry.addData("Red", redValueLeft);
                            telemetry.addData("Blue", blueValueLeft);
                            telemetry.addData("Green", greenValueLeft);
                            telemetry.addData("Red Blue Difference", Math.abs((redValueLeft - blueValueLeft)));
                        } else
                        {
                            telemetry.addData("Mineral Color", "Gold");
                            telemetry.addData("Red", redValueLeft);
                            telemetry.addData("Blue", blueValueLeft);
                            telemetry.addData("Green", greenValueLeft);
                        }
                        telemetry.addData("Width", rgbImage.getWidth());
                        telemetry.addData("Height", rgbImage.getHeight());
                        telemetry.update();
                        sleep(5000);
                        stopCamera();



                    }
                }

                sleep(5000);

                if (isCameraAvailable())
                {
                    xMax = 845;
                    xMin = 290;

                    yMin = 925;
                    yMax = 1175;

                    setCameraDownsampling(1);

                    startCamera();

                    if(imageReady())
                    {
                        int redValueLeft = -76800;
                        int blueValueLeft = -76800;
                        int greenValueLeft = -76800;

                        Bitmap rgbImage;
                        //The last value must correspond to the downsampling value from above
                        rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);


                        //This is for only saving the color image if needed.

                        for (int x = xMin; x <= xMax; x++) {
                            for (int y = yMin; y <= yMax; y++) {
                                if (x == xMax && y <= yMax) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                                }
                                if (x <= xMax && y == yMin) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                                }
                                if (x == xMin && y <= yMax) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                                }
                                if (x <= xMax && y == yMax) {
                                    rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));

                                }
                            }
                        }




                        SaveImage(rgbImage);
                        //Analyzing Jewel Color
                        for (int x = xMin; x < xMax; x++) {
                            for (int y = yMin; y < yMax; y++) {
                                int pixel = rgbImage.getPixel(x, y);
                                redValueLeft += red(pixel);
                                blueValueLeft += blue(pixel);
                                greenValueLeft += green(pixel);
                            }
                        }
                        redValueLeft = normalizePixels(redValueLeft);
                        blueValueLeft = normalizePixels(blueValueLeft);
                        greenValueLeft = normalizePixels(greenValueLeft);
                        /*telemetry.addData("redValueLeft", redValueLeft);
                        telemetry.addData("blueValueLeft", blueValueLeft);
                        telemetry.addData("greenValueLeft", greenValueLeft);
                        telemetry.addData("Width", rgbImage.getWidth());
                        telemetry.addData("Height", rgbImage.getHeight());
                        telemetry.update();*/


                       /* mineralColorInt = highestColor(redValueLeft, blueValueLeft, greenValueLeft);

                        telemetry.addData("Mineral", mineralColorInt);
                        if (Math.abs((redValueLeft - blueValueLeft)) <= 5) {
                            telemetry.addData("Mineral Color", "0 : Silver");
                            telemetry.addData("Red", redValueLeft);
                            telemetry.addData("Blue", blueValueLeft);
                            telemetry.addData("Green", greenValueLeft);
                            telemetry.addData("Red Blue Difference", Math.abs((redValueLeft - blueValueLeft)));
                        } else
                        {
                            telemetry.addData("Mineral Color", "Gold");
                            telemetry.addData("Red", redValueLeft);
                            telemetry.addData("Blue", blueValueLeft);
                            telemetry.addData("Green", greenValueLeft);
                        }
                        telemetry.addData("Width", rgbImage.getWidth());
                        telemetry.addData("Height", rgbImage.getHeight());
                        telemetry.update();
                        sleep(5000);*/
                        stopCamera();
                    }
                }

                sleep(5000);

        if (isCameraAvailable())
        {
            xMax = 845;
            xMin = 290;

            yMin = 925;
            yMax = 1175;

            setCameraDownsampling(1);

            startCamera();

            if(imageReady())
            {
                int redValueLeft = -76800;
                int blueValueLeft = -76800;
                int greenValueLeft = -76800;

                Bitmap rgbImage;
                //The last value must correspond to the downsampling value from above
                rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);


                //This is for only saving the color image if needed.

                for (int x = xMin; x <= xMax; x++) {
                    for (int y = yMin; y <= yMax; y++) {
                        if (x == xMax && y <= yMax) {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                        }
                        if (x <= xMax && y == yMin) {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                        }
                        if (x == xMin && y <= yMax) {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                        }
                        if (x <= xMax && y == yMax) {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));

                        }
                    }
                }




                SaveImage(rgbImage);
                //Analyzing Jewel Color
                for (int x = xMin; x < xMax; x++) {
                    for (int y = yMin; y < yMax; y++) {
                        int pixel = rgbImage.getPixel(x, y);
                        redValueLeft += red(pixel);
                        blueValueLeft += blue(pixel);
                        greenValueLeft += green(pixel);
                    }
                }
                redValueLeft = normalizePixels(redValueLeft);
                blueValueLeft = normalizePixels(blueValueLeft);
                greenValueLeft = normalizePixels(greenValueLeft);
                        /*telemetry.addData("redValueLeft", redValueLeft);
                        telemetry.addData("blueValueLeft", blueValueLeft);
                        telemetry.addData("greenValueLeft", greenValueLeft);
                        telemetry.addData("Width", rgbImage.getWidth());
                        telemetry.addData("Height", rgbImage.getHeight());
                        telemetry.update();*/


                mineralColorInt = highestColor(redValueLeft, blueValueLeft, greenValueLeft);

                telemetry.addData("Mineral", mineralColorInt);
                if (Math.abs((redValueLeft - blueValueLeft)) <= 5) {
                    telemetry.addData("Mineral Color", "0 : Silver");
                    telemetry.addData("Red", redValueLeft);
                    telemetry.addData("Blue", blueValueLeft);
                    telemetry.addData("Green", greenValueLeft);
                    telemetry.addData("Red Blue Difference", Math.abs((redValueLeft - blueValueLeft)));
                } else
                {
                    telemetry.addData("Mineral Color", "Gold");
                    telemetry.addData("Red", redValueLeft);
                    telemetry.addData("Blue", blueValueLeft);
                    telemetry.addData("Green", greenValueLeft);
                }
                telemetry.addData("Width", rgbImage.getWidth());
                telemetry.addData("Height", rgbImage.getHeight());
                telemetry.update();
                sleep(5000);
                stopCamera();
            }
        }

        }

}
