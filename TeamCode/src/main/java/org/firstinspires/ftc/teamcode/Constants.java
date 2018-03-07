package org.firstinspires.ftc.teamcode;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants {
    // MysteryRobot Properties
    public static double kWheelDiameter = 1.0;
    public static double kWheelbaseWidth = 1.0;
    public static int kTicksPerRevolution = 1;
    public static double kMaxAcceleration = 0.001; // Whatever Unit makes this work

    // Vuforia
    public static String VuforiaLicenseKey = "ASYmU1X/////AAAAGeRbXZz3301OjdKqrFOt4OVPb5SKSng95X7hatnoDNuElPjQYMuihKQH5o9PC1jUJXk8lD12tefsfHj1PDgB6ga2gfL08UI3WL62Fov/j8rDLTMqKqBOP+jljOXhePm0stYgsK2+aSVaOIHmpY84uZHQ4pNExqGfkESerC6Nz1BZvDO/9zumPcCF98CjmcaGBGv4va7Kjd7XEQRBt0p+j+PAa9wYXywulvmqVnWTfh3fGiVWotAhI8jmzdxRAwTcutcl9CIBulmPa8/cGI3dGKXkKiXyR62gkgPOtLriz8lOzxwnyLC5vWPrr1MqbX5TRfrls3IQdQyfvPrtWnqirdtsWQ7m0eTNSC1/J1flxeaW";

    // PID gains for drive velocity loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveVelocityKp = 1.0;
    public static double kDriveVelocityKi = 0.0;
    public static double kDriveVelocityKd = 6.0;
    public static double kDriveVelocityKf = 0.5;
    public static double kDriveMaxDps = 1920; // Max degrees per second, neveRest 20

    // PID gains for motion profiling.
    public static double kPathfinderKp = 0.5;
    public static double kPathfinderKi = 0.5;
    public static double kPathfinderKd = 0.5;
    public static double kPathfinderKa = 0.5;
    public static double kPathfinderMaxVelocity = 0;
    public static double kPathfinderMaxAcceleration = 0;
    public static double kPathfinderMaxJerk = 0;

    public static double kBotLeftClawOpen = 0.56;
    public static double kBotLeftClawClosed = 0.13;
    public static double kBotRightClawOpen = 0.44;
    public static double kBotRightClawClosed = 0.85;
    public static double kTopLeftClawOpen = 0.9;
    public static double kTopLeftClawClosed = 0.6;
    public static double kTopRightClawOpen = 0.0;
    public static double kTopRightClawClosed = 0.6;
    public static double kTopJewelUp = 0;
    public static double kTopJewelDown = 0.5;
    public static double kBottomJewelOpen = 0.5;
    public static double kBottomJewelClosed = 0.2;

    public static int kClawOpen = 0;
    public static int kClawClosed = -540;


    public static int kLiftZero = 0;
    public static int kLiftOne = -2331;
    public static int kLiftTwo = -3741;
    public static int kLiftThree = -5164;
    public static int kLiftMax = -5350;





}
