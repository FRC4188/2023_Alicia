// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import csplib.motors.CSP_Falcon;
import csplib.utils.Conversions;
import csplib.utils.TempManager;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule {
    private CSP_Falcon speed;
    private CSP_Falcon angle; 
    private WPI_CANCoder encoder;
    private double zero;
    private double anglekP;
    private double anglekI;
    private double anglekD;

    /**
     * Creates a SwerveModule object
     * @param speedID CAN ID of the Falcon 500 controlling speed
     * @param angleID CAN ID of the Falcon 500 controlling angle
     * @param encoderID CAN ID of the Cancoder reading the angle
     * @param zero angle at which the module is zeroed 
     * @param anglekP proportional gain for controlling angle
     * @param anglekI integral gain for controlling angle
     * @param anglekD derivative gain for controlling angle
     */
    public SwerveModule(int speedID, int angleID, int encoderID, double zero, double anglekP, double anglekI, double anglekD) {
        speed = new CSP_Falcon(speedID);
        angle = new CSP_Falcon(angleID);
        encoder = new WPI_CANCoder(encoderID);
        this.zero = zero;
        this.anglekP = anglekP;
        this.anglekI = anglekI;
        this.anglekD = anglekD;

        init();

        TempManager.addMotor(speed, angle);
    }

    /**
     * Configures devices for usage 
     */
    private void init() {
        speed.setBrake(true);
        speed.setRampRate(Constants.drivetrain.RAMP_RATE);
        speed.setPIDF(Constants.drivetrain.speed.kP, Constants.drivetrain.speed.kI, Constants.drivetrain.speed.kD, 0);
        speed.setScalar(Constants.drivetrain.DRIVE_COUNTS_PER_METER);

        encoder.configFactoryDefault();
        encoder.clearStickyFaults();
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        encoder.setPosition(0.0);
        encoder.configSensorDirection(false);
        encoder.configMagnetOffset(-zero);

        angle.configFactoryDefault();
        angle.setBrake(false);
        angle.setScalar(Constants.drivetrain.ANGLE_DEGREES_PER_TICK);  
        System.out.println(Conversions.degreesSignedToUnsigned(encoder.getAbsolutePosition()));
        angle.setEncoder(Conversions.degreesSignedToUnsigned(encoder.getAbsolutePosition()));
        angle.configFeedbackNotContinuous(true, 0);
        angle.setPIDF(anglekP, anglekI, anglekD, anglekD);
    }

    /**
     * Sets the speed and angle of the module
     * @param desired SwerveModuleState object containing desired velocity and angle
     */
    public void setModuleState(SwerveModuleState desired) {
        SwerveModuleState optimized = SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(getAngle()));
        speed.setVelocity(optimized.speedMetersPerSecond);
        angle.setPosition(optimized.angle.getDegrees());
    }

    /**
     * Sets the PIDF gains for the speed motor
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param kF feedforward gain
     */
    public void setSpeedPIDF(double kP, double kI, double kD, double kF) {
        speed.setPIDF(kP, kI, kD, kF);
    }

    /**
     * Sets the PIDF gains for the angle motor
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param kF feedforward gain
     */
    public void setAnglePIDF(double kP, double kI, double kD, double kF) {
        angle.setPIDF(kP, kI, kD, kF);
    }

    /**
     * Sets the speed and angle motors to zero power
     */
    public void zeroPower() {
        angle.set(0.0);
        speed.set(0.0);
    }

    /**
     * Gives the velocity and angle of the module
     * @return SwerveModuleState object containing velocity and angle
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(speed.getVelocity(), Rotation2d.fromDegrees(getAngle()));
    }

    /**
     * Gives the position and angle of the module
     * @return SwerveModulePosition object containing position and angle
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(speed.getPosition(), Rotation2d.fromDegrees(getAngle()));
    }

    /**
     * Gives the angle of the module and constantly corrects it with Cancoder reading
     * @return the angle with range [0, 360]
     */
    private double getAngle() {
        double motorAngle = Conversions.degreesSignedToUnsigned(angle.getPosition());
        if (Math.abs(motorAngle - encoder.getAbsolutePosition()) > 0.1) {
            angle.setEncoder(Conversions.degreesSignedToUnsigned(encoder.getAbsolutePosition()));
        }

        return motorAngle;
    }

    /**
     * Gives the absolute angle of the module read by the Cancoder 
     * @return the absolute angle with range [-180, 180]
     */
    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition();
    }

    /**
     * Gives the temperatures of the speed and angle motors
     * @return an array containing speed and angle motor temperatures
     */
    public double[] getTemperature() {
        return new double[] {speed.getTemperature(), angle.getTemperature()};
    }
}

