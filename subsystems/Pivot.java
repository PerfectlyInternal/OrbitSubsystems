package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OrbitPID;

public class Pivot extends SubsystemBase {

    public class PivotConfig {
        public String name;

        // Generic config constants
        public boolean dualMotor; // Are we using two motors? 
        // Motor config
        // You can either set the motors directly and pre-configure them, or use builtin config options
        public int masterID, slaveID; 
        public boolean masterInvert, slaveInvert;
        public double gearRatio;
        public CANSparkMax master, slave;

        // Encoder config
        // Can be config'd the same way the motors are
        public int encoderPort;
        public double encoderOffset;
        public AnalogEncoder absoluteEncoder;

   }

    private PivotConfig config;

    // Motion inputs
    private double targetAngle;
    
    // Motion outputs & calculation stuff
    private double angularVelocity;  // angular velocity in deg / second
    private double lastAngle;
    private long lastTime;

    private boolean moving;

    public Pivot(PivotConfig _config) {
        config = _config;
    
        // only set motors and encoder if they werent alr set
        if (config.master == null) config.master = new CANSparkMax(config.masterID);
        if (config.dualMotor && config.slave == null) config.slave = new CANSparkMax(config.slaveID);
        if (config.absoluteEncoder == null) config.absoluteEncoder = new AnalogEncoder(config.encoderPort);

        if (config.dualMotor) config.slave.follow(config.master);

        config.master.getEncoder().setPositionConversionFactor(config.gearRatio);

        this.resetEncoder();
    }

    public void setVoltage(double v) {
        config.master.setVoltage(v);
    }

    public void setNormalizedVoltage(double v) {
        config.master.setVoltage(v * 12.0);
    }

    public double getRotations() {
        return config.master.getEncoder().getPosition() - config.encoderOffset;
    }

    public double getDegrees() {
        return getRotations() * 360.0;
    }

    public void resetEncoder() {
        double newPosition = config.absoluteEncoder.getPosition() - config.encoderOffset;
        config.master.getEncoder.setPosition(newPosition);
    }

    public double getSpeed() {
        return this.angularVelocity;
    }

    private void updateSpeed() {
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0;

        this.angularVelocity = (this.getACPAngle() - lastAngle) / deltaTime;
        this.lastAngle = this.getACPAngle();
        this.lastTime = currentTime;
    }

    @Override
    public void periodic() {
        this.updateSpeed();
    }
}
