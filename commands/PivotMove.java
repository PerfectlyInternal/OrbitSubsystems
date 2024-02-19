package frc.robot.commands.Pivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.util.OrbitTimer;

public class PivotMove extends Command {
    
    public class PivotMoveConfig {
        public Pivot pivot;

        public boolean directionalMotionProfiles; // Do we want to use separate up/down motion profiles? (for big loads)

        // Motion profiles, up is used by default if you only have one set
        // Set like motors
        public double maxVel, maxAccel;
        public TrapezoidProfile.Constraints motionProfileConstraints; 

        // PIDF controllers
        public OrbitPID pid;
        public ArmFeedforward feedForward;  
    }

    // this changes often between commands for the same pivot, let it be separate
    private double targetAngle;

    // interal motion profile state
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;

    // timing for motion profile
    private OrbitTimer timer;

    // internal calculation vars
    private double target, input, pidOutput;
    
    public ACPGoToPositionCommand(PivotMoveConfig _config, double angle) {
        this.config = _config;
        this.angle = angle;

        if (config.upMotionProfileConstraints

        this.timer = new OrbitTimer();
        addRequirements(config.pivot);
    }

    @Override
    public void initialize() {
        config.pid.reset();
        
        this.startState = new TrapezoidProfile.State(config.pivot.getDegrees(), 0.0);
        this.endState = new TrapezoidProfile.State(this.angle), 0.0);

        this.motionProfile = new TrapezoidProfile(config.motionProfileConstraints);

        this.timer.start();
    }

    @Override
    public void execute() {
        // Rest of your execute method remains unchanged

        TrapezoidProfile.State profileTarget = 
            motionProfile.calculate(
                this.timer.getTimeDeltaSec(),
            	this.startState, this.endState
            );
	
        target = profileTarget.position;
        input = config.pivot.getDegrees();
        pidOutput = config.pid.calculate(target, input);
        config.pivot.setNormalizedVoltage(speed);
    }

    @Override
    public boolean isFinished() {
        return this.motionProfile.isFinished(this.timer.getTimeDeltaSec());
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Shoulder_Move_Time", this.timer.getTimeDeltaSec());
        config.pivot.setVoltage(0.0); // stop
    }
}
