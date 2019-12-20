package frc.team4373.robot.commands.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.input.WheelVector;
import frc.team4373.robot.subsystems.Drivetrain;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.SwerveModifier;

public class MotionProfileCommand extends Command {

    private final EncoderFollower[] followers;
    private Drivetrain drivetrain;

    /**
     * Constructs a new MotionProfileCommand.
     */
    public MotionProfileCommand() {
        requires(this.drivetrain = Drivetrain.getInstance());

        final Waypoint[] demoPoints = new Waypoint[] {
            new Waypoint(0, 0, 0),
            new Waypoint(5 / RobotMap.ENCODER_UNITS_TO_IN, 0, 0),
            new Waypoint(10 / RobotMap.ENCODER_UNITS_TO_IN,
                    5 / RobotMap.ENCODER_UNITS_TO_IN, 0)
        };

        final Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
                Trajectory.Config.SAMPLES_HIGH, 1.0 / RobotMap.SCHEDULER_EXEC_RATE,
                RobotMap.MAX_VELOCITY, RobotMap.MAX_ACCEL, RobotMap.MAX_JERK);
        final SwerveModifier mod = new SwerveModifier(Pathfinder.generate(demoPoints, config))
                .modify(RobotMap.ROBOT_TRACKWIDTH, RobotMap.ROBOT_WHEELBASE,
                        SwerveModifier.Mode.SWERVE_DEFAULT);

        this.followers = new EncoderFollower[] {
            new EncoderFollower(mod.getFrontRightTrajectory()),
            new EncoderFollower(mod.getBackRightTrajectory()),
            new EncoderFollower(mod.getFrontLeftTrajectory()),
            new EncoderFollower(mod.getBackLeftTrajectory())
        };

        for (EncoderFollower follower: followers) {
            follower.configureEncoder(0, RobotMap.WHEEL_ENCODER_TICKS_PER_REV,
                    RobotMap.WHEEL_DIAMETER);
            follower.configurePIDVA(RobotMap.MOT_PROF_PID.kP,RobotMap.MOT_PROF_PID.kI,
                    RobotMap.MOT_PROF_PID.kD, 1 / RobotMap.MAX_VELOCITY, 0);
        }
    }

    @Override
    protected void initialize() {
        for (EncoderFollower follower: followers) {
            follower.reset();
        }
    }

    @Override
    protected void execute() {
        double r1Output = followers[0].calculate(
                (int) drivetrain.getWheelLinearPos(Drivetrain.WheelID.RIGHT_1));
        double r1Heading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(followers[0].getHeading()));
        WheelVector r1Vec = new WheelVector(r1Output, r1Heading);

        double r2Output = followers[1].calculate(
                (int) drivetrain.getWheelLinearPos(Drivetrain.WheelID.RIGHT_2));
        double r2Heading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(followers[1].getHeading()));
        WheelVector r2Vec = new WheelVector(r2Output, r2Heading);

        double l1Output = followers[2].calculate(
                (int) drivetrain.getWheelLinearPos(Drivetrain.WheelID.LEFT_1));
        double l1Heading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(followers[2].getHeading()));
        WheelVector l1Vec = new WheelVector(l1Output, l1Heading);

        double l2Output = followers[3].calculate(
                (int) drivetrain.getWheelLinearPos(Drivetrain.WheelID.LEFT_2));
        double l2Heading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(followers[3].getHeading()));
        WheelVector l2Vec = new WheelVector(l2Output, l2Heading);

        drivetrain.setWheelsPID(new WheelVector.VectorSet(r1Vec, r2Vec, l1Vec, l2Vec));
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
