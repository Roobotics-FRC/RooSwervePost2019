package frc.team4373.robot.commands.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.team4373.robot.RobotMap;
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
            new Waypoint(5, 0, 0),
            new Waypoint(10, 5, 0)
        };
        final Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
                Trajectory.Config.SAMPLES_HIGH, 1.0 / 50.0, RobotMap.MAX_VELOCITY,
                RobotMap.MAX_ACCEL, RobotMap.MAX_JERK);
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
        // TODO: fill in
        // ref https://github.com/Paradox2102/SwerveDrive/blob/master/src/team2102/robot/subsystems/DriveSubsystem.java
        // ref https://github.com/JacisNonsense/Pathfinder/wiki/Pathfinder-for-FRC---Java
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
