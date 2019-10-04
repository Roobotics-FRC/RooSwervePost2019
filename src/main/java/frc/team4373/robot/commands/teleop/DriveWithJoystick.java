package frc.team4373.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.team4373.robot.input.OI;
import frc.team4373.robot.subsystems.Drivetrain;

public class DriveWithJoystick extends Command {
        private Drivetrain drivetrain;

        public DriveWithJoystick(){
            requires(this.drivetrain = Drivetrain.getInstance());

        }

        @Override
        public void initialize() {
            this.drivetrain.zeroMotors();
        }

        @Override
        public void execute() {
            double twist = OI.getInstance().getDriveJoystick().rooGetZ();
            double forward = OI.getInstance().getDriveJoystick().rooGetY();
            this.drivetrain.getSwerveWheel(Drivetrain.WheelID.RIGHT_1).setPercentOutput(forward, twist);
            this.drivetrain.getSwerveWheel(Drivetrain.WheelID.RIGHT_2).setPercentOutput(forward, twist);
            this.drivetrain.getSwerveWheel(Drivetrain.WheelID.LEFT_1).setPercentOutput(forward, twist);
            this.drivetrain.getSwerveWheel(Drivetrain.WheelID.LEFT_2).setPercentOutput(forward, twist);

        }
        @Override
        public boolean isFinished(){
            return false;
        }

        @Override
        public void end(){
            this.drivetrain.zeroMotors();
        }

        @Override
        public void interrupted(){
            this.end();
        }
    }



