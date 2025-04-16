from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
import math
from ntcore import NetworkTableInstance
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.util import DriveFeedforwards
from phoenix6 import SignalLogger, swerve, units, utils
from typing import Callable, overload
from wpilib import DriverStation, Notifier, RobotController
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds


class CommandSwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain):  # type: ignore
    _SIM_LOOP_PERIOD: units.second = 0.005

    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)

    @overload
    def __init__(  # type: ignore
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        modules: list[swerve.SwerveModuleConstants],  # type: ignore
    ) -> None: ...

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        modules: list[swerve.SwerveModuleConstants],  # type: ignore
    ) -> None: ...

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        odometry_standard_deviation: tuple[float, float, float],
        vision_standard_deviation: tuple[float, float, float],
        modules: list[swerve.SwerveModuleConstants],  # type: ignore
    ) -> None: ...

    def __init__(  # type: ignore
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        arg0=None,  # type: ignore
        arg1=None,  # type: ignore
        arg2=None,  # type: ignore
        arg3=None,  # type: ignore
    ):
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(  # type: ignore
            self,
            drive_motor_type,
            steer_motor_type,
            encoder_type,
            drivetrain_constants,
            arg0,  # type: ignore
            arg1,  # type: ignore
            arg2,  # type: ignore
            arg3,  # type: ignore
        )

        self._sim_notifier: Notifier | None = None
        self._last_sim_time: units.second = 0.0

        self._has_applied_operator_perspective = False

        self._apply_robot_speeds = swerve.requests.ApplyRobotSpeeds()

        self._translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self._steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self._rotation_characterization = swerve.requests.SysIdSwerveRotation()

        self._drive_state_table = NetworkTableInstance.getDefault().getTable(
            "DriveState"
        )
        self._wanted_chassis_speeds = self._drive_state_table.getStructTopic(
            "WantedChassisSpeeds", ChassisSpeeds
        ).publish()

        self._sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage=4.0,
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                ),  # type: ignore
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._translation_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )

        self._sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage=7.0,
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),  # type: ignore
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._steer_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )

        self._sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate=math.pi / 6,
                stepVoltage=7.0,
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),  # type: ignore
            ),
            SysIdRoutine.Mechanism(
                lambda output: (
                    self.set_control(
                        self._rotation_characterization.with_rotational_rate(output)
                    ),
                    SignalLogger.write_double("Rotational_Rate", output),
                ),  # type: ignore
                lambda log: None,
                self,
            ),
        )

        self._sys_id_routine_to_apply = self._sys_id_routine_translation

        if utils.is_simulation():
            self._start_sim_thread()
        self._configure_auto_builder()

    def apply_pp_speeds(
        self, speeds: ChassisSpeeds, feedforwards: DriveFeedforwards
    ) -> None:
        self._wanted_chassis_speeds.set(speeds)
        self.set_control(
            self._apply_robot_speeds.with_speeds(speeds)
            .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
            .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
        )

    def _configure_auto_builder(self):
        config = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            lambda: self.get_state().pose,
            self.reset_pose,
            lambda: self.get_state().speeds,
            lambda speeds, feedforwards: self.apply_pp_speeds(speeds, feedforwards),
            PPHolonomicDriveController(
                PIDConstants(10.0, 0.0, 0.0),
                PIDConstants(7.0, 0.0, 0.0),
            ),
            config,
            lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue)
            == DriverStation.Alliance.kRed,
            self,
        )

    def apply_request(
        self, request: Callable[[], swerve.requests.SwerveRequest]
    ) -> Command:
        def applyAndLog():
            req = request()
            wanted_chassis_speeds = ChassisSpeeds()
            if isinstance(
                req,
                (
                    swerve.requests.FieldCentric,
                    swerve.requests.FieldCentricFacingAngle,
                    swerve.requests.ApplyFieldSpeeds,
                ),
            ):
                wanted_chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    req.velocity_x,  # type: ignore
                    req.velocity_y,  # type: ignore
                    req.rotational_rate,  # type: ignore
                    self.get_state().pose.rotation()
                    + self.get_operator_forward_direction(),
                )
            else:
                if (
                    hasattr(req, "velocity_x")
                    and hasattr(req, "velocity_y")
                    and hasattr(req, "rotational_rate")
                ):
                    wanted_chassis_speeds = ChassisSpeeds(
                        req.velocity_x,  # type: ignore
                        req.velocity_y,  # type: ignore
                        req.rotational_rate,  # type: ignore
                    )
            self._wanted_chassis_speeds.set(wanted_chassis_speeds)
            self.set_control(req)

        return self.run(applyAndLog)

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine_to_apply.dynamic(direction)

    def periodic(self):
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

    def _start_sim_thread(self):
        def _sim_periodic():
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        self._last_sim_time = utils.get_current_time_seconds()
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(self._SIM_LOOP_PERIOD)

    def add_vision_measurement(
        self,
        vision_robot_pose: Pose2d,
        timestamp: units.second,
        vision_measurement_std_devs: tuple[float, float, float] | None = None,
    ):
        swerve.SwerveDrivetrain.add_vision_measurement(  # type: ignore
            self,
            vision_robot_pose,
            utils.fpga_to_current_time(timestamp),
            vision_measurement_std_devs,
        )
