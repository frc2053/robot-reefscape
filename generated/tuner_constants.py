import math
from phoenix6 import CANBus, configs, hardware, signals, swerve, units
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from wpimath.units import inchesToMeters
from wpimath.system.plant import DCMotor


class TunerConstants:
    _steer_gains = (
        configs.Slot0Configs()
        .with_k_p(100)
        .with_k_i(0)
        .with_k_d(0.05)
        .with_k_s(0.2403)
        .with_k_v(2.606)
        .with_k_a(0.095481)
        .with_static_feedforward_sign(
            signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
        )
    )
    _drive_gains = (
        configs.Slot0Configs()
        .with_k_p(7)
        .with_k_i(0)
        .with_k_d(0)
        .with_k_s(0.29943)
        .with_k_v(0.11982)
        .with_k_a(0.0031508)
    )

    _steer_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE
    _drive_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE

    _drive_motor_type = swerve.DriveMotorArrangement.TALON_FX_INTEGRATED
    _steer_motor_type = swerve.SteerMotorArrangement.TALON_FX_INTEGRATED

    _steer_feedback_type = swerve.SteerFeedbackType.FUSED_CANCODER

    _slip_current: units.ampere = 120.0

    _drive_initial_configs = configs.TalonFXConfiguration()
    _steer_initial_configs = configs.TalonFXConfiguration().with_current_limits(
        configs.CurrentLimitsConfigs()
        .with_stator_current_limit(60)
        .with_stator_current_limit_enable(True)
    )
    _encoder_initial_configs = configs.CANcoderConfiguration()

    _pigeon_configs: configs.Pigeon2Configuration | None = None

    canbus = CANBus("*", "./logs/example.hoot")

    _couple_ratio = 50.0 / 14.0

    _drive_gear_ratio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)
    _steer_gear_ratio = (50.0 / 14.0) * (60.0 / 10.0)
    _wheel_radius: units.meter = inchesToMeters(1.9345)

    speed_at_12_volts: units.meters_per_second = (
        DCMotor.krakenX60FOC().freeSpeed / _drive_gear_ratio
    ) * _wheel_radius

    _invert_left_side = False
    _invert_right_side = True

    _pigeon_id = 14

    # These are only used for simulation
    _steer_inertia: units.kilogram_square_meter = 0.01
    _drive_inertia: units.kilogram_square_meter = 0.01
    # Simulated voltage necessary to overcome friction
    _steer_friction_voltage: units.volt = 0.2
    _drive_friction_voltage: units.volt = 0.2

    _wheelbase_width: units.meter = inchesToMeters(22.75)
    _wheelbase_length: units.meter = inchesToMeters(22.75)
    _drivebase_radius: units.meter = math.hypot(
        _wheelbase_width / 2, _wheelbase_length / 2
    )

    drivetrain_constants = (
        swerve.SwerveDrivetrainConstants()
        .with_can_bus_name(canbus.name)
        .with_pigeon2_id(_pigeon_id)
        .with_pigeon2_configs(_pigeon_configs)
    )

    _constants_creator: swerve.SwerveModuleConstantsFactory[
        configs.TalonFXConfiguration,  # type: ignore
        configs.TalonFXConfiguration,  # type: ignore
        configs.CANcoderConfiguration,  # type: ignore
    ] = (
        swerve.SwerveModuleConstantsFactory()
        .with_drive_motor_gear_ratio(_drive_gear_ratio)
        .with_steer_motor_gear_ratio(_steer_gear_ratio)
        .with_coupling_gear_ratio(_couple_ratio)
        .with_wheel_radius(_wheel_radius)
        .with_steer_motor_gains(_steer_gains)
        .with_drive_motor_gains(_drive_gains)
        .with_steer_motor_closed_loop_output(_steer_closed_loop_output)
        .with_drive_motor_closed_loop_output(_drive_closed_loop_output)
        .with_slip_current(_slip_current)
        .with_speed_at12_volts(speed_at_12_volts)
        .with_drive_motor_type(_drive_motor_type)
        .with_steer_motor_type(_steer_motor_type)
        .with_feedback_source(_steer_feedback_type)
        .with_drive_motor_initial_configs(_drive_initial_configs)  # type: ignore
        .with_steer_motor_initial_configs(_steer_initial_configs)
        .with_encoder_initial_configs(_encoder_initial_configs)
        .with_steer_inertia(_steer_inertia)
        .with_drive_inertia(_drive_inertia)
        .with_steer_friction_voltage(_steer_friction_voltage)
        .with_drive_friction_voltage(_drive_friction_voltage)
    )

    # Front Left
    _front_left_drive_motor_id = 2
    _front_left_steer_motor_id = 3
    _front_left_encoder_id = 4
    _front_left_encoder_offset: units.rotation = 0.29687
    _front_left_steer_motor_inverted = True
    _front_left_encoder_inverted = False

    _front_left_x_pos: units.meter = _wheelbase_length / 2
    _front_left_y_pos: units.meter = _wheelbase_width / 2

    # Front Right
    _front_right_drive_motor_id = 5
    _front_right_steer_motor_id = 6
    _front_right_encoder_id = 7
    _front_right_encoder_offset: units.rotation = -0.046387
    _front_right_steer_motor_inverted = True
    _front_right_encoder_inverted = False

    _front_right_x_pos: units.meter = _wheelbase_length / 2
    _front_right_y_pos: units.meter = -_wheelbase_width / 2

    # Back Left
    _back_left_drive_motor_id = 8
    _back_left_steer_motor_id = 9
    _back_left_encoder_id = 10
    _back_left_encoder_offset: units.rotation = -0.396973
    _back_left_steer_motor_inverted = True
    _back_left_encoder_inverted = False

    _back_left_x_pos: units.meter = -_wheelbase_length / 2
    _back_left_y_pos: units.meter = _wheelbase_width / 2

    # Back Right
    _back_right_drive_motor_id = 11
    _back_right_steer_motor_id = 12
    _back_right_encoder_id = 13
    _back_right_encoder_offset: units.rotation = 0.124512
    _back_right_steer_motor_inverted = True
    _back_right_encoder_inverted = False

    _back_right_x_pos: units.meter = -_wheelbase_length / 2
    _back_right_y_pos: units.meter = -_wheelbase_width / 2

    front_left = _constants_creator.create_module_constants(
        _front_left_steer_motor_id,
        _front_left_drive_motor_id,
        _front_left_encoder_id,
        _front_left_encoder_offset,
        _front_left_x_pos,
        _front_left_y_pos,
        _invert_left_side,
        _front_left_steer_motor_inverted,
        _front_left_encoder_inverted,
    )
    front_right = _constants_creator.create_module_constants(
        _front_right_steer_motor_id,
        _front_right_drive_motor_id,
        _front_right_encoder_id,
        _front_right_encoder_offset,
        _front_right_x_pos,
        _front_right_y_pos,
        _invert_right_side,
        _front_right_steer_motor_inverted,
        _front_right_encoder_inverted,
    )
    back_left = _constants_creator.create_module_constants(
        _back_left_steer_motor_id,
        _back_left_drive_motor_id,
        _back_left_encoder_id,
        _back_left_encoder_offset,
        _back_left_x_pos,
        _back_left_y_pos,
        _invert_left_side,
        _back_left_steer_motor_inverted,
        _back_left_encoder_inverted,
    )
    back_right = _constants_creator.create_module_constants(
        _back_right_steer_motor_id,
        _back_right_drive_motor_id,
        _back_right_encoder_id,
        _back_right_encoder_offset,
        _back_right_x_pos,
        _back_right_y_pos,
        _invert_right_side,
        _back_right_steer_motor_inverted,
        _back_right_encoder_inverted,
    )

    @classmethod
    def create_drivetrain(cls) -> CommandSwerveDrivetrain:
        return CommandSwerveDrivetrain(
            hardware.TalonFX,
            hardware.TalonFX,
            hardware.CANcoder,
            cls.drivetrain_constants,
            [
                cls.front_left,
                cls.front_right,
                cls.back_left,
                cls.back_right,
            ],
        )
