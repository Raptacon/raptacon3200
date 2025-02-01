import rev


def configureSparkMaxCanRates(
    motor: rev.SparkMax,
    faultRateMs=50,
    motorTelmRateMs=50,
    motorPosRateMs=50,
    analogRateMs=1833,
    altEncoderRateMs=1050,
    dutyCycleEncRateMs=2150,
    dutyCycleEncVelRateMs=3150,
):

    # Some of the configurations prior to 2025 are no longer available
    # according to https://www.chiefdelphi.com/t/rev-spark-max-migration-to-2025/479555/4
    # the idea is that a get method will request via CAN vs it always just sending on the bus
    config = rev.SparkMaxConfig()
    config.signals.faultsPeriodMs(faultRateMs)
    # config.signals.motorTelmRateMs(motorTelmRateMs)
    # config.signals.motorPosRateMs(motorPosRateMs)
    # config.signals.analogRateMs(analogRateMs)
    # config.signals.altEncoderRateMs(altEncoderRateMs)
    # config.signals.dutyCycleEncRateMs(dutyCycleEncRateMs)
    # config.signals.dutyCycleEncVelRateMs(dutyCycleEncVelRateMs)

    # motor.configure(config, rev.kResetSafeParameters, rev.kPersistParameters)
    motor.configure(
        config,
        rev.SparkBase.ResetMode.kResetSafeParameters,
        rev.SparkBase.PersistMode.kPersistParameters,
    )

    # motor.setPeriodicFramePeriod(rev.SparkLowLevel.PeriodicFrame.kStatus0, faultRateMs)
    # motor.setPeriodicFramePeriod(rev.SparkLowLevel.PeriodicFrame.kStatus1, motorTelmRateMs)
    # motor.setPeriodicFramePeriod(rev.SparkLowLevel.PeriodicFrame.kStatus2, motorPosRateMs)
    # motor.setPeriodicFramePeriod(rev.SparkLowLevel.PeriodicFrame.kStatus3, analogRateMs)
    # motor.setPeriodicFramePeriod(rev.SparkLowLevel.PeriodicFrame.kStatus4, altEncoderRateMs)
    # motor.setPeriodicFramePeriod(rev.SparkLowLevel.PeriodicFrame.kStatus5, dutyCycleEncRateMs)
    # motor.setPeriodicFramePeriod(rev.SparkLowLevel.PeriodicFrame.kStatus6, dutyCycleEncVelRateMs)
    # motor.setPeriodicFramePeriod(rev.SparkLowLevel.PeriodicFrame.kStatus7, 500) #Unknown frame type? default 250ms prob not important?
