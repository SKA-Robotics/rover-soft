#!/usr/bin/python3
import rospy

from canbus_interface import CanbusInterface

from can_msgs.msg import Frame
from canbus_modules.msg import PowerStatus


class PowerMeasurementCanbus(CanbusInterface):
    def __init__(self) -> None:
        super().__init__(rospy.get_param("~device_id", 0x30))

        rospy.init_node("power_measurement_canbus")

        self.status_topic = rospy.get_param("~status_topic", "/power_status")

        self.resolution = rospy.get_param("~resolution", 18)
        self.voltage_multiplier = 2 * 2.048 / (1 << self.resolution)

        self.status_publisher = rospy.Publisher(
            self.status_topic, PowerStatus, queue_size=10
        )

        self.readings = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]

    def run(self) -> None:
        rospy.spin()

    def receive_frame(self, command_id, data, frame: Frame):
        if command_id == 0:
            self.readings[0:2] = [
                (data[index] << 24)
                | (data[index + 1] << 16)
                | (data[index + 2] << 8)
                | data[index + 3]
                for index in range(0, 8, 4)
            ]
        if command_id == 1:
            self.readings[2:4] = [
                (data[index] << 24)
                | (data[index + 1] << 16)
                | (data[index + 2] << 8)
                | data[index + 3]
                for index in range(0, 8, 4)
            ]

        adc_voltages = [
            (reading if (reading & (1 << 31)) == 0 else (reading - (1 << 32)))
            * self.voltage_multiplier
            for reading in self.readings
        ]

        battery_voltage = 18.0 + (adc_voltages[1] * 6.10)
        battery_percentage = 100.0  # todo once we know the battery charge curve

        motor_current = ((adc_voltages[0] * 2.5) - 2.5) / 0.066
        computer_current = ((adc_voltages[2] * 2.5) - 2.5) / 0.185
        peripheral_current = ((adc_voltages[3] * 2.5) - 2.5) / 0.185

        total_current = motor_current + computer_current + peripheral_current
        total_power = battery_voltage * total_current

        self.status_publisher.publish(
            PowerStatus(
                battery_voltage=battery_voltage,
                battery_percentage=battery_percentage,
                total_current=total_current,
                motor_current=motor_current,
                computer_current=computer_current,
                peripheral_current=peripheral_current,
                total_power=total_power,
            )
        )


if __name__ == "__main__":
    try:
        PowerMeasurementCanbus().run()
    except rospy.ROSInterruptException:
        pass
