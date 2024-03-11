#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, MagneticField
from imu import MPU9150  # Asumiendo que imu.py contiene una clase llamada MPU9150

def prepare_imu(imu_data):
    #accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'imu_link'
    # Asignar valores de imu_data a imu_msg
    imu_msg.linear_acceleration.x = imu_data[0]
    imu_msg.linear_acceleration.y = imu_data[1]
    imu_msg.linear_acceleration.z = imu_data[2]
    imu_msg.angular_velocity.x = imu_data[3]
    imu_msg.angular_velocity.y = imu_data[4]
    imu_msg.angular_velocity.z = imu_data[5]
    return imu_msg

def prepare_mag(mag_data):
    mag_msg = MagneticField()
    mag_msg.header.stamp = rospy.Time.now()
    mag_msg.header.frame_id = 'mag_link'
    # Asignar valores de mag_data a mag_msg
    mag_msg.magnetic_field.x = mag_data[0]
    mag_msg.magnetic_field.y = mag_data[1]
    mag_msg.magnetic_field.z = mag_data[2]
    return mag_msg

def main():
    rospy.init_node('mpu9150_publisher_node')

    # Crea los publicadores para los tópicos de IMU y magnetómetro
    imu_pub = rospy.Publisher('/mpu91_imu', Imu, queue_size=10)
    mag_pub = rospy.Publisher('/mpu91_mag', MagneticField, queue_size=10)

    rate = rospy.Rate(200) 

    # Instancia de la clase MPU9150
    mpu9150 = MPU9150()

    count = 0

    while not rospy.is_shutdown():
        # Leer datos de la IMU
        imu_data = mpu9150.read_mpu9150_fast()
        # Crear y publicar mensaje IMU

        # Aquí se asignarían los campos de imu_msg basados en imu_data
        imu_pub.publish(prepare_imu(imu_data))

        # Cada 4 lecturas de la IMU, leer y publicar datos del magnetómetro
        if count % 4 == 0:
            mag_data = mpu9150.read_mag()
            # Aquí se asignarían los campos de mag_msg basados en mag_data
            mag_pub.publish(prepare_mag(mag_data))

        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
