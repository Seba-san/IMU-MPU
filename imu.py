#!/usr/bin/env python3
# asegúrate de que este archivo es ejecutable: chmod +x publish_imu.py

#import rospy
#from sensor_msgs.msg import Imu

import smbus
import time





class MPU9150:
    def __init__(self) -> None:
        self.bus= smbus.SMBus(1)  # 1 indica el bus I2C-1, cambia si es necesario
        self.setup()

    def setup(self):
        """Inicializa el MPU9150."""
        # Configuraciones del MPU9150
        self.MPU9150_I2C_ADDR = 0x68  # Dirección I2C del MPU9150 (puede ser 0x68 o 0x69)
        self.PWR_MGMT_1 = 0x6B
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H = 0x43
        self.GYRO_YOUT_H = 0x45
        self.GYRO_ZOUT_H = 0x47
        self.address=0x68
        self.gravedad=9.80665



        self.bus.write_byte_data(self.MPU9150_I2C_ADDR, self.PWR_MGMT_1, 0)
        self.init_magnetometer()

    def convert_to_float(self, high, low, scale):
        # Convierte los valores de alto y bajo a un valor de punto flotante, escalado según el parámetro scale
        value = (high << 8) | low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value / scale

    def read_i2c_block(self, reg, length=14):
        # Leer un bloque de datos
        block = self.bus.read_i2c_block_data(self.address, reg, length)
        return block

    def read_mpu9150(self):
        """Lee los datos de aceleración y giroscopio del MPU9150."""

        accel_x = self.read_i2c_word(self.ACCEL_XOUT_H)
        accel_y = self.read_i2c_word(self.ACCEL_YOUT_H)
        accel_z = self.read_i2c_word(self.ACCEL_ZOUT_H)

        gyro_x = self.read_i2c_word(self.GYRO_XOUT_H)
        gyro_y = self.read_i2c_word(self.GYRO_YOUT_H)
        gyro_z = self.read_i2c_word(self.GYRO_ZOUT_H)

        #mag_x, mag_y, mag_z = self.get_magnet_data()

        #print(f"Accel (X, Y, Z): ({accel_x}, {accel_y}, {accel_z}) Gyro (X, Y, Z): ({gyro_x}, {gyro_y}, {gyro_z})")
        #print(f"Accel (X, Y, Z): ({accel_x}, {accel_y}, {accel_z}) mag (X, Y, Z): ({mag_x}, {mag_y}, {mag_z})")

    def read_mpu9150_fast(self):
        block = self.read_i2c_block(0x3B)

        # Convertir los bytes a valores flotantes
        accel_x = self.convert_to_float(block[0], block[1], 16384.0)*self.gravedad
        accel_y = self.convert_to_float(block[2], block[3], 16384.0)*self.gravedad
        accel_z = self.convert_to_float(block[4], block[5], 16384.0)*self.gravedad

        temp= self.convert_to_float(block[6], block[7], 340.0)+ 36.53
        
        gyro_x = self.convert_to_float(block[8], block[9], 131.0)
        gyro_y = self.convert_to_float(block[10], block[11], 131.0)
        gyro_z = self.convert_to_float(block[12], block[13], 131.0)

        # tarda mucho esto, ver como paralelizarlo, ademas hay que transformarlo en cuaternion.

        #mag_x,mag_y,mag_z=self.read_mag()


        #return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp, mag_x, mag_y, mag_z
        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp

    def read_mag(self):
        AK8975_CNTL = 0x0A
        AK8975_ADDRESS = 0x0C
        AK8975_HXL = 0x03
        # Iniciar una medición del magnetómetro
        self.bus.write_byte_data(AK8975_ADDRESS, AK8975_CNTL, 0x01)
        # Establecer un tiempo de espera (timeout) en segundos
        timeout = 2  # Por ejemplo, 2 segundos
        start_time = time.time()
         # Esperar a que la medición esté completa o hasta que se supere el tiempo de espera
        while True:
            if (time.time() - start_time) > timeout:
                print("Timeout: La medición del magnetómetro ha tardado demasiado.")
                break  # Salir del bucle si se supera el tiempo de espera
            
            status = self.bus.read_byte_data(AK8975_ADDRESS, AK8975_CNTL)
            if (status & 0x01) == 0:
                break  # Salir del bucle si la medición está completa
        # Esperar a que la medición esté completa
        # while True:
            # status = self.bus.read_byte_data(AK8975_ADDRESS, AK8975_CNTL)
            # if (status & 0x01) == 0:
                # break
            
        # Leer los datos del magnetómetro
        mag_x = self.bus.read_byte_data(AK8975_ADDRESS, AK8975_HXL) | self.bus.read_byte_data(AK8975_ADDRESS, AK8975_HXL+1) << 8
        mag_y = self.bus.read_byte_data(AK8975_ADDRESS, AK8975_HXL+2) | self.bus.read_byte_data(AK8975_ADDRESS, AK8975_HXL+3) << 8
        mag_z = self.bus.read_byte_data(AK8975_ADDRESS, AK8975_HXL+4) | self.bus.read_byte_data(AK8975_ADDRESS, AK8975_HXL+5) << 8
        return mag_x, mag_y, mag_z

    def read_i2c_word(self, register, device=None):
        """Lee y devuelve un valor de palabra (2 bytes) desde un registro dado."""
        if device is None:
            device = self.MPU9150_I2C_ADDR
        
        high = self.bus.read_byte_data(device, register)
        low = self.bus.read_byte_data(device, register + 1)
        value = (high << 8) + low
        
        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value
        
    def init_magnetometer(self):
        # Configurar el MPU-9150 para habilitar el acceso al magnetómetro
        # Direcciones I2C relevantes
        MPU9150_ADDRESS = 0x68
        AK8975_ADDRESS = 0x0C
        AK8975_ASAX = 0x10
        MPU9150_ADDRESS = 0x68
        AK8975_HXL = 0x03

        # Registros AK8975

       

        # Habilitar el I2C Master Mode del MPU9150
        self.bus.write_byte_data(MPU9150_ADDRESS, 0x6A, 0x00) # Deshabilitar I2C Master
        self.bus.write_byte_data(MPU9150_ADDRESS, 0x24, 0x0D) # Configurar el control de I2C Master
        self.bus.write_byte_data(MPU9150_ADDRESS, 0x25, AK8975_ADDRESS | 0x80) # Configurar la dirección del esclavo AK8975 para lectura
        self.bus.write_byte_data(MPU9150_ADDRESS, 0x26, AK8975_HXL) # Configurar el registro desde el cual iniciar la lectura
        self.bus.write_byte_data(MPU9150_ADDRESS, 0x27, 0x88) # Leer 8 bytes y habilitar
        self.bus.write_byte_data(MPU9150_ADDRESS, 0x6A, 0x20) # Habilitar el I2C Master Mode

        self.bus.write_byte_data(self.MPU9150_I2C_ADDR, 0x37, 0x02) # Habilitar passthrough mode
        self.bus.write_byte_data(0x0C, 0x0A, 0x01) # Habilitar el magnetómetro
    
    def get_magnet_data(self):
        # Leer los datos del magnetómetro
        # 12 = 0X0C
        mag_x = self.read_i2c_word(0x03, device=0x0C)
        mag_y = self.read_i2c_word(0x05, device=0x0C)
        mag_z = self.read_i2c_word(0x07, device=0x0C)

        return  mag_x, mag_y, mag_z

def publish_imu_data():
    #rospy.init_node('imu_publisher_node', anonymous=True)
    #pub = rospy.Publisher('/imu/mpu9150', Imu, queue_size=10)
    #rate = rospy.Rate(10)  # 10Hz
    mpu=MPU9150()


    #while not rospy.is_shutdown():
    t=time.time()
    count=0
    while True:
        while (time.time()-t)<1:                    
            data=mpu.read_mpu9150_fast()
            count=count+1
            if count%4==0: #ratio imu (mag) ,1 is 73hz (73hz), 2 is 130hz (65hz); 4 is 200hz (50hz), 8 is 280hz (35hz)
                mag=mpu.read_mag()

        print(f"count: {count}")
        print(f"Accel (X, Y, Z): ({data[0]:.3f}, {data[1]:.3f}, {data[2]:.3f}) Gyro (X, Y, Z): ({data[3]:.3f}, {data[4]:.3f}, {data[5]:.3f}), Mag (X,Y,Z):({mag[0]:.3f},{mag[1]:.3f},{mag[2]:.3f} ), Temp: {data[6]:.3f}")
        #print(f"Accel (X, Y, Z): ({data[0]}, {data[1]}, {data[2]}) Gyro (X, Y, Z): ({data[3]}, {data[4]}, {data[5]}) Temp: {data[6]} Mag (X, Y, Z): ({data[7]}, {data[8]}, {data[9]})")
        count=0
        t=time.time()
        #imu_msg = Imu()
#
        ## Aquí deberías rellenar imu_msg con datos reales de tu IMU
        ## Para este ejemplo, generaremos datos aleatorios
        #imu_msg.header.stamp = rospy.Time.now()
        #imu_msg.header.frame_id = "imu_link"
        #imu_msg.orientation.x = 0.0
        #imu_msg.orientation.y = 0.0
        #imu_msg.orientation.z = 0.0
        #imu_msg.orientation.w = 0.0
#
        ## Publica el mensaje
        #pub.publish(imu_msg)
        #rate.sleep()

if __name__ == '__main__':
    publish_imu_data()
    #try:
    #    publish_imu_data()
    #except rospy.ROSInterruptException:
    #    pass
