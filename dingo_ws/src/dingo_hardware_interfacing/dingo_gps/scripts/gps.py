#!/usr/bin/env python
import rospy
from serial import Serial
from GPS.msg import GPS  

def gps_reader():
    rospy.init_node('gps_node', anonymous=True)
    pub = rospy.Publisher('dingo_gps_telemetry', GPS, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Configurar el puerto serial
    ser = Serial("/dev/ttyS0", 9600, timeout=1)  # Ajusta el puerto y el baudrate según tu configuración

    while not rospy.is_shutdown():
        line = ser.readline().decode('utf-8').strip()
        if line.startswith('$GPGGA'):  # Asumiendo que estamos usando el formato GGA para simplicidad
            data = line.split(',')
            gps_msg = GPS()
            gps_msg.latitude = float(data[2])  # Asegúrate de convertir y manejar los datos correctamente
            gps_msg.longitude = float(data[4])
            gps_msg.altitude = float(data[9])
            gps_msg.speed = 0  # Asumimos cero o calcula si es necesario
            gps_msg.satellites = int(data[7])
            pub.publish(gps_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        gps_reader()
    except rospy.ROSInterruptException:
        pass
