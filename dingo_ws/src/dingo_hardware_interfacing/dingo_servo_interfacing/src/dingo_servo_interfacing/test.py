#!/usr/bin/env python3
# coding: utf-8
from adafruit_servokit import ServoKit
import time

# Configuración de la placa PCA9685
kit = ServoKit(channels=16)  # Asegúrate de que el número de canales sea correcto

# Función para mover un servo a un ángulo especificado
def move_servo(channel, angle):
    kit.servo[channel].angle = angle
    print(f"Servo {channel} movido a {angle} grados")

# Función para mover todos los servos
def test_all_servos():
    try:

        # Mover cada servo a 0 grados, luego a 90, y finalmente a 180 grados
        for angle in [0, 90, 180]:
            for i in range(4):  # Asumiendo que quieres mover 12 servos
                move_servo(i, angle)
                time.sleep(0.5)  # Espera medio segundo entre cada movimiento
            time.sleep(1)  # Espera un segundo después de mover todos los servos
    except KeyboardInterrupt:
        print("Prueba interrumpida por el usuario.")
        move_servo(0,0)
    except Exception as e:
        print(f"Ocurrió un error: {e}")

# Ejecutar la prueba de servos
test_all_servos()
