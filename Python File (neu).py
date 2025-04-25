#Automatisches Nachführungssystem für ein Solarpanel, das durch Uhrzeiten und Lichtstärken Sensoren die Optimale Ausrichtung des Solarpanels findet
#und damit den Maximalen Energie Ertrag aus dem Solarpanel erzielt.
#Außerdem lässt sich das Solarpanel auch manuell verfahren dabei können Aktuelle Ertrags Werte und Umweltdaten wie
#Temperatur und Helligkeit in einem Dashboard dargestellt werden.

# Automatisches Nachführungssystem für ein Solarpanel ...

import machine
from machine import Pin, ADC, I2C
from time import sleep, ticks_ms, ticks_diff
import network
import onewire, ds18x20
from umqtt.simple import MQTTClient
import time
from ina226 import INA226

# WLAN-Konfiguration
WIFI_SSID = "FRITZ!Box 6590 Cable Bremer"
WIFI_PASSWORD = "Maunzi2216"

# MQTT-Konfiguration
MQTT_SERVER = "192.168.178.24"
MQTT_CLIENT_ID = "solar_tracker"
MQTT_TOPIC_COMMAND = b"solar/command"
MQTT_TOPIC_TEMPERATURE = b"solar/temperature"
MQTT_TOPIC_HUMIDITY = b"solar/humidity"
MQTT_TOPIC_LIMITS = b"solar/limits"
MQTT_TOPIC_VOLTAGE = b"solar/voltage"
MQTT_TOPIC_CURRENT = b"solar/current"
MQTT_TOPIC_WIND = b"solar/wind"
MQTT_TOPIC_LDR_LEFT = b"solar/ldr/left"
MQTT_TOPIC_LDR_RIGHT = b"solar/ldr/right"
MQTT_TOPIC_LDR_FRONT = b"solar/ldr/front"
MQTT_TOPIC_LDR_BACK = b"solar/ldr/back"
MQTT_TOPIC_AUTOMODE = b"solar/auto_control"

MAX_WIND_SPEED = 30.0  # km/h Grenze

last_sensor_time = ticks_ms()

# Temperatursensor DS18B20
ds_pin = Pin(41)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))
roms = ds_sensor.scan()

# LDRs
LDR_LEFT = ADC(2)
LDR_RIGHT = ADC(4)
LDR_FRONT = ADC(20)
LDR_BACK = ADC(19)

# BTS7960 Steuerung (RPWM/LPWM)
RPWM_X = Pin(14, Pin.OUT)
LPWM_X = Pin(12, Pin.OUT)
RPWM_Y = Pin(13, Pin.OUT)
LPWM_Y = Pin(11, Pin.OUT)

# Stromsensor
SCT_PIN = ADC(3)

# Wind-Sensor (Reedkontakt)
WIND_PIN = Pin(36, Pin.IN, Pin.PULL_UP)
wind_count = 0

def wind_callback(pin):
    global wind_count
    wind_count += 1

WIND_PIN.irq(trigger=Pin.IRQ_FALLING, handler=wind_callback)

def calculate_wind_speed(pulses, interval_s):
    rpm = pulses * (60 / interval_s)
    kmh = rpm * 2.4  # Kalibrierfaktor
    return kmh

# I2C für AHT10 (SDA 1, SCL 0)
i2c = machine.I2C(scl=Pin(1), sda=Pin(0), freq=100000)
AHT10_ADDR = 0x38

# AHT10 initialisieren
i2c.writeto(AHT10_ADDR, bytearray([0xE1, 0x08, 0x00]))
sleep(0.05)

# INA226 Setup
ina226 = INA226(i2c, 0x40)

# WLAN verbinden
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if not wlan.isconnected():
    print(f"Verbinde mit WLAN {WIFI_SSID}...")
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    timeout = 10
    while not wlan.isconnected() and timeout > 0:
        sleep(1)
        timeout -= 1
        print(".")
    if wlan.isconnected():
        print("WLAN verbunden, IP-Adresse:", wlan.ifconfig()[0])
    else:
        print("WLAN-Verbindung fehlgeschlagen!")

# MQTT
client = MQTTClient(MQTT_CLIENT_ID, MQTT_SERVER)
auto_mode = True
wind_speed = 0.0

def move_actuator(r_pwm, l_pwm, direction, duration=0.5):
    r_pwm.value(0)
    l_pwm.value(0)
    sleep(0.1)

    if direction == "extend":
        r_pwm.value(1)
        l_pwm.value(0)
    elif direction == "retract":
        r_pwm.value(0)
        l_pwm.value(1)
    else:
        r_pwm.value(0)
        l_pwm.value(0)

    sleep(duration)
    r_pwm.value(0)
    l_pwm.value(0)

def mqtt_callback(topic, msg):
    global auto_mode
    print(f"Nachricht erhalten: {topic.decode()} - {msg.decode()}")

    if topic == MQTT_TOPIC_AUTOMODE:
        if msg.decode().lower() == "on":
            auto_mode = True
            print("Automatik EIN")
        elif msg.decode().lower() == "off":
            auto_mode = False
            print("Automatik AUS")

    if topic == b"solar/command/move/x/left":
        move_actuator(RPWM_X, LPWM_X, "extend", duration=10)
    elif topic == b"solar/command/move/x/right":
        move_actuator(RPWM_X, LPWM_X, "retract", duration=10)
    elif topic == b"solar/command/move/y/up":
        move_actuator(RPWM_Y, LPWM_Y, "extend", duration=10)
    elif topic == b"solar/command/move/y/down":
        move_actuator(RPWM_Y, LPWM_Y, "retract", duration=10)

client.set_callback(mqtt_callback)
client.connect()
client.subscribe(MQTT_TOPIC_COMMAND + b"/#")
client.subscribe(MQTT_TOPIC_AUTOMODE)
print("MQTT verbunden.")

# Funktion zum Auslesen von Temperatur und Feuchtigkeit (AHT10)
def read_aht10():
    try:
        i2c.writeto(AHT10_ADDR, b'\xAC\x33\x00')
        sleep(0.1)
        d = i2c.readfrom(AHT10_ADDR, 6)

        if d[0] & 0x08 != 0x08:
            print("Sensor nicht bereit")
            return

        rf = ((d[1] << 16) | (d[2] << 8) | d[3]) >> 4
        rt = ((d[3] & 0x0F) << 16) | (d[4] << 8) | d[5]

        feuchte = round((rf / 1048576) * 100, 1)
        temp = round((rt / 1048576) * 200 - 50, 1)

        print("Temp:", temp, "°C  Feuchte:", feuchte, "%")
        client.publish(MQTT_TOPIC_TEMPERATURE, str(temp))
        client.publish(MQTT_TOPIC_HUMIDITY, str(feuchte))
    except:
        print("Fehler beim Auslesen AHT10")

# Hauptschleife
while True:
    client.check_msg()

    current_time = ticks_ms()
    if ticks_diff(current_time, last_sensor_time) >= 10_000:
        for rom in roms:
            temp = ds_sensor.read_temp(rom)
            print(f"Temperatur DS18B20: {temp:.2f}°C")
            client.publish(MQTT_TOPIC_TEMPERATURE, f"{temp:.2f}")

        try:
            voltage = ina226.bus_voltage         # Spannung in Volt
            current_ina = ina226.current / 1000  # Strom in Ampere (mA / 1000)
            power = ina226.power / 1000         # Leistung in Watt (mW / 1000)

            print(f"INA226 - Spannung: {voltage:.2f} V, Strom: {current_ina:.2f} A, Leistung: {power:.2f} W")
            client.publish(MQTT_TOPIC_VOLTAGE, f"{voltage:.2f}")
            client.publish(MQTT_TOPIC_CURRENT, f"{current_ina:.2f}")
            client.publish(MQTT_TOPIC_LIMITS, f"{power:.2f}")
        except Exception as e:
            print("Fehler beim INA226:", e)

        wind_speed = calculate_wind_speed(wind_count, 10)
        print(f"Windgeschwindigkeit: {wind_speed:.2f} km/h")
        client.publish(MQTT_TOPIC_WIND, f"{wind_speed:.2f}")
        wind_count = 0

        read_aht10()
        last_sensor_time = current_time

    l_left = LDR_LEFT.read_u16()
    l_right = LDR_RIGHT.read_u16()
    l_front = LDR_FRONT.read_u16()
    l_back = LDR_BACK.read_u16()

    client.publish(MQTT_TOPIC_LDR_LEFT, str(l_left))
    client.publish(MQTT_TOPIC_LDR_RIGHT, str(l_right))
    client.publish(MQTT_TOPIC_LDR_FRONT, str(l_front))
    client.publish(MQTT_TOPIC_LDR_BACK, str(l_back))

    if auto_mode and wind_speed < MAX_WIND_SPEED:
        if l_left > l_right + 500:
            move_actuator(RPWM_X, LPWM_X, "extend", duration=2)
        elif l_right > l_left + 500:
            move_actuator(RPWM_X, LPWM_X, "retract", duration=2)

        if l_front > l_back + 500:
            move_actuator(RPWM_Y, LPWM_Y, "extend", duration=2)
        elif l_back > l_front + 500:
            move_actuator(RPWM_Y, LPWM_Y, "retract", duration=2)

    sleep(0.2)
