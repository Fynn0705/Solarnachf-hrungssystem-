#Automatisches Nachführungssystem für ein Solarpanel, das durch Lichtstärken Sensoren die Optimale Ausrichtung des Solarpanels findet
#Damit den Maximalen Energie Ertrag aus dem Solarpanel erzielt.
#Solarpanel lässt sich auch manuell verfahren.
#Bei einer zu hohen Windgeschwindigkeit fährt das Solarpanel in Schutzstellung.
#Temperatur und Helligkeit sowie Strom, Spannung und Leistung werden in einem Dashboard dargestellt.
#Ersteller Fynn Bremer
#letztes Update 04.05.2025

import machine
from machine import Pin, ADC, I2C
from time import sleep, ticks_ms, ticks_diff
import network
import onewire, ds18x20
from umqtt.simple import MQTTClient
from ina226 import INA226

# WLAN-Konfiguration
WLAN_NAME = "FRITZ!Box 6590 Cable Bremer"       # SSID des WLANs
WLAN_PASSWORT = "Maunzi2216"                    # Passwort für das WLAN

# MQTT-Konfiguration
MQTT_SERVER = "192.168.178.24"                  # Adresse des MQTT-Brokers
MQTT_CLIENT_ID = "solar_tracker"                # Eindeutige Gerätekennung

# MQTT-Themen (Topics)
MQTT_TOPIC_BEFEHL = b"solar/befehl"             # Steuerbefehle
MQTT_TOPIC_TEMPERATUR = b"solar/temperatur"     # Temperaturdaten
MQTT_TOPIC_LUFTFEUCHTE = b"solar/luftfeuchte"   # Luftfeuchte vom AHT10
MQTT_TOPIC_LEISTUNG = b"solar/leistung"         # Elektrische Leistung
MQTT_TOPIC_SPANNUNG = b"solar/spannung"         # Spannung (INA226)
MQTT_TOPIC_STROM = b"solar/strom"               # Strom (INA226)
MQTT_TOPIC_WIND = b"solar/wind"                 # Windgeschwindigkeit
MQTT_TOPIC_LDR_LINKS = b"solar/ldr/links"       # Lichtsensor links
MQTT_TOPIC_LDR_RECHTS = b"solar/ldr/rechts"     # Lichtsensor rechts
MQTT_TOPIC_LDR_VORNE = b"solar/ldr/vorne"       # Lichtsensor vorne
MQTT_TOPIC_LDR_HINTEN = b"solar/ldr/hinten"     # Lichtsensor hinten
MQTT_TOPIC_AUTOMATIK = b"solar/automatik"       # Automatikbetrieb ein/aus
MQTT_TOPIC_WIND_ALARM = b"solar/wind/alarm"     # Windalarmstatus
MQTT_TOPIC_WIND_RESET = b"solar/wind/zuruecksetzen" # Rücksetzen des Windalarms
MQTT_TOPIC_WIND_MAX = b"solar/wind/maximal"     # Neue Grenzwerte setzen
MQTT_TOPIC_LDR_LINKS_KAL = b"solar/ldr/links/kalibrierung" # LDR-Korrekturfaktor

# Wind- und Kalibriergrenzwerte
maximale_windgeschwindigkeit = 10.0             # obere Grenze für Automatikbetrieb
WIND_ALARM_GRENZE = 15.0                         # Windgeschwindigkeit für Alarm
LDR_LINKS_KALIBRIERUNG = 0.95                    # Korrekturwert für linken LDR

# Zeitverfolgung für Sensorintervall
letzte_sensorzeit = ticks_ms()

# Initialisierung des DS18B20 Temperatursensors
ds_pin = Pin(41)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))
roms = ds_sensor.scan()

# LDR-Sensoren an ADC-Pins
LDR_LINKS = ADC(2)
LDR_RECHTS = ADC(4)
LDR_VORNE = ADC(18)
LDR_HINTEN = ADC(19)

# Steuerung für Aktuatoren (BTS7960 H-Brücke)
RPWM_X = Pin(14, Pin.OUT)
LPWM_X = Pin(12, Pin.OUT)
RPWM_Y = Pin(13, Pin.OUT)
LPWM_Y = Pin(11, Pin.OUT)

# Windsensor – Impulse zählen
WIND_PIN = Pin(21, Pin.IN, Pin.PULL_UP)
wind_impulse = 0
letzte_windzeit = 0

# IRQ-Handler für Windimpulse
def wind_callback(pin):
    global wind_impulse, letzte_windzeit
    jetzt = ticks_ms()
    if ticks_diff(jetzt, letzte_windzeit) > 50:  # Entprellung
        wind_impulse += 1
        letzte_windzeit = jetzt

WIND_PIN.irq(trigger=Pin.IRQ_FALLING, handler=wind_callback)

# Berechnung der Windgeschwindigkeit aus Impulsen
def berechne_windgeschwindigkeit(impulse, intervall_s):
    rpm = impulse * (30 / intervall_s)
    kmh = rpm * 0.1
    return kmh

# I2C-Bus & Sensoren initialisieren
i2c = machine.I2C(scl=Pin(1), sda=Pin(0), freq=100000)
AHT10_ADRESSE = 0x38
i2c.writeto(AHT10_ADRESSE, bytearray([0xE1, 0x08, 0x00]))  # Initialisieren
sleep(0.05)
ina226 = INA226(i2c, 0x40)  # Strom/Spannungssensor INA226

# WLAN-Verbindung aufbauen
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if not wlan.isconnected():
    print(f"Verbinde mit WLAN {WLAN_NAME}...")
    wlan.connect(WLAN_NAME, WLAN_PASSWORT)
    timeout = 10
    while not wlan.isconnected() and timeout > 0:
        sleep(1)
        timeout -= 1
        print(".")
    if wlan.isconnected():
        print("WLAN verbunden, IP-Adresse:", wlan.ifconfig()[0])
    else:
        print("WLAN fehlgeschlagen!")
        machine.reset()

# MQTT-Verbindung und Abonnements
def mqtt_verbinden():
    global client
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_SERVER)
    client.set_callback(mqtt_callback)
    client.connect()
    client.subscribe(MQTT_TOPIC_BEFEHL + b"/#")
    client.subscribe(MQTT_TOPIC_AUTOMATIK)
    client.subscribe(MQTT_TOPIC_WIND_RESET)
    client.subscribe(MQTT_TOPIC_WIND_MAX)
    client.subscribe(MQTT_TOPIC_LDR_LINKS_KAL)
    print(" MQTT verbunden.")

automatik_aktiv = True
wind_aktiv = False
windgeschwindigkeit = 0.0

# Steuerung der Motoren über Richtungsparameter
def bewege_aktor(r_pwm, l_pwm, richtung, dauer=0.5):
    r_pwm.value(0)
    l_pwm.value(0)
    sleep(0.1)
    if richtung == "ausfahren":
        r_pwm.value(1)
        l_pwm.value(0)
    elif richtung == "einfahren":
        r_pwm.value(0)
        l_pwm.value(1)
    sleep(dauer)
    r_pwm.value(0)
    l_pwm.value(0)

# MQTT-Ereignisbehandlung
def mqtt_callback(topic, msg):
    global automatik_aktiv, wind_aktiv, maximale_windgeschwindigkeit
    print(f" Nachricht: {topic.decode()} - {msg.decode()}")

    if topic == MQTT_TOPIC_AUTOMATIK:
        automatik_aktiv = msg.decode().lower() == "on"
        print("Automatik:", "EIN" if automatik_aktiv else "AUS")

    if topic == b"solar/befehl/bewege/x/links":
        bewege_aktor(RPWM_X, LPWM_X, "ausfahren", 10)
    elif topic == b"solar/befehl/bewege/x/rechts":
        bewege_aktor(RPWM_X, LPWM_X, "einfahren", 10)
    elif topic == b"solar/befehl/bewege/y/hoch":
        bewege_aktor(RPWM_Y, LPWM_Y, "ausfahren", 10)
    elif topic == b"solar/befehl/bewege/y/runter":
        bewege_aktor(RPWM_Y, LPWM_Y, "einfahren", 10)

    if topic == MQTT_TOPIC_WIND_RESET and wind_aktiv:
        print("Wind-Reset empfangen - Automatik reaktiviert.")
        automatik_aktiv = True
        wind_aktiv = False
        client.publish(MQTT_TOPIC_WIND_ALARM, b"OK")

    if topic == MQTT_TOPIC_WIND_MAX:
        try:
            neue_grenze = float(msg.decode())
            maximale_windgeschwindigkeit = neue_grenze
            print(f"Neue max. Windgeschwindigkeit: {maximale_windgeschwindigkeit} km/h")
        except ValueError:
            print("Ungültiger Wert für Windgrenze:", msg)

    if topic == MQTT_TOPIC_LDR_LINKS_KAL:
        try:
            neuer_faktor = float(msg.decode())
            if 0.5 <= neuer_faktor <= 1.5:
                global LDR_LINKS_KALIBRIERUNG
                LDR_LINKS_KALIBRIERUNG = neuer_faktor
                print(f" Kalibrierung LDR links: {LDR_LINKS_KALIBRIERUNG:.2f}")
            else:
                print(" Kalibrierungswert außerhalb von 0.5–1.5!")
        except ValueError:
            print(" Ungültiger Kalibrierungswert:", msg)

mqtt_verbinden()

# Temperatur und Luftfeuchte vom AHT10 auslesen
def lese_aht10():
    try:
        i2c.writeto(AHT10_ADRESSE, b'\xAC\x33\x00')
        sleep(0.1)
        daten = i2c.readfrom(AHT10_ADRESSE, 6)

        if daten[0] & 0x08 != 0x08:
            print("Sensor nicht bereit")
            return

        roh_feuchte = ((daten[1] << 16) | (daten[2] << 8) | daten[3]) >> 4
        roh_temp = ((daten[3] & 0x0F) << 16) | (daten[4] << 8) | daten[5]

        feuchte = round((roh_feuchte / 1048576) * 100, 1)
        temperatur = round((roh_temp / 1048576) * 200 - 50, 1)

        print("️ Temperatur:", temperatur, "°C  Feuchte:", feuchte, "%")
        client.publish(MQTT_TOPIC_TEMPERATUR, str(temperatur))
        client.publish(MQTT_TOPIC_LUFTFEUCHTE, str(feuchte))
    except Exception as e:
        print("Fehler AHT10:", e)

# Hauptschleife: Sensoren lesen, MQTT verarbeiten, Motoren steuern
while True:
    if not wlan.isconnected():
        print(" WLAN getrennt – Neustart...")
        sleep(5)
        machine.reset()

    try:
        client.check_msg()  # Neue MQTT-Nachrichten verarbeiten
    except Exception as e:
        print(" MQTT-Fehler:", e)
        sleep(5)
        machine.reset()

    jetzt = ticks_ms()

    if ticks_diff(jetzt, letzte_sensorzeit) >= 20_000:
        # Temperatur DS18B20
        for rom in roms:
            temperatur = ds_sensor.read_temp(rom)
            print(f" DS18B20: {temperatur:.2f} °C")
            client.publish(MQTT_TOPIC_TEMPERATUR, f"{temperatur:.2f}")

        # INA226 auslesen
        try:
            spannung = ina226.bus_voltage
            shunt_spannung = ina226.shunt_voltage
            shunt_widerstand = 0.002
            strom = shunt_spannung / shunt_widerstand
            leistung = ina226.power
            print(f" Spannung: {spannung:.2f} V, Strom: {strom:.2f} A, Leistung: {leistung:.2f} W")
            client.publish(MQTT_TOPIC_SPANNUNG, f"{spannung:.2f}")
            client.publish(MQTT_TOPIC_STROM, f"{strom:.2f}")
            client.publish(MQTT_TOPIC_LEISTUNG, f"{leistung:.2f}")
        except Exception as e:
            print("Fehler INA226:", e)

        # Windgeschwindigkeit berechnen
        windgeschwindigkeit = berechne_windgeschwindigkeit(wind_impulse, 10)
        print(f" Windgeschwindigkeit: {windgeschwindigkeit:.2f} km/h")
        client.publish(MQTT_TOPIC_WIND, f"{windgeschwindigkeit:.2f}")

        # Windalarm auslösen
        if windgeschwindigkeit > WIND_ALARM_GRENZE:
            print("STURMALARM! Wind zu stark:", windgeschwindigkeit)
            client.publish(MQTT_TOPIC_WIND_ALARM, f"ALARM: {windgeschwindigkeit:.2f} km/h")
            automatik_aktiv = False
            wind_aktiv = True
            bewege_aktor(RPWM_X, LPWM_X, "einfahren", 25)
            bewege_aktor(RPWM_Y, LPWM_Y, "einfahren", 25)

        wind_impulse = 0
        lese_aht10()
        letzte_sensorzeit = jetzt
        client.publish(MQTT_TOPIC_WIND_ALARM, b"ALARM" if wind_aktiv else b"OK")

    # Lichtsensoren auslesen und senden
    l_links = LDR_LINKS.read_u16() * LDR_LINKS_KALIBRIERUNG
    l_rechts = LDR_RECHTS.read_u16()
    l_vorne = LDR_VORNE.read_u16()
    l_hinten = LDR_HINTEN.read_u16()

    client.publish(MQTT_TOPIC_LDR_LINKS, str(l_links))
    client.publish(MQTT_TOPIC_LDR_RECHTS, str(l_rechts))
    client.publish(MQTT_TOPIC_LDR_VORNE, str(l_vorne))
    client.publish(MQTT_TOPIC_LDR_HINTEN, str(l_hinten))

    # Automatische Nachführung aktivieren, falls erlaubt
    if automatik_aktiv and windgeschwindigkeit < maximale_windgeschwindigkeit:
        if l_links > l_rechts + 500:
            bewege_aktor(RPWM_X, LPWM_X, "ausfahren", 2)
        elif l_rechts > l_links + 500:
            bewege_aktor(RPWM_X, LPWM_X, "einfahren", 2)

        if l_vorne > l_hinten + 500:
            bewege_aktor(RPWM_Y, LPWM_Y, "ausfahren", 2)
        elif l_hinten > l_vorne + 500:
            bewege_aktor(RPWM_Y, LPWM_Y, "einfahren", 2)

    sleep(0.2)
