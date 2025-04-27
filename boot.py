import machine
import network
import time

# WLAN einschalten (optional)
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

# Du kannst hier noch ein paar Sekunden warten lassen, falls nötig
time.sleep(1)

# Danach übernimmt automatisch main.py
print("Boot abgeschlossen, Starte Hauptprogramm...")