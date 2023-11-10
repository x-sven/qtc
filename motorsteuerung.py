from pymavlink import mavutil
import time
import re

######### PARAMETER #############################################
mode        = 'MANUAL'  # Modus für direkte Befehle
min_servo3  = 1100      # minimale Servoposition in Mikrosekunden
max_servo3  = 1900      # maximale Servoposition in Mikrosekunden
position_1  = 1500      # Erste Position in Mikrosekunden
position_2  = 1800      # Zweite Position in Mikrosekunden
update_rate = 10        # Hz Aktualisierungsrate
duration    = 3         # Sekunden Dauer einer Periode
#################################################################

### Funktion ### 
# zum Setzen der Position von Servo 3
def set_servo3(position_us = min_servo3):
    master.mav.rc_channels_override_send(
        master.target_system,  master.target_component,
        0,
        0,
        max(min_servo3, min(position_us, max_servo3)),
        0,
        0,
        0,
        0,
        0
    )

### MAIN ###
# # Verbindung zum Pixhawk herstellen (angepasst für "COM4" und 57600 Baudrate)
# master = mavutil.mavlink_connection('com4', baud=57600)
# print("Verbindung zum Pixhawk direkt hergestellt.")

# Verbindung zum Pixhawk über MAVProxy herstellen (angepasst für die IP-Adresse und Portnummer von MAVProxy)
master = mavutil.mavlink_connection('udp:127.0.0.1:14777')  # Beispiel: IP 127.0.0.1 und Port 14550
print("Verbindung zum Pixhawk über MAVProxy hergestellt.")

#einen Heartbeat abwarten, dann sind target_system und target_component gesetzt
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        custom_mode = msg.custom_mode
        break  # Breche die Schleife ab, nachdem der Modus angezeigt wurde

# Get mode ID
mode_id = master.mode_mapping()[mode]
#print("Mode ", mode, " has ID: ", mode_id)

# Ändere den Modus in MANUAL
master.mav.set_mode_send(
    master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
#print("MAV_MODE_MANUAL gesendet.")

# Warte auf eine Bestätigung, dass der Modus aktiviert ist oder eine Fehlermeldung
while True:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    #msg = master.recv_match(type='COMMAND_ACK', condition='MAV_CMD_DO_SET_MODE', blocking=True)
    if msg:
        if msg.result == 0:
            print("MANUAL-Modus bestätigt.")
        else:
            print(f"Fehler beim Setzen des Modus. Fehlercode: {msg.result}")
        break

print("Pixhawk ARM in 3 Sekunden ...")
time.sleep(3)

# https://mavlink.io/en/messages/common.html#COMMAND_LONG
# https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
master.mav.command_long_send(
    master.target_system, master.target_component, 
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
    0, #confirmation
    1, 0, # Arm, Force
    0, 0, 0, 0, 0) 

# Prüfung ob armed ...
# Warte auf eine Bestätigung, dass der Modus aktiviert ist oder eine Fehlermeldung
while True:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg:
        if msg.result == 0:
            print("Aircraft is ARMED.")
        else:
            print(f"Aircraft NOT ARMED. Fehler beim Setzen des Modus. Fehlercode: {msg.result}")
            exit(0)
        break
time.sleep(1)

# los geht's mit den Zyklen bis <Strg-C>
num_updates = int(update_rate * duration) # Berechnung der Anzahl der Updates
try:
    while True:
        print(f"Motorwert: {position_1}")
        for _ in range(num_updates):
            set_servo3(position_1)
            time.sleep(1.0 / update_rate) # Warten für die nächste Aktualisierung

        print(f"Motorwert: {position_2}")
        for _ in range(num_updates):
            set_servo3(position_2)
            time.sleep(1.0 / update_rate) # Warten für die nächste Aktualisierung

except KeyboardInterrupt:
    # Das Programm wird beendet, wenn du "Strg+C" drückst
    pass

# Motor auf Null (aus) setzen
set_servo3(min_servo3)
print("Motor ausgeschaltet.")

# DISARM
master.mav.command_long_send(
    master.target_system, master.target_component, 
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
    0, #confirmation
    0, 0, # Arm, Force
    0, 0, 0, 0, 0) 

# Warte auf eine Bestätigung, dass der Modus aktiviert ist oder eine Fehlermeldung
while True:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg:
        if msg.result == 0:
            print("Pixhawk DISARMED.")
        else:
            print(f"!!! Pixhawk STILL ARMED. Fehler beim Setzen des Modus. Fehlercode: {msg.result}")
            exit(0)
        break

# Verbindung beenden
master.close()
