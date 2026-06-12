# STEELHAUS — Heat Treat Oven Controller
### Complete Build and Setup Guide

STEELHAUS is a PID temperature controller for knife-making heat treat ovens. It runs on a Raspberry Pi 4 with a touchscreen display. An ESP32 microcontroller handles the thermocouple reading and controls the solid-state relays that switch power to the heating element. A web interface runs on the touchscreen and is also accessible from your phone or laptop over your local network or from anywhere via Tailscale.

This guide walks through the entire build from buying parts to a running system.

---

## Table of Contents

1. [What You Are Building](#1-what-you-are-building)
2. [Parts List](#2-parts-list)
3. [240V Mains Wiring — Safety Warning](#3-240v-mains-wiring--safety-warning)
4. [Low-Voltage Wiring Summary](#4-low-voltage-wiring-summary)
5. [Set Up the ESP32](#5-set-up-the-esp32)
6. [Set Up the Raspberry Pi](#6-set-up-the-raspberry-pi)
7. [Install the Controller Software](#7-install-the-controller-software)
8. [First Boot and Test](#8-first-boot-and-test)
9. [Set Up Remote Access with Tailscale](#9-set-up-remote-access-with-tailscale)
10. [First Use — PID Autotune](#10-first-use--pid-autotune)
11. [Built-in Heat Treat Programs](#11-built-in-heat-treat-programs)
12. [Troubleshooting](#12-troubleshooting)
13. [File Reference](#13-file-reference)

---

## 1. What You Are Building

The controller has two main boards:

**Raspberry Pi 4** — A small single-board computer that runs the web interface, the PID control algorithm, and stores all your programs and temperature logs. It connects to the ESP32 over a serial cable (two wires).

**ESP32** — A small microcontroller board that handles the real-time hardware side: reading the thermocouple temperature sensor 4 times per second, switching the solid-state relays on and off based on commands from the Pi, and cutting power immediately if any fault is detected (door open, communication lost, thermocouple fault).

The Pi and the ESP32 talk to each other over a direct serial connection — two wires soldered or jumpered between them. They do not use USB or WiFi to communicate with each other.

The web interface auto-launches on the touchscreen when the Pi boots. You can also open it from any browser on the same network by going to http://kiln.local:5000

---

## 2. Parts List

All prices are approximate. Any equivalent part will work unless noted otherwise.

### Core Electronics

| Part | What to Buy | Notes |
|------|-------------|-------|
| Raspberry Pi 4 | Any RAM variant | 2GB recommended. 1GB will work but is tight — Chromium alone uses 300-400MB. Buy from a reputable supplier — avoid counterfeits. |
| Touchscreen | 7" HDMI touchscreen with USB touch | Search "7 inch HDMI touchscreen Raspberry Pi". Needs both an HDMI cable and a USB cable to the Pi. |
| ESP32 development board | Hosyond 3-Pack ESP32 ESP-WROOM-32 on Amazon (~$16) | Any 38-pin ESP32 WROOM-32 board works. The 3-pack gives you spares. Do NOT buy an ESP32-C3, ESP32-S2, or ESP32-S3 — the firmware only works with the original ESP32 WROOM-32. |
| Thermocouple amplifier | MAX31856 breakout board | Adafruit sells a reliable one (product #3263). |
| Thermocouple | K-type, rated above your max oven temperature | Stainless sheath recommended. Buy one rated to at least 2200F (1200C). |
| Solid-state relays x 2 | Baomain SSR-60DA, 60A | Two required — one per hot leg of the 240V circuit. |
| Contactor | Schneider Electric LC1D32BD | This specific model matters — it has a 24V DC coil. |
| MOSFETs x 2 | IRLZ44N | Sold in packs on Amazon. Logic-level gate (works at 3.3V). |
| E-Stop button (OPTIONAL) | Normally-closed (NC) momentary pushbutton, panel-mount | Red mushroom head style is conventional. NC means the circuit is complete when the button is not pressed. If you do not install one, connect ESP32 GPIO34 directly to 3.3V. |
| Door switch | SPDT rocker switch | Needs both a normally-open (NO) and normally-closed (NC) contact. |
| Shutdown button (OPTIONAL) | Normally-open (NO) momentary pushbutton, small panel-mount | Press and hold triggers a clean Pi shutdown. You can skip this — the controller has a software shutdown button in the Settings tab, and you can also shut down via SSH. |

### Power Supplies

| Part | What to Buy | Notes |
|------|-------------|-------|
| 5V DIN rail supply | Mean Well MDR-60-5 | Powers the Pi, ESP32, and touchscreen. |
| 24V DIN rail supply | Mean Well MDR-60-24 | Powers the contactor coil, SSRs, and fans. |

### Hardware and Wiring

| Part | Notes |
|------|-------|
| Resistors — 100 ohm x 2 | Gate series resistors for the MOSFETs. |
| Resistors — 10k ohm x 4 | Pull-down resistors for MOSFET gates, E-Stop input, and door switch input. |
| Cooling fans x 2-3 | Alinan 8025 80mm 24V fans recommended. Keep the control box cool. |
| DIN rail and enclosure | Size to fit your components. |
| Terminal strip / ground bus bar | Land all ground wires here. |
| Wire | Use appropriately rated wire for each voltage. 18 AWG is fine for low-voltage control wiring. For 240V mains wiring see Section 3. |
| MicroSD card | 16GB or larger, Class 10. |

### Tools You Will Need

- Computer (Windows or Mac) to flash the Pi SD card and the ESP32
- MicroSD card reader
- Soldering iron (for connecting wires to the ESP32 and MAX31856, if using a bare breakout board)
- Multimeter (strongly recommended for verifying wiring before powering on)

---

## 3. 240V Mains Wiring — Safety Warning

**WARNING: This section involves 240V AC mains wiring. Contact with live 240V wiring can cause serious injury or death. If you are not experienced with mains electrical work, hire a licensed electrician to complete the 240V connections. Do not skip this warning.**

The 240V side of this controller connects your oven's heating element to household mains power through a contactor and two solid-state relays. The contactor acts as a master disconnect — it physically breaks the AC circuit when the door is open or any fault occurs, regardless of what the software is doing.

**What the 240V circuit looks like:**

Mains power (two hot legs at 240V) enters the enclosure through a dedicated 30A double-pole breaker and a NEMA 6-30P inlet. Both hot legs connect to the input terminals (L1, L2) of the contactor. The output terminals (T1, T2) connect to the AC input of SSR1 and SSR2 respectively. The AC outputs of the SSRs connect to the two legs of the heating element.

The contactor coil runs on 24V DC (from the MDR-60-24 supply) and is controlled by the door switch and the ESP32. When the door is open, the hardware circuit cuts 24V to the contactor coil and the contactor drops out — this is completely independent of the software.

**What a licensed electrician needs to do:**
- Run 10/3 SOOW flexible cable from the breaker panel to the enclosure
- Install a 30A double-pole breaker
- Wire the NEMA 6-30P inlet
- Connect L1 and L2 to the contactor input terminals
- Verify all connections and ground continuity before energizing

See STEELHAUS_Wiring.docx for the complete wiring diagram including the AC circuit.

---

## 4. Low-Voltage Wiring Summary

Everything in this section runs at 5V, 3.3V, or 24V. These voltages will not electrocute you, but wiring errors can damage components — double-check each connection before powering on.

All grounds (5V supply negative, 24V supply negative, Pi ground, ESP32 ground, MOSFET sources) connect to a single common ground bus.

### Pi to ESP32 (Serial — 2 wires)

The Pi and ESP32 communicate directly over two wires. TX always connects to RX and vice versa.

| Pi Pin | Pi Signal | Direction | ESP32 Pin |
|--------|-----------|-----------|-----------|
| Pin 8 | GPIO14 TX | -> | GPIO16 RX2 |
| Pin 10 | GPIO15 RX | <- | GPIO17 TX2 |

The Pi and ESP32 each get their own 5V connection from the MDR-60-5 supply. They do not power each other through these serial wires.

### ESP32 to MAX31856 Thermocouple Amplifier (SPI — 4 wires)

| ESP32 Pin | MAX31856 Pin | Notes |
|-----------|--------------|-------|
| GPIO5 | CS | |
| GPIO18 | SCK | |
| GPIO19 | MISO (SDO) | |
| GPIO23 | MOSI (SDI) | |
| 3.3V | VCC | Use 3.3V — NOT 5V |
| GND | GND | |

### ESP32 GPIO Pins

| ESP32 Pin | Function | Connected To |
|-----------|----------|--------------|
| GPIO27 | SSR control | IRLZ44N #1 gate via 100 ohm resistor |
| GPIO33 | Contactor control | IRLZ44N #2 gate via 100 ohm resistor |
| GPIO34 | E-Stop input | NC button + 10k ohm pull-down to GND. If not using an E-Stop button, connect GPIO34 directly to 3.3V. |
| GPIO35 | Door switch input | Door switch NC contact + 10k ohm pull-down to GND |

### MOSFET Wiring (both MOSFETs wired identically)

    ESP32 GPIO ---- 100 ohm ---- MOSFET Gate
                                 MOSFET Gate ---- 10k ohm ---- GND
                                 MOSFET Source ---- GND
                                 MOSFET Drain ---- Load (-)

### SSR Control Circuit (MOSFET #1, GPIO27)

    MDR-60-24 (+24V) ---- SSR1 DC+ input
    SSR1 DC- ---- SSR2 DC+ input   (SSRs in series on control side)
    SSR2 DC- ---- IRLZ44N #1 Drain
    IRLZ44N #1 Source ---- GND

### Contactor Coil Circuit (MOSFET #2, GPIO33)

    MDR-60-24 (+24V) ---- Door Switch NO contact
    Door Switch NO contact ---- Contactor A1 (coil +)
    Contactor A2 (coil -) ---- IRLZ44N #2 Drain
    IRLZ44N #2 Source ---- GND

### Door Switch Wiring

The door switch has two independent circuits using two separate poles:

**Pole 1 — NO contacts (hardware safety, no software involved):**
Door closed = switch pressed = NO contact closes = 24V reaches contactor coil = contactor energized = AC power available to heating element. Door open = NO contact opens = contactor drops out = AC power cut. This is entirely hardwired with no software involved.

**Pole 2 — NC contacts (software awareness):**

    ESP32 3.3V ---- NC contact ---- ESP32 GPIO35 ---- 10k ohm ---- GND

Door closed = NC contact open = GPIO35 LOW = normal.
Door open = NC contact closed = GPIO35 HIGH = dashboard alert displayed.

### E-Stop Button (Optional — NC)

    ESP32 3.3V ---- NC button ---- ESP32 GPIO34 ---- 10k ohm ---- GND

Button intact = GPIO34 HIGH = normal. Button pressed or wire break = GPIO34 LOW = immediate shutdown.

If you are not installing an E-Stop button, skip the button and resistor and instead run a single wire from ESP32 3.3V directly to GPIO34. This tells the firmware the E-Stop circuit is always intact. You can add a physical E-Stop button later if desired.

### Shutdown Button (Optional — NO)

    Pi GPIO21 (Pin 40) ---- NO button ---- GND

Press and hold triggers a clean Pi shutdown. The Pi has an internal pull-up on this pin — no resistor needed. This is optional. Without it, you can shut the Pi down using the software shutdown button in the Settings tab of the controller UI, or by running "sudo shutdown now" over SSH.

### Power Supply Connections

| MDR-60-5 (+5V) powers | MDR-60-24 (+24V) powers |
|---|---|
| Raspberry Pi (GPIO pins 2 or 4) | Door switch NO contact (contactor circuit) |
| ESP32 (VIN pin) | SSR1 DC+ input |
| Touchscreen (5V input) | Fans (+) |

See STEELHAUS_Wiring.docx for the complete printable wiring diagram with all connections.

---

## 5. Set Up the ESP32

This section programs the ESP32 with the firmware that reads the thermocouple and controls the SSRs. Do this from your computer before installing the ESP32 in the control box.

### Step 1 — Install Arduino IDE

Go to https://www.arduino.cc/en/software and download Arduino IDE 2 for your operating system (Windows or Mac). Install it.

### Step 2 — Add ESP32 Board Support

Arduino IDE does not support ESP32 boards out of the box. You need to add them.

1. Open Arduino IDE
2. Go to File -> Preferences (on Mac: Arduino IDE -> Settings)
3. Find the field labeled "Additional boards manager URLs"
4. Paste this URL into that field:
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
5. Click OK
6. Go to Tools -> Board -> Boards Manager
7. Search for: esp32
8. Find "esp32 by Espressif Systems" and click Install. This downloads about 300MB and takes a few minutes.

### Step 3 — Install Required Libraries

The ESP32 firmware needs two libraries.

Go to Tools -> Manage Libraries (or click the library icon on the left sidebar).

Search for and install each of these:

- Adafruit MAX31856 library — by Adafruit. Click Install. When asked if you want to install dependencies, click Install All.
- ArduinoJson — by Benoit Blanchon. Install version 6.x (not 7.x).

### Step 4 — Open the Firmware

In Arduino IDE, go to File -> Open and navigate to the esp32_firmware folder inside the STEELHAUS project files. Open esp32_firmware.ino.

### Step 5 — Select the Correct Board

This step is important. Using the wrong board setting will cause the firmware to fail.

Go to Tools -> Board -> esp32 -> ESP32 Dev Module

Make sure it says ESP32 Dev Module — not ESP32-C3, not ESP32-S2, not ESP32 Wrover. Exactly "ESP32 Dev Module."

### Step 6 — Connect the ESP32 and Upload

Plug the ESP32 into your computer with a Micro-USB cable.

Go to Tools -> Port and select the port that appeared when you plugged in the ESP32.

- On Windows it will look like COM3 or COM4 (the number may vary)
- On Mac it will look like /dev/cu.usbserial-xxxx

If you see multiple ports and are not sure which one is the ESP32, unplug it, check what ports are listed, plug it back in, and the new one that appears is the ESP32.

Click the Upload button (the right-arrow icon, or go to Sketch -> Upload).

The IDE will compile the firmware (takes about 30 seconds) and then upload it. You will see progress dots in the status bar. When it says "Done uploading," the ESP32 is programmed.

You can now disconnect the ESP32 from your computer and install it in the control box.

---

## 6. Set Up the Raspberry Pi

This section gets the Pi flashed, onto your network, and ready to receive the software.

### Step 1 — Download Raspberry Pi Imager

Go to https://www.raspberrypi.com/software/ and download Raspberry Pi Imager for your operating system (Windows or Mac). Install it.

### Step 2 — Flash the SD Card

Insert your MicroSD card into your computer (using a USB card reader if needed).

Open Raspberry Pi Imager.

- Click Choose Device -> select Raspberry Pi 4
- Click Choose OS -> select Raspberry Pi OS (64-bit)

IMPORTANT: The Raspberry Pi Imager may default to the latest version of the operating system, which is currently called "Trixie." This controller requires Bookworm. If the OS shown does not say "Bookworm," click "Raspberry Pi OS (other)" and look for the Bookworm 64-bit option in the list. Do not use Trixie or any other version — only Bookworm is supported.

- Click Choose Storage -> select your SD card

Before clicking Write, click the gear icon (or "Edit Settings") to configure advanced options:

- Set hostname to: kiln
- Enable SSH: check this box
- Set username to: pi  and choose a password (write it down — you will need it)
- Configure your WiFi network name and password
- Set your locale and timezone

Click Save, then click Write. This takes a few minutes. When it finishes, eject the SD card.

### Step 3 — First Boot

Insert the SD card into the Raspberry Pi. Connect the HDMI touchscreen and USB touch cable. Power on the Pi.

Wait about 60-90 seconds for first boot to complete. The Pi will resize the filesystem and reboot once automatically.

### Step 4 — Connect via SSH

SSH lets you type commands on the Pi from your computer without a keyboard attached to it.

On Windows: Open PowerShell (search for it in the Start menu). Type:

    ssh pi@kiln.local

Press Enter. If asked "are you sure you want to continue connecting?" type yes and press Enter. Enter your password when prompted.

On Mac: Open Terminal (in Applications -> Utilities). Type the same command:

    ssh pi@kiln.local

If kiln.local does not resolve, find your Pi's IP address on your router's device list and use that instead (for example: ssh pi@192.168.1.42).

You should now see a command prompt that looks like:  pi@kiln:~ $
You are now typing commands on the Pi.

---

## 7. Install the Controller Software

All of the following commands are typed into the SSH session on the Pi (from Step 4 of Section 6), unless noted otherwise.

### Step 1 — Copy the Project Files to the Pi

This step is done from your computer — not from the SSH session. Open a new PowerShell or Terminal window (keep the SSH session open in a separate window).

The scp command copies the kiln_controller folder from your computer to the Pi. You need to run this command from the folder that contains kiln_controller — for most people this will be their Downloads folder.

On Windows, if you downloaded and unzipped the files to your Downloads folder, run:

    scp -r C:\Users\YourName\Downloads\kiln_controller pi@kiln.local:/home/pi/

Replace YourName with your actual Windows username (this is the name of your user folder in C:\Users\). If you saved the files somewhere else, replace the path with the correct location.

On Mac, if the folder is in Downloads:

    scp -r ~/Downloads/kiln_controller pi@kiln.local:/home/pi/

This copies the entire kiln_controller folder to the Pi. The folder must contain all of the project files: app.py, controller.py, pid_controller.py, program_engine.py, program_store.py, esp32_bridge.py, requirements.txt, setup.sh, and static/index.html.

### Step 2 — Run the Setup Script

Back in your SSH session, run:

    cd /home/pi/kiln_controller
    sudo bash setup.sh

This script does the following automatically:
- Installs all required Python packages (Flask, pyserial)
- Installs Chromium browser
- Creates the systemd service that starts the controller at boot
- Configures the Pi to boot directly into the kiosk (no desktop, just the controller UI)
- Sets the touchscreen to the correct orientation
- Installs Tailscale for remote access

It takes 2-5 minutes. When it finishes you will see a "Setup Complete!" message.

### Step 3 — Enable the Hardware Serial Port

The Pi's GPIO serial port needs to be enabled manually. In the SSH session, run:

    sudo raspi-config

This opens a text-based configuration menu. Use the arrow keys and Enter to navigate:

1. Select Interface Options
2. Select Serial Port
3. "Would you like a login shell to be accessible over the serial port?" -> Select No
4. "Would you like the serial port hardware to be enabled?" -> Select Yes
5. Select Finish
6. When asked to reboot, select No (you will reboot in a moment)

Then run this command to disable Bluetooth (which shares the serial port and must be freed up):

    echo "dtoverlay=disable-bt" | sudo tee -a /boot/firmware/config.txt

Then tell the controller service to use the correct serial port:

    sudo sed -i 's|KILN_SERIAL=.*|KILN_SERIAL=/dev/ttyAMA0|' /etc/systemd/system/kiln-controller.service
    sudo systemctl daemon-reload

### Step 4 — Speed Up Boot (Optional but Recommended)

These commands disable services you do not need, which gets the controller UI on screen faster:

    sudo systemctl disable NetworkManager-wait-online.service
    sudo systemctl disable cups
    sudo systemctl disable ModemManager
    sudo systemctl disable colord
    sudo systemctl disable dphys-swapfile

### Step 5 — Reboot

    sudo reboot

The Pi will reboot. After about 8-10 seconds, the touchscreen should show the STEELHAUS controller interface. If it shows a desktop or a blank screen, wait a few more seconds — on first boot after setup the kiosk can take up to 20 seconds to appear.

---

## 8. First Boot and Test

### Verify the Controller is Running

From your computer's browser, go to:

    http://kiln.local:5000

You should see the STEELHAUS interface. If the page does not load, the controller service may still be starting — wait 30 seconds and try again.

### Check the Logs

If something is not working, SSH back into the Pi and check the logs:

    sudo journalctl -u kiln-controller -f

This shows live output from the controller. Press Ctrl+C to stop watching.

Common things to look for:
- "Connected to ESP32 on /dev/ttyAMA0" — serial communication is working
- Temperature readings appearing — thermocouple is working
- Any error messages

### Verify Temperature is Reading

On the dashboard, the current temperature should display room temperature (roughly 65-80F / 18-27C). If it shows 0.0F or displays a fault code, see the Troubleshooting section.

### Test the Door Switch

Open and close the kiln door. The dashboard should show a door open banner when the door is open and clear when it is closed. If it does not, check the door switch NC contact wiring (GPIO35).

### Test the E-Stop (if installed)

Press the E-Stop button. The dashboard should show an E-Stop alert. Press Reset on the dashboard to clear it.

---

## 9. Set Up Remote Access with Tailscale

Tailscale lets you access the controller from your phone or laptop from anywhere, even when you are not on the same WiFi network.

### Step 1 — Create a Tailscale Account

Go to https://tailscale.com and sign up. Use your Google, Microsoft, or GitHub account — you do not need a new password.

### Step 2 — Connect the Pi

In the SSH session on the Pi, run:

    sudo tailscale up

It will print a URL. Copy that URL and open it in a browser on your computer. Log in with the same account from Step 1. The Pi is now connected to your Tailscale network.

### Step 3 — Get the Pi's Tailscale IP Address

    tailscale ip

This prints an address starting with 100. Write it down — this is the permanent address for your controller.

### Step 4 — Install Tailscale on Your Phone or Laptop

Download the Tailscale app from the App Store or Google Play, or from https://tailscale.com/download for your laptop. Log in with the same account.

### Step 5 — Access from Anywhere

Open a browser and go to:

    http://100.x.x.x:5000

(Use the IP from Step 3.)

The controller interface will load exactly as it does on the local network. All browsers connected at the same time stay synchronized — changing the setpoint on your phone updates the touchscreen instantly.

---

## 10. First Use — PID Autotune

Before running any heat treat programs, you need to run an autotune at each temperature you plan to use. Autotune measures how your specific oven responds to heat and sets the control parameters (called PID gains) that give you precise, stable temperature control.

You need to autotune at each major temperature you use. For AEB-L stainless knife steel, that means:
- ~400F (204C) for tempering
- ~1000F (538C) — mid-range anchor
- ~1925F (1052C) for hardening

Each autotune takes 30-60 minutes and must run to completion without interruption.

### How to Run an Autotune

1. Make sure the oven is at room temperature before starting
2. Close the kiln door
3. Go to the PID tab
4. Enter the target temperature (e.g. 204C for 400F)
5. Leave the method set to STEELHAUS (this is the recommended default)
6. Click Start Autotune
7. Leave the oven running until the autotune completes — the dashboard shows progress
8. When complete, click Apply to save the result

Repeat for each temperature. Once you have zones at all three temperatures, the controller will automatically interpolate between them for any setpoint in between.

---

## 11. Built-in Heat Treat Programs

The following programs are pre-loaded. You can run them from the Programs tab.

| Program | Description |
|---------|-------------|
| 1084 High Carbon | Stress relief soak at 700F, then harden at 1475F (802C). 10-min soak then quench. |
| D2 Tool Steel | Step ramp to 1850F with soaks at 930F and 1470F before austenitizing. |
| Temper 375F | Double temper cycle at 375F (190C). Two 60-minute soaks. |
| Normalize 1084 | Single ramp to 1600F (870C) with a 10-minute soak to relieve stress. |

To run a program: go to the Programs tab, select the program, and click Start. The dashboard shows current segment, time remaining, and a live temperature chart.

Custom programs can be created in the Programs tab by defining ramp and soak segments.

---

## 12. Troubleshooting

**Temperature shows 0.0 or a thermocouple fault**
- Check the SPI wiring between the ESP32 and MAX31856 (GPIO5/18/19/23)
- Make sure MAX31856 VCC is connected to 3.3V — not 5V
- Check that the thermocouple wires are connected to T+ and T- on the MAX31856
- Check the logs: sudo journalctl -u kiln-controller -f

**No temperature / ESP32 not connected**
- Verify UART wiring: Pi Pin 8 (TX) -> ESP32 GPIO16, Pi Pin 10 (RX) -> ESP32 GPIO17
- Verify Bluetooth is disabled: grep disable-bt /boot/firmware/config.txt (should return a result)
- Verify the serial port setting: grep KILN_SERIAL /etc/systemd/system/kiln-controller.service (should show /dev/ttyAMA0)
- Check that /dev/ttyAMA0 exists: ls /dev/ttyAMA0

**Door open alert will not clear**
- Door closed should leave GPIO35 LOW — check that the NC contact is wired so that closing the door opens the contact
- Verify the 10k ohm pull-down resistor is between GPIO35 and GND

**Contactor will not energize**
- Check that the door switch NO contact is wired in series between +24V and contactor terminal A1
- Check MOSFET #2 wiring: Drain to A2, Source to GND
- Check that the ESP32 is connected and no faults are active on the dashboard

**E-Stop fault showing at startup**
- If you did not install an E-Stop button, make sure GPIO34 is wired directly to 3.3V (not left floating)
- If you did install an E-Stop button, check the 10k ohm pull-down from GPIO34 to GND and verify the NC button connects 3.3V to GPIO34 when not pressed

**Kiosk screen rotated wrong**

The display rotation is set in a file on the Pi. To change it, edit the file over SSH using the nano text editor:

1. SSH into the Pi
2. Open the file in nano by typing this command and pressing Enter:

       nano /home/pi/kiln_controller/xinitrc.sh

3. The file will open on screen. Use the arrow keys to move the cursor to the line that contains --rotate right
4. Change the word "right" to "left", "normal", or "inverted" depending on which way your screen needs to rotate
5. Save the file: hold Ctrl and press O, then press Enter to confirm the filename
6. Exit nano: hold Ctrl and press X
7. Reboot the Pi:

       sudo reboot

**Can't reach via Tailscale**
- On the Pi: tailscale status — the Pi should show as connected
- Both devices must be logged into the same Tailscale account
- Re-authenticate if needed: sudo tailscale up

**Browser shows "Reconnecting"**
- Restart the controller: sudo systemctl restart kiln-controller
- Check logs: sudo journalctl -u kiln-controller -n 50

**Can't SSH into the Pi**
- SSH always works regardless of kiosk mode — kiosk only runs on the screen
- Try using the IP address directly if kiln.local does not resolve

---

## 13. File Reference

    kiln_controller/
    |-- app.py                  Flask web server, REST API, SSE broadcast
    |-- controller.py           Main PID control loop
    |-- pid_controller.py       PID algorithm and relay autotune
    |-- program_engine.py       Ramp/soak program execution
    |-- program_store.py        SQLite storage for programs, logs, settings
    |-- esp32_bridge.py         Serial communication with ESP32
    |-- requirements.txt        Python package list
    |-- setup.sh                One-time Pi setup script (run once on fresh install)
    |-- start_kiosk.sh          Kiosk launcher (called at boot)
    |-- xinitrc.sh              X session: display rotation + Chromium
    |-- cage_wrapper.sh         Wayland wrapper (kept for reference)
    |-- STEELHAUS_Wiring.docx   Complete printable wiring diagram
    |-- data/                   SQLite database (created automatically at runtime)
    |-- static/
    |   |-- index.html          Complete single-file web UI
    |-- esp32_firmware/
        |-- esp32_firmware.ino  Arduino firmware for ESP32 + MAX31856

### Useful Commands

    # Check if the controller is running
    sudo systemctl status kiln-controller

    # Watch live logs
    sudo journalctl -u kiln-controller -f

    # Restart the controller
    sudo systemctl restart kiln-controller

    # Check boot time
    systemd-analyze

    # Get Tailscale IP
    tailscale ip

    # Pull a file from the Pi to Windows (run in PowerShell, not SSH)
    scp pi@kiln.local:/home/pi/kiln_controller/app.py C:\Users\YourName\Downloads\
