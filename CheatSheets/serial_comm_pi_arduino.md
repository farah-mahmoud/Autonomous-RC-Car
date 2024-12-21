
# Setting Up and Testing Serial Communication Between Arduino and Raspberry Pi Using SSH

## 1. Connecting the Raspberry Pi and Arduino
1. Connect the Arduino to the Raspberry Pi using a USB cable or serial pins (TX/RX). For TX/RX:
   - **Arduino TX** to **Raspberry Pi RX**.
   - **Arduino RX** to **Raspberry Pi TX**.
   - Common Ground (GND) between both devices.

2. Enable UART on the Raspberry Pi by editing the `/boot/config.txt` file:
   ```bash
   sudo nano /boot/config.txt
   ```
   Add or ensure the following lines are present:
   ```
   enable_uart=1
   ```
   Save and exit (`Ctrl + O`, `Enter`, `Ctrl + X`).

3. Reboot the Raspberry Pi:
   ```bash
   sudo reboot
   ```

---

## 2. Writing the Arduino Code
1. Open the Arduino IDE and write a simple echo program:
   ```cpp
   void setup() {
       Serial.begin(9600); // Start serial communication at 9600 baud rate
   }

   void loop() {
       if (Serial.available()) {
           String message = Serial.readString();
           Serial.print("Echo: ");
           Serial.println(message); // Echo back the received message
       }
   }
   ```

2. Upload the code to the Arduino.

---

## 3. Writing the Raspberry Pi Python Script
1. Write a Python script (`serial_comm.py`) to send and receive messages:
   ```python
   import serial
   import time

   # Open serial connection
   ser = serial.Serial('/dev/serial0', 9600, timeout=1)
   ser.flush()

   while True:
       # Send a message to Arduino
       ser.write(b"Hello Arduino!\n")
       print("Sent: Hello Arduino!")
       time.sleep(1)

       # Read and print Arduino's response
       if ser.in_waiting > 0:
           response = ser.readline().decode('utf-8').strip()
           print(f"Received: {response}")
   ```

2. Save the script on your development machine.

---

## 4. Transferring the Python Script to Raspberry Pi
### Using `scp`
1. Run this command on your development machine:
   ```bash
   scp /path/to/serial_comm.py pi_user@pi_ip:/home/pi/
   ```

2. Replace `/path/to/serial_comm.py`, `pi_user`, and `pi_ip` as appropriate.

3. SSH into the Raspberry Pi to verify:
   ```bash
   ssh pi_user@pi_ip
   ls /home/pi/
   ```

---

## 5. Running the Serial Communication Script on Raspberry Pi
1. SSH into the Raspberry Pi:
   ```bash
   ssh pi_user@pi_ip
   ```

2. Run the Python script:
   ```bash
   python3 /home/pi/serial_comm.py
   ```

3. Monitor the output:
   - **Sent Messages**: "Hello Arduino!"
   - **Received Messages**: "Echo: Hello Arduino!"

---

## 6. Debugging Tips
- Ensure the Arduino is properly connected and powered.
- Verify the Raspberry Pi serial port (`/dev/serial0` or another). Run `ls /dev/tty*` to list available ports.
- Test the Arduino code using the Arduino Serial Monitor to ensure it's working as expected.
