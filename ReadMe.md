### PWM Project

**Project Name:** PWM Project  
**Purpose:** Generate PWM pulses at the output and allow changes at the input using an encoder, button, and a user interface that communicates with the device via a COM port.

#### **Application:**
- Quick testing of buck-boost circuits.
- Gaining deeper understanding of chip functionality.

---

### **Hardware**

1. **MCU:** STM32F103C8T6  
2. **OLED SSD1306: (7 pin SPI)
3. **Encoder Module** : 5 pin ( Have button )

---

### **Software Features**

1. **PWM Signal Generation:**
   - Generate PWM pulses with a fixed frequency and adjustable duty cycle.

2. **Encoder Signal Reading:**
   - Read signals from the encoder module for input changes.

3. **OLED Display:**
   - Display information such as frequency, duty cycle, and selection options on the OLED screen using SPI or I2C communication.

4. **Integration of Encoder and OLED:**
   - Combine the encoder signal reading with OLED display updates.

5. **PWM Adjustment Based on Encoder:**
   - Adjust PWM pulse parameters (frequency and duty cycle) dynamically using the encoder input.

6. **Computer Communication:**
   - Write a computer program to communicate with the device via a COM port.
   - Allow the user to adjust PWM parameters through the computer interface.

7. **Enclosure Design:**
   - Create a 3D design for the project box.

8. **Data Persistence:**
   - Save the final parameters (frequency and duty cycle) to ROM memory.
   - Ensure the device resumes with the last saved values after being powered off and back on.

---

### **Development Steps**

1. Generate a PWM signal with adjustable frequency and duty cycle.
2. Implement encoder signal reading functionality.
3. Configure and display real-time parameters on the OLED screen.
4. Integrate encoder reading and OLED display.
5. Adjust PWM output based on encoder input.
6. Develop a PC program for COM port communication.
7. Design a 3D enclosure for the hardware.
8. Implement ROM data saving and retrieval functionality.

