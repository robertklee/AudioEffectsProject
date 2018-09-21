# Audio Effects Project

Designed by Robert Lee and Declan McIntosh

*University of Victoria*

*May-August, 2018*

## Project Goal

The goal of this project was to create an audio player which can take an analog input and perform some transformations (pitch and echo) on the music, then output this modified music with an imbedded speaker. This unit should have a convenient user interface and a vibrant display. 

## Constraints

Several constraints were placed on the project by the customer. The project required the use of the **STM32F4 Discovery board** (STM32 board) for digital signal processing (DSP) and logical processing. The board and all peripherals on the printed circuit board (PCB) must be powered solely by a USB port on a computer. USB ports will provide 5 V at 0.1 A for low power devices before handshaking negotiations are required [1]. The STM32 board must be used to implement pitch shifting and add an echo effect. The software must be written in the Eclipse Integrated Development Environment (IDE). The PCB must be designed using KiCAD. The designed enclosure (see Appendix A) must be designed in SolidWorks. The PCB and all components must be RoHS (Restriction of Hazardous Substances) compliant. A cost constraint of $150 CAD was decided by the group. 

## Repository Structure

Source code is provided in `/src`. The embedded C program, `main.c`, contains the logic driving all the functionality of the board. The code is broken down into the following major components:
-	Timer-run interrupt service routine to drive the LED 8-by-8 matrix display, as described above
-	Timer-run interrupt service routine to poll input buttons and pitch shifting potentiometer, as described above
-	Timer-run interrupt service routine to read ADC audio input, and to output processed data using the DAC
-	Finite State Machine logic to determine the current state, and to display it on the LED matrix display
-	LED matrix display backend functions and data, to enable the user to fill the display buffer with up to 10 seconds of images, played at 25 frames per second

There were three major analog circuits within the **PCB**:
-	Level shifter circuit: since the input signal is sinusoidal centered at 0 V, and the Analog-to-Digital Converter (ADC) accepts a voltage range from 0 V to 3 V, the input signal must be shifted up.
-	Digital-to-Analog Converter (DAC) Quantization Error Amelioration Circuit: since the DAC can only output discrete voltages, not a continuous range of voltages, there are a lot of jagged bumps in the signal. These arise as high-frequency noise in the signal and are removed with a lowpass filter.
-	Audio amplification and bandpass filtering prior to output to speaker: the LM386 amplifier was operated with a gain of 20. The bandpass filter consists of a DC blocking capacitor and a lowpass filter.


## Testing & Validation

The testing and validation of the audio effects board required the use of the following equipment:
-	Oscilloscope with two inputs and probes
-	Function/signal generator
-	DC power source
-	Multimeter
-	Audio source
-	3.5mm male connector
-	Laptop

The testing and validation of the LED board required the use of the following equipment:
-	DC power source
-	Laptop

## Conclusion & Recommendations 

All major objectives of functionality were achieved for the final project, with some parts of systems having minor issues and others exceeding requirements adding functionality. The project’s analog audio subsystems including the second order low pass filter to remove quantization error, the level shifting input, and the audio signal amplifier using a LM386 functioned as expected with some marginal deviation from theoretical values. Some noise was noted in the final build of the FX project. This noise is expected to be caused by noise in the power supply. Further the amplification of the audio signal was approximately 15 times gain which was lower than the expected 20 gain of the configuration as designed, giving a quiet sound to the output. The display requirements were met entirely with an 8 by 8 LED matrix for display. Further the requirements were met with two linear potentiometers used as voltage dividers of the input voltage signal which were used for coarse and fine volume control. However, as the potentiometers did not have plastic caps affixed, when a person would touch the potentiometers some noise would be introduced from the person. All software requirements were met. Button debouncing was performed using an ISR routine on a 5ms timer. Pitch shifting was implemented using potentiometer-controlled analog input to determine the magnitude and direction of the frequency shift. A circular buffer was used to store and retrieve raw analog ADC values to implement echo. A finite state machine architecture was implemented to control which effects were being used at any given time. Finally, another circular buffer was implemented to display things on the LED array. Overall all expected and requested functionality was at least minimally met. 

There were some issues with a lack of simulation software embedded into the KiCAD software. The wire used for the connection between the LED display and the main Audio FX board was 22AWG multi-strand wire and would often break when handled. This was an issue that would had taken a considerable amount of time to fix entirely so the breaks were fixed as they happened. During the implementation of pitch shifting there were some issues shifting up, as low frequencies resulting from the Fast Fourier Transform of the level shift offset, which would produce an impulse-like value in the low frequency range. As frequencies were shifted up, this impulse-like value would be shifted past the minimum frequency to be filtered out by the DC-blocking capacitor, causing it to be audible as a harmonic. This was solved by starting the shift at a higher frequency, so the impulse value would be unmoved.

There were several limitations to the final design of our Audio FX board. Firstly, the display buffer could only handle 10 seconds of frames at 25 FPS. This took up 2 KB of memory, which can be an issue given the relatively limited 192 KB total memory on the board being shared across an echo buffer, FFT dependencies, and FFT output bins [7]. The I/O, while functional for the required testing with only 3 pushbuttons, was relatively limited in potential for further functionality without a convoluted user interface. The major limitation of the final implementation of the project was the poor maximum volume only amplifying the outputted signal by 15 times gain which made the sound quiet. 

If a successor to our first design was to be built some major changes would be made. First, another second-order active-low-pass filter would be placed on the input to the ADC of the STM board so that higher frequency signals on the input would not be sampled and cause aliasing issues as it wraps around into the maximum frequency of 8 kHz signals the STM board is sampling at. This would help improve general sound quality of the audio signal. To further improve the audio signal a capacitor should be placed between power and ground to reduce ripples in the voltage. Alternatively, all the amplifiers could be powered off a separate voltage regulator which would help provide a clean power source. This is expected to reduce the notable noise assumed to be caused by a noisy voltage source. Knowing that the board is not being put in a manufactured case, the components would be chosen as surface mounts rather than mounting holes for wires to connect to offboard components. Some changes should also be made to confirm a better gain on the audio amplifier to make the music experience much better to the ear. These changes would improve the sound quality and make the music listening experience much better. Additionally, using plastic caps on the potentiometers would prevent capacitance noise on the audio line. This noise is caused when the conductive metal exterior of the potentiometer is touched by a capacitive disturbance like a person who is not properly grounded. Finally, the number of pull-up resistors could be reduced by using the board’s internal pull-up or pull-down resistors, reducing the cost and complexity of the PCB.

### References
[1]	“USB Power Delivery,” Universal Serial Bus. [Online]. Available: http://www.usb.org/developers/powerdelivery/. [Accessed: 29-Jul-2018]. 
