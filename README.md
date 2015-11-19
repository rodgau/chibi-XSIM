## Welcome to chibi-XSIM ##


*chibi-XSIM* is an [*STM32F4 Discovery*](http://www.st.com/web/catalog/tools/FM116/SC959/SS1532/PF252419?sc=internet/evalboard/product/252419.jsp)  based motor controller for the [*X-Sim*](http://www.x-sim.de) motion simulator software.
# Overview #

*X-Sim* is a Windows application designed to power motion simulator hardware for racing and flightsim games. The *chibi-XSIM* project demonstrates the feasibility of *STM32F4 Discovery* powered motion simulators and offers an exciting level of performance through its unique choice to ride upon a real time operating system core–as found in [*ChibiOS/RT*](http://www.chibios.org/dokuwiki/doku.php?id=chibios:product:rt:start).  It's offered here as a demonstration project and possible launch pad for others.

I would be excited to see what anyone develops from this!

# History #
----------
##### Open6DOF High Speed is dead! #####
##### Long Live Open6DOF High Speed! #####

## ##

Spring 2013 saw the announcment of a project called [*Open6DOF High Speed*](https://rodwerks.wordpress.com/projects/open6dof-high-speed/) to spotlight the potential of *Discovery*-based motion simulators. The firmware for that project, though never released, is what is being offered here in its embryonic form as *chibi-XSIM*.

*chibi-XSIM* represents how far *Open6DOF High Speed* got before its plug was pulled; a snapshot of its state of development as of June 2013

### So, What Happened...? ###
Let's see...

- *X-Sim* interface:				**CHECK**
- User Interface:			**CHECK**
- Advanced PID Implementation:  **CHECK**
- Valet mode, Agression Presets, Data Logging, RTOS-based, 100 kHz Precision: **CHECK** 
- Game Tested:				**CHECK**
- Practical...? **BIG QUESTION MARK**

While standing back to reflect upon his mad creation, the author asked himself how accessible *Open6DOF High Speed* would be for others. The problem:  an unfriendly and horribly complicated development tool chain was at its core. Think:  *Arduino's Opposite.*

It worked wonderfully, though, humming along in all its *ChibiOS/RT*-powered glory with 0.01 millisecond *Dependant Ideal PID* implemented precision. It even had a nifty user interface menu system where a guy could plug it into his computer while it was running to tweak settings.

My vision, however, was a project people across the world could plug into easily, add their shared contribution, and elevate it to something amazing....and this just didn't feel like it; appealing only to the select few, perhaps.

# Why chibi-XSIM? #
Just to put it out there anyway, man!

*Open6DOF*'s been taken out of storage, dusted off, rebranded, and *GitHubbed* for all to see. While it's only a subset of the *Open6DOF* project, it does still offer that useful bridge between the *X-Sim* middleware and any imaginable motor actuator out there.

# Details #
Currently written for only two motors and the [*Pololu*](https://www.pololu.com/category/94/pololu-simple-motor-controllers) brushed DC motor controller, *chibi-XSIM* can easily be expanded to support any kind of motor, hydraulic or pneumatic system and as many working axes as desired.


**EX:**  Racing Game ↔ *X-Sim* ↔ **Discovery Board** ↔ H-bridges ↔ DC Motors ↔ Roll/Pitch

The board connects to the PC via a USB-hosted serial cable to receive streamed position commands from *X-Sim*, and to H-bridge motor controllers to provide the required PWM motion and direction signals. A second cable connects to your PC for the *chibi-XSIM* on-screen control user interface. There you navigate various menus to make on-the-fly PID changes and to follow a calibration "wizard" for initial end points setup. 


# Features #
The current firmware offers:

- *X-Sim* and *xSimCTRL* software protocol compatible.
- Support for 3 DOF (Stewart transforms for 6 DOF not implemented)
- On screen Setup menu (through your terminal emulator) 
- Emergency Stop, Return-to-Center, Machine Calibration Wizard, and adjustable Deadzones
- Five agression level presets
- Data Logging
- PID tuning by real time performance graphs (see below)
- P-only, PI-only, and both PID mode variants (Independent and Dependent)
- An attempt to offer IMC ("Internal Model Control") PID Tuning capability ([see here](http://blog.opticontrols.com/archives/260)). 
- Adjustable Peak Limiter for testing safety and to match large motors to small power supplies (or vice-versa)
- *Valet Mode* for the kids :-)
- Potentiometer Software Scaling (change your mechanical gearing without affecting your PID feedback behavior)
- Oversampling and digital filtering
- Rapid Response Capable (*X-Sim*→*chibi-XSIM* latency ≈0.006 ms)
- 500 Hz control loops nominal, with latencies of around 0.013 ms
- entirely hardware-based event-driven callback architecture
- could potentially drive very precise and rapid real time positioning solutions with demands well beyond those of a gaming application
- HAL drivers for a wide variety of peripherals (quadrature optical encoders, LCD displays, SPI, I²C and CAN bus interfaces, etc.)
- written for ChibiOS/RT v2.5.2 


# Graphing #

### XSimCTRL ###
*XSimCTRL* is fantastic software written by Alexey Priladyshev used to help tune motion simulators. *chibi-XSIM* supports the *XSimCTRL* data stream protocol, so you can track your machine's performance while you experiment with various PID settings.

**Screenshot:**

![XSimCTRL Performance Monitoring](http://i.imgur.com/7zY0DeS.png)

**Video:**

[![XSimCTRL Demo](http://img.youtube.com/vi/bQH6uieXrV4/0.jpg)](http://www.youtube.com/watch?v=bQH6uieXrV4)

### Generic Data Loggers ###
*chibi-XSIM* also supports most third party data loggers with its ability to output serial data streams describing the performance of your running motion simulator. The free version of *DAQFactory Express*, for example, is ideal for this and has been used with success.

# Disclaimer #
Please be cautious with the use of this very experimental software. I offer no claims as to the suitability **or the safety** of the firmware provided here in *chibi-XSIM*.

Is it buggy? Quite probably.

Does it work? Yes, but not a single motion simulator has been made with it yet, so...

Could it hurt you? **Yes!**

Playing with powerful electric motors, home made contraptions and experimental firmware is inherently dangerous. If you choose to try *chibi-XSIM*, please be careful.

# Further Options #

Firmware has also been written for the new and exciting *Electric Imp* platform.

Read about my **zimple-XSIM** motion simulator project [here]( https://github.com/rodgau/zimple-XSIM).