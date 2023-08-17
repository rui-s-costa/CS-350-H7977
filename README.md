# 7-1 Project Submission

**Student:** Rui Costa  
**University:** Southern New Hampshire University  
**Course:** CS-350-H7977 Emerging Sys Arch & Tech 23EW6  
**Instructor:** Professor Roland Morales  

## My Reflections on Thermostat Functionality and Hardware Architecture

### Introduction
In this report, I will present my reflections on developing and analyzing a thermostat system. The aim was to design a system that reads room temperature, controls an LED indicator, allows temperature adjustments using buttons, and simulates data transmission to a cloud server. I employed Code Composer Studio (CCS) IDE for this endeavor. The project's primary components encompassed Timer, Interrupt, I2C, GPIO, and UART peripherals.

### Code Functionality
The code I drafted impeccably executed all the outlined functionalities. Leveraging the I2C communication protocol enabled me to precisely read the room temperature and project the results through the UART. The LED indicator, indicative of the heating status, was adeptly managed by the GPIO peripheral. The buttons ensured convenient modification of the set temperature. With the Timer in play, I systematically oversaw all periodic functions, ensuring the thermostat system's uninterrupted operation.

#### Timer
A continuously running Timer was set with a 1-second interval, vital for the effective functioning of the task scheduler, and by extension, the thermostat system.

#### Interrupt
Configuring interrupts to detect button activations was a gratifying task. By empowering the GPIO with interrupt capabilities and laying down two callback functions, "gpioButtonFxn0" and "gpioButtonFxn1", I effectively modified the variables for adjusting the temperature.

#### I2C Peripheral
The initialization of the I2C peripheral and its subsequent use to fetch data from the temperature sensor was a success. The temperature readings were then interpreted and relayed through the UART.

#### GPIO Peripheral
The GPIO peripheral, configured for LED control and button input, achieved its purpose. It effectively signified heating status and facilitated user interaction.

#### UART Peripheral
I was successful in initializing the UART peripheral for data transmission. The capability to exhibit critical information via UART proved invaluable.

### Task Scheduler Functionality
The task scheduler's implementation was a fulfilling challenge. By strategically utilizing a structured array, tasks were systematically executed based on their designated periods.

### Best Practices
Throughout the project, I adhered to coding best practices ensuring clarity, descriptiveness, and structured organization.

### Conclusion
In conclusion, I am content with the realized code, adhering to all project requirements. The thermostat system operates efficiently, ensuring user-friendliness and functional superiority.

---

### Reflection

1. **Summarize the project and what problem it was solving.**  
   The project involved developing a thermostat system that could read room temperatures, control an LED indicator based on heating status, allow temperature adjustments via buttons, and simulate data transmission to a cloud server. It addressed the need for a user-friendly thermostat system that could be easily monitored and adjusted.

2. **What did you do particularly well?**  
   I executed the I2C communication protocol impeccably, allowing precise temperature readings. The integration of GPIO for LED and button functionalities also stood out.

3. **Where could you improve?**  
   While the project met all the functional requirements, future iterations could include more user-friendly interfaces or integration with more advanced cloud platforms.

4. **What tools and/or resources are you adding to your support network?**  
   This project allowed me to become familiar with the Code Composer Studio (CCS) IDE, enhancing my understanding of the I2C protocol, UART communication, and interrupt handling.

5. **What skills from this project will be particularly transferable to other projects and/or course work?**  
   The skills acquired in effective task scheduling, adhering to coding best practices, and the in-depth understanding of various hardware peripherals will undoubtedly be transferable to future endeavors.

6. **How did you make this project maintainable, readable, and adaptable?**  
   By sticking to coding best practices, using descriptive variable names, and maintaining structured code organization, I ensured the project's maintainability and readability. Modular code structure and clear documentation guarantee adaptability for future modifications.
