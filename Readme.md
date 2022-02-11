Example project on how to read values from a SHT85 sensor connected to a STM32 Microcontroller with USB port and advertise the results using the BleuIO dongle with the STM32 as a USB CDC Host.

# 1. What you will need

- A BleuIO dongle (https://www.bleuio.com/)
- A SHT85 sensor (https://sensirion.com/products/catalog/SHT85/)
- A board with a STM32 Microcontroller with a USB port. (A Nucleo-144 development board: NUCLEO-H743ZI2, was used developing this example. (https://www.st.com/en/evaluation-tools/nucleo-h743zi.html)<br>
  To connect the dongle to the Nucleo board we used a "USB A to Micro USB B"-cable with a USB A female-to-female adapter.)
- STM32CubeIDE (https://www.st.com/en/development-tools/stm32cubeide.html)

# 2. How to setup project

## 2.1 Downloading the project from GitHub

Either clone the project, or download it as a zip file and unzip it, into your STM32CubeIDE workspace.

## 2.2 Importing as an Existing Project

- From STM32CubeIDE choose File>Import...>General>Existing Projects into Workspace then click 'Next >'
- Make sure you've choosen your workspace in 'Select root directory:'
- You should see the project "stm32_bleuio_SHT85_example", check it and click 'Finish'.
  > If you download the project as a zip file you will need to rename the project folder from 'stm32_bleuio_SHT85_example-master' to 'stm32_bleuio_SHT85_example'

# 3. Running the example

- In STMCubeIDE click the hammer icon to build the project.
- Open up the 'STMicroelectronics STLink Viritual COM Port' with a serial terminal emulation program like TeraTerm, Putty or CoolTerm.

  > Baudrate: 115200

  > Data Bits: 8

  > Parity: None

  > Stop Bits: 1

  > Flow Control: None

- In STMCubeIDE click the green play button to flash and run it on your board. The first time you click it the 'Run Configuration' window will appear. You can just leave it as is and click run.

- Connect the BleuIO Dongle.

# 4. Links

- [Go to BleuIO Manual][1]
- [Go to BleuIO Blog][2]

  [1]: https://www.bleuio.com/getting_started/docs/intro/
  [2]: https://www.bleuio.com/blog/
