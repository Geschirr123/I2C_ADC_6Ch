# I2C_ADC_6Ch

STM32L011 as an I2C-slave 6-channel ADC.

## I2C interface

- Slave address: **0x04** (7-bit), standard mode 100 kHz.
- Registers: `0x0A` = channel enable mask (6 bits, one per channel).

| Operation | Bytes | Effect |
|---|---|---|
| Read (no prior write) | `read N*2` | ADC data, MSB/LSB per enabled channel, packed densely from the lowest enabled channel. `N` = number of enabled channels. |
| Write | `[0x0A, mask]` | Set enabled channels; stored in EEPROM, survives reboot. |
| Write + read | `write [0x0A]`, `read 1` | Read back the mask. Any other register returns `0xFF`. |

Channel → pin: `0`=PA0, `1`=PA1, `2`=PA2, `3`=PA3, `4`=PA5, `5`=PB0.

Example: `mask = 0b100001` enables channels 0 and 5 → a read returns 4 bytes: `CH0_MSB CH0_LSB CH5_MSB CH5_LSB`.

Values are 12-bit right-aligned (0..4095). Reading more bytes than available returns `0xFF` padding. Each read is a snapshot taken at address match, so MSB/LSB are consistent.

## Under the hood

ADC scans the enabled channels continuously, DMA1_CH1 writes samples to RAM in circular mode, the I2C address-match interrupt copies them into MSB/LSB registers, and the I2C interrupt serves them. Mask changes are written to on-chip data EEPROM and the ADC/DMA are reconfigured. LL drivers only, 32 MHz from HSI+PLL.
