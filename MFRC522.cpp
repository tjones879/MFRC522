#include "MFRC522.hpp"
namespace MFRC {

void MFRC522::PCD_WriteRegister(PCDRegister reg, uint8_t value)
{
    _spi.nssLow();
    _spi.send(reg);
    _spi.send(value);
    _spi.nssHigh();
}

void MFRC522::PCD_WriteRegister(PCDRegister reg, uint8_t count, uint8_t *values)
{
    _spi.setLow()(_spi);
    _spi.send(reg);
    for (auto i = 0; i < count; i++)
        _spi.send(values[index]);

    _spi.nssHigh();
}

uint8_t MFRC522::PCD_ReadRegister(PCDRegister reg)
{
    uint8_t value;
    _spi.setLow()(_spi);
    value = _spi.transfer(readReg(reg)) & 0xFF; // MSB := 1 for reading. LSB is not used in address. See 8.1.2.3.
    _spi.send(0); // Send 0 to stop reading.
    _spi.nssHigh();
    return value;
}

void MFRC522::PCD_ReadRegister(PCDRegister reg, uint8_t count, uint8_t *values, uint8_t rxAlign)
{
    if (!count)
        return;

    uint8_t address = readReg(reg);
    uint8_t index = 0;
    _spi.nssLow();
    count--;
    _spi.send(address);

    if (rxAlign) {
        uint8_t mask = (0xFF << rxAlign) & 0xFF;
        uint8_t value = _spi.transfer(address);
        values[0] = (values[0] & ~mask) | (value & mask);
        index++;
    }

    for (index; index < count; index++) {
        values[index] = _spi->transfer(address);
        index++;
    }

    values[index] = _spi->transfer(address);
    _spi.nssHigh();
}

void MFRC522::PCD_SetRegisterBitMask(PCDRegister reg, uint8_t mask)
{
    uint8_t tmp;
    tmp = PCD_ReadRegister(reg);
    PCD_WriteRegister(reg, tmp | mask);
}

void MFRC522::PCD_ClearRegisterBitMask(PCDRegister reg, uint8_t mask)
{
    uint8_t tmp;
    tmp = PCD_ReadRegister(reg);
    PCD_WriteRegister(reg, tmp & (~mask));
}

StatusCode MFRC522::PCD_CalculateCRC(uint8_t *data, uint8_t length, uint8_t *result)
{
    PCD_WriteRegister(CommandReg, PCD_Idle);        // Stop any active command.
    PCD_WriteRegister(DivIrqReg, 0x04);             // Clear the CRCIRq interrupt request bit
    PCD_WriteRegister(FIFOLevelReg, 0x80);          // FlushBuffer = 1, FIFO initialization
    PCD_WriteRegister(FIFODataReg, length, data);   // Write data to the FIFO
    PCD_WriteRegister(CommandReg, PCD_CalcCRC);     // Start the calculation

    // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73μs.
    // TODO check/modify for other architectures than Arduino Uno 16bit

    // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
    for (uint16_t i = 5000; i > 0; i--) {
        // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
        uint8_t n = PCD_ReadRegister(DivIrqReg);
        if (n & 0x04) {                                 // CRCIRq bit set - calculation done
            PCD_WriteRegister(CommandReg, PCD_Idle);    // Stop calculating CRC for new content in the FIFO.
            // Transfer the result from the registers to the result buffer
            result[0] = PCD_ReadRegister(CRCResultRegL);
            result[1] = PCD_ReadRegister(CRCResultRegH);
            return STATUS_OK;
        }
    }
    // 89ms passed and nothing happend. Communication with the MFRC522 might be down.
    return STATUS_TIMEOUT;
}

void MFRC522::PCD_Init() {
    bool hardReset = false;

    // Set the chipSelectPin as digital output, do not select the slave yet
    pinMode(_chipSelectPin, OUTPUT);
    digitalWrite(_chipSelectPin, HIGH);

    // If a valid pin number has been set, pull device out of power down / reset state.
    if (_resetPowerDownPin != UNUSED_PIN) {
        // First set the resetPowerDownPin as digital input, to check the MFRC522 power down mode.
        pinMode(_resetPowerDownPin, INPUT);

        if (digitalRead(_resetPowerDownPin) == LOW) {   // The MFRC522 chip is in power down mode.
            pinMode(_resetPowerDownPin, OUTPUT);        // Now set the resetPowerDownPin as digital output.
            digitalWrite(_resetPowerDownPin, HIGH);     // Exit power down mode. This triggers a hard reset.
            // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
            delay(50);
            hardReset = true;
        }
    }

    if (!hardReset) { // Perform a soft reset if we haven't triggered a hard reset above.
        PCD_Reset();
    }

    // Reset baud rates
    PCD_WriteRegister(TxModeReg, 0x00);
    PCD_WriteRegister(RxModeReg, 0x00);
    // Reset ModWidthReg
    PCD_WriteRegister(ModWidthReg, 0x26);

    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    PCD_WriteRegister(TModeReg, 0x80);          // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    PCD_WriteRegister(TPrescalerReg, 0xA9);     // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
    PCD_WriteRegister(TReloadRegH, 0x03);       // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    PCD_WriteRegister(TReloadRegL, 0xE8);

    PCD_WriteRegister(TxASKReg, 0x40);      // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    PCD_WriteRegister(ModeReg, 0x3D);       // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    PCD_AntennaOn();                        // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
}

void MFRC522::PCD_Init(uint8_t chipSelectPin, uint8_t resetPowerDownPin)
{
    _chipSelectPin = chipSelectPin;
    _resetPowerDownPin = resetPowerDownPin;
    // Set the chipSelectPin as digital output, do not select the slave yet
    PCD_Init();
}

void MFRC522::PCD_Reset()
{
    PCD_WriteRegister(CommandReg, PCD_SoftReset);   // Issue the SoftReset command.
    // The datasheet does not mention how long the SoftRest command takes to complete.
    // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
    // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
    uint8_t count = 0;
    do {
        // Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
        delay(50);
    } while ((PCD_ReadRegister(CommandReg) & (1 << 4)) && (++count) < 3);
}

void MFRC522::PCD_AntennaOn()
{
    uint8_t value = PCD_ReadRegister(TxControlReg);
    if ((value & 0x03) != 0x03) {
        PCD_WriteRegister(TxControlReg, value | 0x03);
    }
}

void MFRC522::PCD_AntennaOff()
{
    PCD_ClearRegisterBitMask(TxControlReg, 0x03);
}

uint8_t MFRC522::PCD_GetAntennaGain()
{
    return PCD_ReadRegister(RFCfgReg) & (0x07<<4);
}

void MFRC522::PCD_SetAntennaGain(uint8_t mask)
{
    if (PCD_GetAntennaGain() != mask) {                     // only bother if there is a change
        PCD_ClearRegisterBitMask(RFCfgReg, (0x07<<4));      // clear needed to allow 000 pattern
        PCD_SetRegisterBitMask(RFCfgReg, mask & (0x07<<4)); // only set RxGain[2:0] bits
    }
}

bool MFRC522::PCD_PerformSelfTest() {
    // This follows directly the steps outlined in 16.1.1
    // 1. Perform a soft reset.
    PCD_Reset();

    // 2. Clear the internal buffer by writing 25 bytes of 00h
    uint8_t ZEROES[25] = {0x00};
    PCD_WriteRegister(FIFOLevelReg, 0x80);      // flush the FIFO buffer
    PCD_WriteRegister(FIFODataReg, 25, ZEROES); // write 25 bytes of 00h to FIFO
    PCD_WriteRegister(CommandReg, PCD_Mem);     // transfer to internal buffer

    // 3. Enable self-test
    PCD_WriteRegister(AutoTestReg, 0x09);

    // 4. Write 00h to FIFO buffer
    PCD_WriteRegister(FIFODataReg, 0x00);

    // 5. Start self-test by issuing the CalcCRC command
    PCD_WriteRegister(CommandReg, PCD_CalcCRC);

    // 6. Wait for self-test to complete
    uint8_t n;
    for (uint8_t i = 0; i < 0xFF; i++) {
        // The datasheet does not specify exact completion condition except
        // that FIFO buffer should contain 64 bytes.
        // While selftest is initiated by CalcCRC command
        // it behaves differently from normal CRC computation,
        // so one can't reliably use DivIrqReg to check for completion.
        // It is reported that some devices does not trigger CRCIRq flag
        // during selftest.
        n = PCD_ReadRegister(FIFOLevelReg);
        if (n >= 64) {
            break;
        }
    }
    PCD_WriteRegister(CommandReg, PCD_Idle);        // Stop calculating CRC for new content in the FIFO.

    // 7. Read out resulting 64 bytes from the FIFO buffer.
    uint8_t result[64];
    PCD_ReadRegister(FIFODataReg, 64, result, 0);

    // Auto self-test done
    // Reset AutoTestReg register to be 0 again. Required for normal operation.
    PCD_WriteRegister(AutoTestReg, 0x00);

    // Determine firmware version (see section 9.3.4.8 in spec)
    uint8_t version = PCD_ReadRegister(VersionReg);

    // Pick the appropriate reference values
    const uint8_t *reference;
    switch (version) {
        case 0x88:  // Fudan Semiconductor FM17522 clone
            reference = FM17522_firmware_reference;
            break;
        case 0x90:  // Version 0.0
            reference = MFRC522_firmware_referenceV0_0;
            break;
        case 0x91:  // Version 1.0
            reference = MFRC522_firmware_referenceV1_0;
            break;
        case 0x92:  // Version 2.0
            reference = MFRC522_firmware_referenceV2_0;
            break;
        default:    // Unknown version
            return false; // abort test
    }

    // Verify that the results match up to our expectations
    for (uint8_t i = 0; i < 64; i++) {
        if (result[i] != pgm_read_byte(&(reference[i]))) {
            return false;
        }
    }

    // Test passed; all is good.
    return true;
}

//IMPORTANT NOTE!!!!
//Calling any other function that uses CommandReg will disable soft power down mode !!!
//For more details about power control, refer to the datasheet - page 33 (8.6)

void MFRC522::PCD_SoftPowerDown()
{
    //Note : Only soft power down mode is available throught software
    uint8_t val = PCD_ReadRegister(CommandReg); // Read state of the command register 
    val |= (1<<4);// set PowerDown bit ( bit 4 ) to 1 
    PCD_WriteRegister(CommandReg, val);//write new value to the command register
}

void MFRC522::PCD_SoftPowerUp()
{
    uint8_t val = PCD_ReadRegister(CommandReg); // Read state of the command register 
    val &= ~(1<<4);// set PowerDown bit ( bit 4 ) to 0 
    PCD_WriteRegister(CommandReg, val);//write new value to the command register
    // wait until PowerDown bit is cleared (this indicates end of wake up procedure) 
    const uint32_t timeout = (uint32_t)millis() + 500;// create timer for timeout (just in case) 

    while(millis()<=timeout){ // set timeout to 500 ms 
        val = PCD_ReadRegister(CommandReg);// Read state of the command register
        if(!(val & (1<<4))){ // if powerdown bit is 0 
            break;// wake up procedure is finished 
        }
    }
}

StatusCode MFRC522::PCD_TransceiveData(uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, bool checkCRC)
{
    uint8_t waitIRq = 0x30;        // RxIRq and IdleIRq
    return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}

StatusCode MFRC522::PCD_CommunicateWithPICC(uint8_t command, uint8_t waitIRq, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, bool checkCRC)
{
    // Prepare values for BitFramingReg
    uint8_t txLastBits = validBits ? *validBits : 0;
    uint8_t bitFraming = (rxAlign << 4) + txLastBits;      // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

    PCD_WriteRegister(CommandReg, PCD_Idle);            // Stop any active command.
    PCD_WriteRegister(ComIrqReg, 0x7F);                 // Clear all seven interrupt request bits
    PCD_WriteRegister(FIFOLevelReg, 0x80);              // FlushBuffer = 1, FIFO initialization
    PCD_WriteRegister(FIFODataReg, sendLen, sendData);  // Write sendData to the FIFO
    PCD_WriteRegister(BitFramingReg, bitFraming);       // Bit adjustments
    PCD_WriteRegister(CommandReg, command);             // Execute the command
    if (command == PCD_Transceive) {
        PCD_SetRegisterBitMask(BitFramingReg, 0x80);    // StartSend=1, transmission of data starts
    }

    // Wait for the command to complete.
    // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
    // Each iteration of the do-while-loop takes 17.86μs.
    // TODO check/modify for other architectures than Arduino Uno 16bit
    uint16_t i;
    for (i = 2000; i > 0; i--) {
        uint8_t n = PCD_ReadRegister(ComIrqReg);   // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
        if (n & waitIRq) {                  // One of the interrupts that signal success has been set.
            break;
        }
        if (n & 0x01) {                     // Timer interrupt - nothing received in 25ms
            return STATUS_TIMEOUT;
        }
    }
    // 35.7ms and nothing happend. Communication with the MFRC522 might be down.
    if (i == 0) {
        return STATUS_TIMEOUT;
    }

    // Stop now if any errors except collisions were detected.
    uint8_t errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
    if (errorRegValue & 0x13) {  // BufferOvfl ParityErr ProtocolErr
        return STATUS_ERROR;
    }

    uint8_t _validBits = 0;

    // If the caller wants data back, get it from the MFRC522.
    if (backData && backLen) {
        uint8_t n = PCD_ReadRegister(FIFOLevelReg);    // Number of bytes in the FIFO
        if (n > *backLen) {
            return STATUS_NO_ROOM;
        }
        *backLen = n;                                           // Number of bytes returned
        PCD_ReadRegister(FIFODataReg, n, backData, rxAlign);    // Get received data from FIFO
        _validBits = PCD_ReadRegister(ControlReg) & 0x07;       // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
        if (validBits) {
            *validBits = _validBits;
        }
    }

    // Tell about collisions
    if (errorRegValue & 0x08) {     // CollErr
        return STATUS_COLLISION;
    }

    // Perform CRC_A validation if requested.
    if (backData && backLen && checkCRC) {
        // In this case a MIFARE Classic NAK is not OK.
        if (*backLen == 1 && _validBits == 4) {
            return STATUS_MIFARE_NACK;
        }
        // We need at least the CRC_A value and all 8 bits of the last byte must be received.
        if (*backLen < 2 || _validBits != 0) {
            return STATUS_CRC_WRONG;
        }
        // Verify CRC_A - do our own calculation and store the control in controlBuffer.
        uint8_t controlBuffer[2];
        MFRC522::StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
        if (status != STATUS_OK) {
            return status;
        }
        if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
            return STATUS_CRC_WRONG;
        }
    }

    return STATUS_OK;
}

StatusCode MFRC522::PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize)
{
    return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
}

StatusCode MFRC522::PICC_WakeupA(uint8_t *bufferATQA, uint8_t *bufferSize)
{
    return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
}

MFRC522::StatusCode MFRC522::PICC_REQA_or_WUPA(uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize)
{
    uint8_t validBits;
    StatusCode status;

    if (bufferATQA == nullptr || *bufferSize < 2) { // The ATQA response is 2 bytes long.
        return STATUS_NO_ROOM;
    }
    PCD_ClearRegisterBitMask(CollReg, 0x80);        // ValuesAfterColl=1 => Bits received after collision are cleared.
    validBits = 7;                                  // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
    status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits);
    if (status != STATUS_OK) {
        return status;
    }
    if (*bufferSize != 2 || validBits != 0) {       // ATQA must be exactly 16 bits.
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

MFRC522::StatusCode MFRC522::PICC_Select(Uid *uid, uint8_t validBits)
{
    bool uidComplete;
    bool selectDone;
    bool useCascadeTag;
    uint8_t cascadeLevel = 1;
    MFRC522::StatusCode result;
    uint8_t count;
    uint8_t checkBit;
    uint8_t index;
    uint8_t uidIndex;                  // The first index in uid->uidByte[] that is used in the current Cascade Level.
    int8_t currentLevelKnownBits;       // The number of known UID bits in the current Cascade Level.
    uint8_t buffer[9];                 // The SELECT/ANTICOLLISION commands uses a 7 uint8_t standard frame + 2 bytes CRC_A
    uint8_t bufferUsed;                // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
    uint8_t rxAlign;                   // Used in BitFramingReg. Defines the bit position for the first bit received.
    uint8_t txLastBits;                // Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
    uint8_t *responseBuffer;
    uint8_t responseLength;

    // Description of buffer structure:
    //      Byte 0: SEL                 Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
    //      Byte 1: NVB                 Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
    //      Byte 2: UID-data or CT      See explanation below. CT means Cascade Tag.
    //      Byte 3: UID-data
    //      Byte 4: UID-data
    //      Byte 5: UID-data
    //      Byte 6: BCC                 Block Check Character - XOR of bytes 2-5
    //      Byte 7: CRC_A
    //      Byte 8: CRC_A
    // The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
    //
    // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
    //      UID size    Cascade level   Byte2   Byte3   Byte4   Byte5
    //      ========    =============   =====   =====   =====   =====
    //       4 bytes        1           uid0    uid1    uid2    uid3
    //       7 bytes        1           CT      uid0    uid1    uid2
    //                      2           uid3    uid4    uid5    uid6
    //      10 bytes        1           CT      uid0    uid1    uid2
    //                      2           CT      uid3    uid4    uid5
    //                      3           uid6    uid7    uid8    uid9

    // Sanity checks
    if (validBits > 80) {
        return STATUS_INVALID;
    }

    // Prepare MFRC522
    PCD_ClearRegisterBitMask(CollReg, 0x80);        // ValuesAfterColl=1 => Bits received after collision are cleared.

    // Repeat Cascade Level loop until we have a complete UID.
    uidComplete = false;
    while (!uidComplete) {
        // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
        switch (cascadeLevel) {
            case 1:
                buffer[0] = PICC_CMD_SEL_CL1;
                uidIndex = 0;
                useCascadeTag = validBits && uid->size > 4; // When we know that the UID has more than 4 bytes
                break;
            case 2:
                buffer[0] = PICC_CMD_SEL_CL2;
                uidIndex = 3;
                useCascadeTag = validBits && uid->size > 7; // When we know that the UID has more than 7 bytes
                break;
            case 3:
                buffer[0] = PICC_CMD_SEL_CL3;
                uidIndex = 6;
                useCascadeTag = false;                      // Never used in CL3.
                break;
            default:
                return STATUS_INTERNAL_ERROR;
                break;
        }

        // How many UID bits are known in this Cascade Level?
        currentLevelKnownBits = validBits - (8 * uidIndex);
        if (currentLevelKnownBits < 0) {
            currentLevelKnownBits = 0;
        }
        // Copy the known bits from uid->uidByte[] to buffer[]
        index = 2; // destination index in buffer[]
        if (useCascadeTag) {
            buffer[index++] = PICC_CMD_CT;
        }
        uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
        if (bytesToCopy) {
            uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
            if (bytesToCopy > maxBytes) {
                bytesToCopy = maxBytes;
            }
            for (count = 0; count < bytesToCopy; count++) {
                buffer[index++] = uid->uidByte[uidIndex + count];
            }
        }
        // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
        if (useCascadeTag) {
            currentLevelKnownBits += 8;
        }

        // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
        selectDone = false;
        while (!selectDone) {
            // Find out how many bits and bytes to send and receive.
            if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
                //Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
                // Calculate BCC - Block Check Character
                buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                // Calculate CRC_A
                result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
                if (result != STATUS_OK) {
                    return result;
                }
                txLastBits      = 0; // 0 => All 8 bits are valid.
                bufferUsed      = 9;
                // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
                responseBuffer  = &buffer[6];
                responseLength  = 3;
            } else { // This is an ANTICOLLISION.
                //Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                txLastBits      = currentLevelKnownBits % 8;
                count           = currentLevelKnownBits / 8;    // Number of whole bytes in the UID part.
                index           = 2 + count;                    // Number of whole bytes: SEL + NVB + UIDs
                buffer[1]       = (index << 4) + txLastBits;    // NVB - Number of Valid Bits
                bufferUsed      = index + (txLastBits ? 1 : 0);
                // Store response in the unused part of buffer
                responseBuffer  = &buffer[index];
                responseLength  = sizeof(buffer) - index;
            }

            // Set bit adjustments
            rxAlign = txLastBits;                                           // Having a separate variable is overkill. But it makes the next line easier to read.
            PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

            // Transmit the buffer and receive the response.
            result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
            if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
                uint8_t valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                if (valueOfCollReg & 0x20) { // CollPosNotValid
                    return STATUS_COLLISION; // Without a valid collision position we cannot continue
                }
                uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
                if (collisionPos == 0) {
                    collisionPos = 32;
                }
                if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
                    return STATUS_INTERNAL_ERROR;
                }
                // Choose the PICC with the bit set.
                currentLevelKnownBits   = collisionPos;
                count           = currentLevelKnownBits % 8; // The bit to modify
                checkBit        = (currentLevelKnownBits - 1) % 8;
                index           = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
                buffer[index]   |= (1 << checkBit);
            }
            else if (result != STATUS_OK) {
                return result;
            }
            else { // STATUS_OK
                if (currentLevelKnownBits >= 32) { // This was a SELECT.
                    selectDone = true; // No more anticollision 
                    // We continue below outside the while.
                }
                else { // This was an ANTICOLLISION.
                    // We now have all 32 bits of the UID in this Cascade Level
                    currentLevelKnownBits = 32;
                    // Run loop again to do the SELECT.
                }
            }
        }

        // We do not check the CBB - it was constructed by us above.
        // Copy the found UID bytes from buffer[] to uid->uidByte[]
        index           = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
        bytesToCopy     = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
        for (count = 0; count < bytesToCopy; count++) {
            uid->uidByte[uidIndex + count] = buffer[index++];
        }

        // Check response SAK (Select Acknowledge)
        if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
            return STATUS_ERROR;
        }
        // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
        result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
        if (result != STATUS_OK) {
            return result;
        }
        if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
            return STATUS_CRC_WRONG;
        }
        if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
            cascadeLevel++;
        }
        else {
            uidComplete = true;
            uid->sak = responseBuffer[0];
        }
    }

    // Set correct uid->size
    uid->size = 3 * cascadeLevel + 1;

    return STATUS_OK;
}

StatusCode MFRC522::PICC_HaltA() {
    StatusCode result;
    uint8_t buffer[4];

    // Build command buffer
    buffer[0] = PICC_CMD_HLTA;
    buffer[1] = 0;
    // Calculate CRC_A
    result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
    if (result != STATUS_OK) {
        return result;
    }

    // Send the command.
    // The standard says:
    //      If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
    //      HLTA command, this response shall be interpreted as 'not acknowledge'.
    // We interpret that this way: Only STATUS_TIMEOUT is a success.
    result = PCD_TransceiveData(buffer, sizeof(buffer), nullptr, 0);
    if (result == STATUS_TIMEOUT) {
        return STATUS_OK;
    }
    if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
        return STATUS_ERROR;
    }
    return result;
}

/**
 * Executes the MFRC522 MFAuthent command.
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522 datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 * 
 * All keys are set to FFFFFFFFFFFFh at chip delivery.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
 */
MFRC522::StatusCode MFRC522::PCD_Authenticate(uint8_t command,     ///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
                                            uint8_t blockAddr,     ///< The block number. See numbering in the comments in the .h file.
                                            MIFARE_Key *key,    ///< Pointer to the Crypteo1 key to use (6 bytes)
                                            Uid *uid            ///< Pointer to Uid struct. The first 4 bytes of the UID is used.
                                            ) {
    uint8_t waitIRq = 0x10;        // IdleIRq
    
    // Build command buffer
    uint8_t sendData[12];
    sendData[0] = command;
    sendData[1] = blockAddr;
    for (uint8_t i = 0; i < MF_KEY_SIZE; i++) {    // 6 key bytes
        sendData[2+i] = key->keyByte[i];
    }
    // Use the last uid bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
    // section 3.2.5 "MIFARE Classic Authentication".
    // The only missed case is the MF1Sxxxx shortcut activation,
    // but it requires cascade tag (CT) byte, that is not part of uid.
    for (uint8_t i = 0; i < 4; i++) {              // The last 4 bytes of the UID
        sendData[8+i] = uid->uidByte[i+uid->size-4];
    }
    
    // Start the authentication.
    return PCD_CommunicateWithPICC(PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData));
} // End PCD_Authenticate()

void MFRC522::PCD_StopCrypto1()
{
    // Clear MFCrypto1On bit
    PCD_ClearRegisterBitMask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
}

/**
 * Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
 * 
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 * 
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 * 
 * The buffer must be at least 18 bytes because a CRC_A is also returned.
 * Checks the CRC_A before returning STATUS_OK.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::MIFARE_Read(   uint8_t blockAddr,     ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
                                            uint8_t *buffer,       ///< The buffer to store the data in
                                            uint8_t *bufferSize    ///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
                                        ) {
    MFRC522::StatusCode result;
    
    // Sanity check
    if (buffer == nullptr || *bufferSize < 18) {
        return STATUS_NO_ROOM;
    }
    
    // Build command buffer
    buffer[0] = PICC_CMD_MF_READ;
    buffer[1] = blockAddr;
    // Calculate CRC_A
    result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
    if (result != STATUS_OK) {
        return result;
    }
    
    // Transmit the buffer and receive the response, validate CRC_A.
    return PCD_TransceiveData(buffer, 4, buffer, bufferSize, nullptr, 0, true);
} // End MIFARE_Read()

/**
 * Writes 16 bytes to the active PICC.
 * 
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 * 
 * For MIFARE Ultralight the operation is called "COMPATIBILITY WRITE".
 * Even though 16 bytes are transferred to the Ultralight PICC, only the least significant 4 bytes (bytes 0 to 3)
 * are written to the specified address. It is recommended to set the remaining bytes 04h to 0Fh to all logic 0.
 * * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::MIFARE_Write(  uint8_t blockAddr, ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
                                            uint8_t *buffer,   ///< The 16 bytes to write to the PICC
                                            uint8_t bufferSize ///< Buffer size, must be at least 16 bytes. Exactly 16 bytes are written.
                                        ) {
    MFRC522::StatusCode result;
    
    // Sanity check
    if (buffer == nullptr || bufferSize < 16) {
        return STATUS_INVALID;
    }
    
    // Mifare Classic protocol requires two communications to perform a write.
    // Step 1: Tell the PICC we want to write to block blockAddr.
    uint8_t cmdBuffer[2];
    cmdBuffer[0] = PICC_CMD_MF_WRITE;
    cmdBuffer[1] = blockAddr;
    result = PCD_MIFARE_Transceive(cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
    if (result != STATUS_OK) {
        return result;
    }
    
    // Step 2: Transfer the data
    result = PCD_MIFARE_Transceive(buffer, bufferSize); // Adds CRC_A and checks that the response is MF_ACK.
    if (result != STATUS_OK) {
        return result;
    }
    
    return STATUS_OK;
} // End MIFARE_Write()

StatusCode MFRC522::MIFARE_Ultralight_Write(uint8_t page, uint8_t *buffer, uint8_t bufferSize)
{
    StatusCode result;

    // Sanity check
    if (buffer == nullptr || bufferSize < 4) {
        return STATUS_INVALID;
    }

    // Build commmand buffer
    uint8_t cmdBuffer[6];
    cmdBuffer[0] = PICC_CMD_UL_WRITE;
    cmdBuffer[1] = page;
    memcpy(&cmdBuffer[2], buffer, 4);

    // Perform the write
    result = PCD_MIFARE_Transceive(cmdBuffer, 6); // Adds CRC_A and checks that the response is MF_ACK.
    if (result != STATUS_OK) {
        return result;
    }
    return STATUS_OK;
}

StatusCode MFRC522::MIFARE_Decrement(uint8_t blockAddr, int32_t delta)
{
    return MIFARE_TwoStepHelper(PICC_CMD_MF_DECREMENT, blockAddr, delta);
}

StatusCode MFRC522::MIFARE_Increment(uint8_t blockAddr, int32_t delta)
{
    return MIFARE_TwoStepHelper(PICC_CMD_MF_INCREMENT, blockAddr, delta);
}

StatusCode MFRC522::MIFARE_Restore(uint8_t blockAddr)
{
    // The datasheet describes Restore as a two step operation, but does not explain what data to transfer in step 2.
    // Doing only a single step does not work, so I chose to transfer 0L in step two.
    return MIFARE_TwoStepHelper(PICC_CMD_MF_RESTORE, blockAddr, 0L);
}

/**
 * Helper function for the two-step MIFARE Classic protocol operations Decrement, Increment and Restore.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode MFRC522::MIFARE_TwoStepHelper(  uint8_t command,   ///< The command to use
                                                    uint8_t blockAddr, ///< The block (0-0xff) number.
                                                    int32_t data        ///< The data to transfer in step 2
                                                    ) {
    StatusCode result;
    uint8_t cmdBuffer[2]; // We only need room for 2 bytes.

    // Step 1: Tell the PICC the command and block address
    cmdBuffer[0] = command;
    cmdBuffer[1] = blockAddr;
    result = PCD_MIFARE_Transceive( cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
    if (result != STATUS_OK) {
        return result;
    }

    // Step 2: Transfer the data
    result = PCD_MIFARE_Transceive( (uint8_t *)&data, 4, true); // Adds CRC_A and accept timeout as success.
    if (result != STATUS_OK) {
        return result;
    }

    return STATUS_OK;
}

StatusCode MFRC522::MIFARE_Transfer(uint8_t blockAddr)
{
    StatusCode result;
    uint8_t cmdBuffer[2]; // We only need room for 2 bytes.

    // Tell the PICC we want to transfer the result into block blockAddr.
    cmdBuffer[0] = PICC_CMD_MF_TRANSFER;
    cmdBuffer[1] = blockAddr;
    result = PCD_MIFARE_Transceive( cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
    if (result != STATUS_OK) {
        return result;
    }
    return STATUS_OK;
}

StatusCode MFRC522::MIFARE_GetValue(uint8_t blockAddr, int32_t *value)
{
    MFRC522::StatusCode status;
    uint8_t buffer[18];
    uint8_t size = sizeof(buffer);

    // Read the block
    status = MIFARE_Read(blockAddr, buffer, &size);
    if (status == STATUS_OK) {
        // Extract the value
        *value = (int32_t(buffer[3])<<24) | (int32_t(buffer[2])<<16) | (int32_t(buffer[1])<<8) | int32_t(buffer[0]);
    }
    return status;
}

StatusCode MFRC522::MIFARE_SetValue(uint8_t blockAddr, int32_t value)
{
    uint8_t buffer[18];

    // Translate the int32_t into 4 bytes; repeated 2x in value block
    buffer[0] = buffer[ 8] = (value & 0xFF);
    buffer[1] = buffer[ 9] = (value & 0xFF00) >> 8;
    buffer[2] = buffer[10] = (value & 0xFF0000) >> 16;
    buffer[3] = buffer[11] = (value & 0xFF000000) >> 24;
    // Inverse 4 bytes also found in value block
    buffer[4] = ~buffer[0];
    buffer[5] = ~buffer[1];
    buffer[6] = ~buffer[2];
    buffer[7] = ~buffer[3];
    // Address 2x with inverse address 2x
    buffer[12] = buffer[14] = blockAddr;
    buffer[13] = buffer[15] = ~blockAddr;

    // Write the whole data block
    return MIFARE_Write(blockAddr, buffer, 16);
}

StatusCode MFRC522::PCD_NTAG216_AUTH(uint8_t *passWord, uint8_t pACK[]) //Authenticate with 32bit password
{
    // TODO: Fix cmdBuffer length and rxlength. They really should match.
    //       (Better still, rxlength should not even be necessary.)

    StatusCode result;
    uint8_t                cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.

    cmdBuffer[0] = 0x1B; //Comando de autentificacion

    for (uint8_t i = 0; i<4; i++)
        cmdBuffer[i+1] = passWord[i];

    result = PCD_CalculateCRC(cmdBuffer, 5, &cmdBuffer[5]);

    if (result!=STATUS_OK) {
        return result;
    }

    // Transceive the data, store the reply in cmdBuffer[]
    uint8_t waitIRq        = 0x30; // RxIRq and IdleIRq
//  uint8_t cmdBufferSize  = sizeof(cmdBuffer);
    uint8_t validBits      = 0;
    uint8_t rxlength       = 5;
    result = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, 7, cmdBuffer, &rxlength, &validBits);

    pACK[0] = cmdBuffer[0];
    pACK[1] = cmdBuffer[1];

    if (result!=STATUS_OK) {
        return result;
    }

    return STATUS_OK;
}

StatusCode MFRC522::PCD_MIFARE_Transceive(uint8_t *sendData, uint8_t sendLen, bool acceptTimeout)
{
    StatusCode result;
    uint8_t cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.

    // Sanity check
    if (sendData == nullptr || sendLen > 16) {
        return STATUS_INVALID;
    }

    // Copy sendData[] to cmdBuffer[] and add CRC_A
    memcpy(cmdBuffer, sendData, sendLen);
    result = PCD_CalculateCRC(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
    if (result != STATUS_OK) { 
        return result;
    }
    sendLen += 2;

    // Transceive the data, store the reply in cmdBuffer[]
    uint8_t waitIRq = 0x30;        // RxIRq and IdleIRq
    uint8_t cmdBufferSize = sizeof(cmdBuffer);
    uint8_t validBits = 0;
    result = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &cmdBufferSize, &validBits);
    if (acceptTimeout && result == STATUS_TIMEOUT) {
        return STATUS_OK;
    }
    if (result != STATUS_OK) {
        return result;
    }
    // The PICC must reply with a 4 bit ACK
    if (cmdBufferSize != 1 || validBits != 4) {
        return STATUS_ERROR;
    }
    if (cmdBuffer[0] != MF_ACK) {
        return STATUS_MIFARE_NACK;
    }
    return STATUS_OK;
}

PICC_Type MFRC522::PICC_GetType(uint8_t sak)
{
    // http://www.nxp.com/documents/application_note/AN10833.pdf 
    // 3.2 Coding of Select Acknowledge (SAK)
    // ignore 8-bit (iso14443 starts with LSBit = bit 1)
    // fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
    sak &= 0x7F;
    switch (sak) {
        case 0x04:  return PICC_TYPE_NOT_COMPLETE;  // UID not complete
        case 0x09:  return PICC_TYPE_MIFARE_MINI;
        case 0x08:  return PICC_TYPE_MIFARE_1K;
        case 0x18:  return PICC_TYPE_MIFARE_4K;
        case 0x00:  return PICC_TYPE_MIFARE_UL;
        case 0x10:
        case 0x11:  return PICC_TYPE_MIFARE_PLUS;
        case 0x01:  return PICC_TYPE_TNP3XXX;
        case 0x20:  return PICC_TYPE_ISO_14443_4;
        case 0x40:  return PICC_TYPE_ISO_18092;
        default:    return PICC_TYPE_UNKNOWN;
    }
}

void MFRC522::MIFARE_SetAccessBits(uint8_t *accessBitBuffer, uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3)
{
    uint8_t c1 = ((g3 & 4) << 1) | ((g2 & 4) << 0) | ((g1 & 4) >> 1) | ((g0 & 4) >> 2);
    uint8_t c2 = ((g3 & 2) << 2) | ((g2 & 2) << 1) | ((g1 & 2) << 0) | ((g0 & 2) >> 1);
    uint8_t c3 = ((g3 & 1) << 3) | ((g2 & 1) << 2) | ((g1 & 1) << 1) | ((g0 & 1) << 0);

    accessBitBuffer[0] = (~c2 & 0xF) << 4 | (~c1 & 0xF);
    accessBitBuffer[1] =          c1 << 4 | (~c3 & 0xF);
    accessBitBuffer[2] =          c3 << 4 | c2;
}


uint8_t MFRC522::readReg(uint8_t reg)
{
    return 0x80 | reg;
}

uint8_t MFRC522::writeReg(uint8_t reg)
{
    return reg;
}
} /* namespace MFRC */
