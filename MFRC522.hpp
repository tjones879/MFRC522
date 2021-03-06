#pragma once

#include <stdint.h>
#include <Arduino.h> // TODO: Replace arduino libraries

namespace MFRC {

enum class PCDRegister : uint8_t {
    // Page 0: Command and status
    CommandReg              = 0x01 << 1,    // starts and stops command execution
    ComIEnReg               = 0x02 << 1,    // enable and disable interrupt request control bits
    DivIEnReg               = 0x03 << 1,    // enable and disable interrupt request control bits
    ComIrqReg               = 0x04 << 1,    // interrupt request bits
    DivIrqReg               = 0x05 << 1,    // interrupt request bits
    ErrorReg                = 0x06 << 1,    // error bits showing the error status of the last command executed 
    Status1Reg              = 0x07 << 1,    // communication status bits
    Status2Reg              = 0x08 << 1,    // receiver and transmitter status bits
    FIFODataReg             = 0x09 << 1,    // input and output of 64 byte FIFO buffer
    FIFOLevelReg            = 0x0A << 1,    // number of bytes stored in the FIFO buffer
    WaterLevelReg           = 0x0B << 1,    // level for FIFO underflow and overflow warning
    ControlReg              = 0x0C << 1,    // miscellaneous control registers
    BitFramingReg           = 0x0D << 1,    // adjustments for bit-oriented frames
    CollReg                 = 0x0E << 1,    // bit position of the first bit-collision detected on the RF interface

    // Page 1: Command
    ModeReg                 = 0x11 << 1,    // defines general modes for transmitting and receiving 
    TxModeReg               = 0x12 << 1,    // defines transmission data rate and framing
    RxModeReg               = 0x13 << 1,    // defines reception data rate and framing
    TxControlReg            = 0x14 << 1,    // controls the logical behavior of the antenna driver pins TX1 and TX2
    TxASKReg                = 0x15 << 1,    // controls the setting of the transmission modulation
    TxSelReg                = 0x16 << 1,    // selects the internal sources for the antenna driver
    RxSelReg                = 0x17 << 1,    // selects internal receiver settings
    RxThresholdReg          = 0x18 << 1,    // selects thresholds for the bit decoder
    DemodReg                = 0x19 << 1,    // defines demodulator settings
    MfTxReg                 = 0x1C << 1,    // controls some MIFARE communication transmit parameters
    MfRxReg                 = 0x1D << 1,    // controls some MIFARE communication receive parameters
    SerialSpeedReg          = 0x1F << 1,    // selects the speed of the serial UART interface

    // Page 2: Configuration
    CRCResultRegH           = 0x21 << 1,    // shows the MSB and LSB values of the CRC calculation
    CRCResultRegL           = 0x22 << 1,
    ModWidthReg             = 0x24 << 1,    // controls the ModWidth setting?
    RFCfgReg                = 0x26 << 1,    // configures the receiver gain
    GsNReg                  = 0x27 << 1,    // selects the conductance of the antenna driver pins TX1 and TX2 for modulation 
    CWGsPReg                = 0x28 << 1,    // defines the conductance of the p-driver output during periods of no modulation
    ModGsPReg               = 0x29 << 1,    // defines the conductance of the p-driver output during periods of modulation
    TModeReg                = 0x2A << 1,    // defines settings for the internal timer
    TPrescalerReg           = 0x2B << 1,    // the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
    TReloadRegH             = 0x2C << 1,    // defines the 16-bit timer reload value
    TReloadRegL             = 0x2D << 1,
    TCounterValueRegH       = 0x2E << 1,    // shows the 16-bit timer value
    TCounterValueRegL       = 0x2F << 1,

    // Page 3: Test Registers
    TestSel1Reg             = 0x31 << 1,    // general test signal configuration
    TestSel2Reg             = 0x32 << 1,    // general test signal configuration
    TestPinEnReg            = 0x33 << 1,    // enables pin output driver on pins D1 to D7
    TestPinValueReg         = 0x34 << 1,    // defines the values for D1 to D7 when it is used as an I/O bus
    TestBusReg              = 0x35 << 1,    // shows the status of the internal test bus
    AutoTestReg             = 0x36 << 1,    // controls the digital self-test
    VersionReg              = 0x37 << 1,    // shows the software version
    AnalogTestReg           = 0x38 << 1,    // controls the pins AUX1 and AUX2
    TestDAC1Reg             = 0x39 << 1,    // defines the test value for TestDAC1
    TestDAC2Reg             = 0x3A << 1,    // defines the test value for TestDAC2
    TestADCReg              = 0x3B << 1     // shows the value of ADC I and Q channels
};

// MFRC522 commands. Described in chapter 10 of the datasheet.
enum class PCD_Command : uint8_t {
    PCD_Idle                = 0x00,     // no action, cancels current command execution
    PCD_Mem                 = 0x01,     // stores 25 bytes into the internal buffer
    PCD_GenerateRandomID    = 0x02,     // generates a 10-byte random ID number
    PCD_CalcCRC             = 0x03,     // activates the CRC coprocessor or performs a self-test
    PCD_Transmit            = 0x04,     // transmits data from the FIFO buffer
    PCD_NoCmdChange         = 0x07,     // no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
    PCD_Receive             = 0x08,     // activates the receiver circuits
    PCD_Transceive          = 0x0C,     // transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
    PCD_MFAuthent           = 0x0E,     // performs the MIFARE standard authentication as a reader
    PCD_SoftReset           = 0x0F      // resets the MFRC522
};

// MFRC522 RxGain[2:0] masks, defines the receiver's signal voltage gain factor (on the PCD).
// Described in 9.3.3.6 / table 98 of the datasheet
enum class PCD_RxGain : uint8_t {
    RxGain_18dB             = 0x00 << 4,    // 000b - 18 dB, minimum
    RxGain_23dB             = 0x01 << 4,    // 001b - 23 dB
    RxGain_33dB             = 0x04 << 4,    // 100b - 33 dB, average, and typical default
    RxGain_38dB             = 0x05 << 4,    // 101b - 38 dB
    RxGain_43dB             = 0x06 << 4,    // 110b - 43 dB
    RxGain_48dB             = 0x07 << 4     // 111b - 48 dB, maximum
};

// Commands sent to the PICC.
enum class PICC_Command : uint8_t {
    // The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
    PICC_CMD_REQA           = 0x26,     // REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
    PICC_CMD_WUPA           = 0x52,     // Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
    PICC_CMD_CT             = 0x88,     // Cascade Tag. Not really a command, but used during anti collision.
    PICC_CMD_SEL_CL1        = 0x93,     // Anti collision/Select, Cascade Level 1
    PICC_CMD_SEL_CL2        = 0x95,     // Anti collision/Select, Cascade Level 2
    PICC_CMD_SEL_CL3        = 0x97,     // Anti collision/Select, Cascade Level 3
    PICC_CMD_HLTA           = 0x50,     // HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
    PICC_CMD_RATS           = 0xE0,     // Request command for Answer To Reset.
    // The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
    // Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
    // The read/write commands can also be used for MIFARE Ultralight.
    PICC_CMD_MF_AUTH_KEY_A  = 0x60,     // Perform authentication with Key A
    PICC_CMD_MF_AUTH_KEY_B  = 0x61,     // Perform authentication with Key B
    PICC_CMD_MF_READ        = 0x30,     // Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
    PICC_CMD_MF_WRITE       = 0xA0,     // Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
    PICC_CMD_MF_DECREMENT   = 0xC0,     // Decrements the contents of a block and stores the result in the internal data register.
    PICC_CMD_MF_INCREMENT   = 0xC1,     // Increments the contents of a block and stores the result in the internal data register.
    PICC_CMD_MF_RESTORE     = 0xC2,     // Reads the contents of a block into the internal data register.
    PICC_CMD_MF_TRANSFER    = 0xB0,     // Writes the contents of the internal data register to a block.
    // The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
    // The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
    PICC_CMD_UL_WRITE       = 0xA2      // Writes one 4 byte page to the PICC.
};

// MIFARE constants that does not fit anywhere else
enum class MIFARE_Misc : uint8_t {
    MF_ACK                  = 0xA,      // The MIFARE Classic uses a 4 bit ACK/NAK. Any other value than 0xA is NAK.
    MF_KEY_SIZE             = 6         // A Mifare Crypto1 key is 6 bytes.
};

// PICC types we can detect. Remember to update PICC_GetTypeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
enum class PICC_Type : uint8_t {
    PICC_TYPE_UNKNOWN       ,
    PICC_TYPE_ISO_14443_4   ,   // PICC compliant with ISO/IEC 14443-4 
    PICC_TYPE_ISO_18092     ,   // PICC compliant with ISO/IEC 18092 (NFC)
    PICC_TYPE_MIFARE_MINI   ,   // MIFARE Classic protocol, 320 bytes
    PICC_TYPE_MIFARE_1K     ,   // MIFARE Classic protocol, 1KB
    PICC_TYPE_MIFARE_4K     ,   // MIFARE Classic protocol, 4KB
    PICC_TYPE_MIFARE_UL     ,   // MIFARE Ultralight or Ultralight C
    PICC_TYPE_MIFARE_PLUS   ,   // MIFARE Plus
    PICC_TYPE_MIFARE_DESFIRE,   // MIFARE DESFire
    PICC_TYPE_TNP3XXX       ,   // Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
    PICC_TYPE_NOT_COMPLETE  = 0xff  // SAK indicates UID is not complete.
};

// Return codes from the functions in this class. Remember to update GetStatusCodeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
enum class StatusCode : uint8_t {
    STATUS_OK               ,   // Success
    STATUS_ERROR            ,   // Error in communication
    STATUS_COLLISION        ,   // Collission detected
    STATUS_TIMEOUT          ,   // Timeout in communication.
    STATUS_NO_ROOM          ,   // A buffer is not big enough.
    STATUS_INTERNAL_ERROR   ,   // Internal error in the code. Should not happen ;-)
    STATUS_INVALID          ,   // Invalid argument.
    STATUS_CRC_WRONG        ,   // The CRC_A does not match
    STATUS_MIFARE_NACK      = 0xff  // A MIFARE PICC responded with NAK.
};

struct Uid {
    uint8_t size; ///< The number of bytes in the UID.
    std::array<uint8_t, 10> bytes;
    uint8_t sak;  ///< The SAK byte returned from the PICC after successful selection.
};

struct MifareKey {
    std::array<uint8_t, MF_KEY_SIZE> key;
};

template <class SPI>
class MFRC522 {
public:
    // Member variables
    Uid uid;    // Used by PICC_ReadCardSerial().

    MFRC522(const uint8_t chipSelectPin, const uint8_t resetPowerDownPin,
            SPI spiClass, const SPISettings spiSettings = SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0))
            : _chipSelectPin(chipSelectPin), _resetPowerDownPin(resetPowerDownPin),
              _spiClass(spiClass), _spiSettings(spiSettings) {};
    MFRC522() : MFRC522(UNUSED_PIN, UNUSED_PIN) {};

    /**
     * Writes a byte to the specified register in the MFRC522 chip.
     *
     * The interface is described in the datasheet section 8.1.2.
     * @param reg   The register to write to.
     * @param value The value to write.
     */
    void PCD_WriteRegister(PCDRegister reg, uint8_t value);

    /**
     * Writes a number of bytes to the specified register in the MFRC522 chip.
     *
     * The interface is described in the datasheet section 8.1.2.
     * @param reg    The register to write to. One of the PCDRegister enums.
     * @param count  The number of bytes to write to the register.
     * @param values The byte array to be written.
     */
    void PCD_WriteRegister(PCDRegister reg, uint8_t count, uint8_t *values);

    /**
     * Reads a byte from the specified register in the MFRC522 chip.
     *
     * The interface is described in the datasheet section 8.1.2.
     * @param reg The register to read from.
     */
    uint8_t PCD_ReadRegister(PCDRegister reg);

    /**
     * Reads a number of bytes from the specified register in the MFRC522 chip.
     *
     * The interface is described in the datasheet section 8.1.2.
     * @param reg The register to read from.
     * @param count The number of bytes to read
     * @param values Byte array to store the values in
     * @param rxAlign Specify which bit positions to update. Only rxAlign..7 in values[0] are updated.
     */
    void PCD_ReadRegister(PCDRegister reg, uint8_t count, uint8_t *values, uint8_t rxAlign = 0);

    /**
     * Perform a bitwise OR on the given register.
     *
     * @param reg  The register to update.
     * @param mask The bitwise OR mask to update the register.
     */
    void PCD_SetRegisterBitMask(PCDRegister reg, uint8_t mask);

    /**
     * Clears the bits given in mask from register reg.
     *
     * @param reg  The register that will be updated.
     * @param mask The bitmask that will be cleared from reg.
     */
    void PCD_ClearRegisterBitMask(PCDRegister reg, uint8_t mask);

    /**
     * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
     *
     * @param data    Pointer to the data to transfer to the FIFO for CRC calculation.
     * @param length  The number of bytes to transfer.
     * @param result  Pointer to the result buffer. Result is written to result[0..1].
     * @return STATUS_OK on success, STATUS_TIMEOUT otherwise.
     */
    StatusCode PCD_CalculateCRC(uint8_t *data, uint8_t length, uint8_t *result);

    /**
     * Initialize the MFRC522 chip.
     */
    void PCD_Init();

    /**
     * Initialize the MFRC522 chip.
     *
     * @param chipSelectPin      Arduino pin connected to MFRC522's SPI slave select input (Pin 24, NSS, active low)
     * @param resetPowerDownPin  Arduino pin connected to MFRC522's reset and power down input (Pin 6, NRSTPD, active low)
     */
    void PCD_Init(uint8_t chipSelectPin, uint8_t resetPowerDownPin);

    /**
     * Perform a soft reset on the MFRC522 chip and wait for it to be ready again.
     */
    void PCD_Reset();

    /**
     * Turns the antenna on by enabling pins TX1 and TX2.
     * After a reset these pins are disabled.
     */
    void PCD_AntennaOn();

    /**
     * Turns the antenna off by disabling pins TX1 and TX2.
     */
    void PCD_AntennaOff();

    /**
     * Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
     * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
     * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
     * 
     * @return Value of the RxGain, scrubbed to the 3 bits used.
     */
    uint8_t PCD_GetAntennaGain();

    /**
     * Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
     * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
     * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
     */
    void PCD_SetAntennaGain(uint8_t mask);

    /**
     * Performs a self-test of the MFRC522
     * See 16.1.1 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
     *
     * @return Whether or not the test passed. Or false if no firmware reference is available.
     */
    bool PCD_PerformSelfTest();

    void PCD_SoftPowerDown();
    void PCD_SoftPowerUp();

    /**
     * Executes the Transceive command.
     *
     * CRC validation can only be done if backData and backLen are specified.
     *
     * @param sendData   Pointer to the data to transfer to the FIFO.
     * @param sendLen    Number of bytes to transfer to the FIFO.
     * @param backData   nullptr or pointer to buffer if data should be read back after executing the command.
     * @param backLen    In: Max number of bytes to write to *backData. Out: The number of bytes returned.
     * @param validBits  In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
     * @param rxAlign    In: Defines the bit position in backData[0] for the first bit received. Default 0.
     * @param checkCRC   In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
     *
     * @return STATUS_OK on success, STATUS_??? otherwise.
     */
    StatusCode PCD_TransceiveData(uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits = nullptr, uint8_t rxAlign = 0, bool checkCRC = false);

    /**
     * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
     *
     * CRC validation can only be done if backData and backLen are specified.
     *
     * @param command    The command to execute. One of the PCD_Command enums.
     * @param waitIRq    The bits in the ComIrqReg register that signals successful completion of the command.
     * @param sendData   Pointer to the data to transfer to the FIFO.
     * @param sendLen    Number of bytes to transfer to the FIFO.
     * @param backData   nullptr or pointer to buffer if data should be read back after executing the command.
     * @param backLen    In: Max number of bytes to write to *backData. Out: The number of bytes returned.
     * @param validBits  In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
     * @param rxAlign    In: Defines the bit position in backData[0] for the first bit received. Default 0.
     * @param checkCRC   In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
     *
     * @return STATUS_OK on success, STATUS_??? otherwise.
     */
    StatusCode PCD_CommunicateWithPICC(uint8_t command, uint8_t waitIRq, uint8_t *sendData, uint8_t sendLen, uint8_t *backData = nullptr, uint8_t *backLen = nullptr, uint8_t *validBits = nullptr, uint8_t rxAlign = 0, bool checkCRC = false);

    /**
     * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
     * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
     *
     * @return STATUS_OK on success, STATUS_??? otherwise.
     */
    StatusCode PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize);

    /**
     * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
     * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
     *
     * @return STATUS_OK on success, the appropriate status otherwise.
     */
    StatusCode PICC_WakeupA(uint8_t *bufferATQA, uint8_t *bufferSize);

    /**
     * Transmits REQA or WUPA commands.
     * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
     *
     * @param command     The command to send PICC_CMD_REQA or PICC_CMD_WUPA
     * @param bufferATQA  The buffer to store the ATQA.
     * @param bufferSize  The buffer size, at least 2 bytes.
     *                    This is also the number of bytes returned if STATU_OK.
     * @return            STATUS_OK on success, the appropriate status otherwise.
     */
    StatusCode PICC_REQA_or_WUPA(uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize);

    /**
     * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
     * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
     * On success:
     *      - The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
     *      - The UID size and value of the chosen PICC is returned in *uid along with the SAK.
     *
     * A PICC UID consists of 4, 7 or 10 bytes.
     * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
     *      UID size    Number of UID bytes     Cascade levels      Example of PICC
     *      ========    ===================     ==============      ===============
     *      single               4                      1               MIFARE Classic
     *      double               7                      2               MIFARE Ultralight
     *      triple              10                      3               Not currently in use?
     *
     * @param validBits  The number of known UID bits supplied in uid. If set,
     *                   you must also supply uid->size.
     * @return           STATUS_OK on success, STATUS_??? otherwise.
     */
    virtual StatusCode PICC_Select(Uid *uid, uint8_t validBits = 0);

    /**
     * Instructs a PICC in state ACTIVE(*) to go to state HALT.
     *
     * @return STATUS_OK on success, STATUS_??? otherwise.
     */
    StatusCode PICC_HaltA();

    /////////////////////////////////////////////////////////////////////////////////////
    // Functions for communicating with MIFARE PICCs
    /////////////////////////////////////////////////////////////////////////////////////
    StatusCode PCD_Authenticate(uint8_t command, uint8_t blockAddr, MIFARE_Key *key, Uid *uid);

    /**
     * Used to exit the PCD from its authenticated state.
     * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
     */
    void PCD_StopCrypto1();
    StatusCode MIFARE_Read(uint8_t blockAddr, uint8_t *buffer, uint8_t *bufferSize);
    StatusCode MIFARE_Write(uint8_t blockAddr, uint8_t *buffer, uint8_t bufferSize);

    /**
     * Writes a 4 byte page to the active MIFARE Ultralight PICC.
     * @param The page (2-15) to write to.
     * @param the 4 bytes to write
     * @param Buffer size, must be at least 4 bytes. Exactly 4 bytes are written.
     *
     * @return STATUS_OK on success, STATUS_??? otherwise.
     */
    StatusCode MIFARE_Ultralight_Write(uint8_t page, uint8_t *buffer, uint8_t bufferSize);

    /**
     * MIFARE Decrement subtracts the delta from the value of the addressed block, and stores the result in a volatile memory.
     *
     * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
     * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
     * Use MIFARE_Transfer() to store the result in a block.
     *
     * @param blockAddr  The block number
     * @param delta      The number to subtract from the value of the block.
     * @return STATUS_OK on success, STATUS_??? otherwise.
     */
    StatusCode MIFARE_Decrement(uint8_t blockAddr, int32_t delta);

    /**
     * MIFARE Increment adds the delta to the value of the addressed block, and stores the result in a volatile memory.
     * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
     * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
     * Use MIFARE_Transfer() to store the result in a block.
     *
     * @param blockAddr  The block number
     * @param delta      The number to subtract from the value of the block.
     * @return STATUS_OK on success, STATUS_??? otherwise.
     */
    StatusCode MIFARE_Increment(uint8_t blockAddr, int32_t delta);

    /**
     * MIFARE Restore copies the value of the addressed block into a volatile memory.
     * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
     * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
     * Use MIFARE_Transfer() to store the result in a block.
     *
     * @return STATUS_OK on success, STATUS_??? otherwise.
     */
    StatusCode MIFARE_Restore(uint8_t blockAddr);

    /**
     * MIFARE Transfer writes the value stored in the volatile memory into one MIFARE Classic block.
     * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
     * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
     *
     * @return STATUS_OK on success, STATUS_??? otherwise.
     */
    StatusCode MIFARE_Transfer(uint8_t blockAddr);

    /**
     * Helper routine to read the current value from a Value Block.
     *
     * Only for MIFARE Classic and only for blocks in "value block" mode, that
     * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
     * the block must be authenticated before calling this function. 
     *
     * @param[in]   blockAddr   The block (0x00-0xff) number.
     * @param[out]  value       Current value of the Value Block.
     * @return STATUS_OK on success, STATUS_??? otherwise.
     */
    StatusCode MIFARE_GetValue(uint8_t blockAddr, int32_t *value);

    /**
     * Helper routine to write a specific value into a Value Block.
     *
     * Only for MIFARE Classic and only for blocks in "value block" mode, that
     * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
     * the block must be authenticated before calling this function. 
     *
     * @param[in]   blockAddr   The block (0x00-0xff) number.
     * @param[in]   value       New value of the Value Block.
     * @return STATUS_OK on success, STATUS_??? otherwise.
     */
    StatusCode MIFARE_SetValue(uint8_t blockAddr, int32_t value);

    /**
     * Authenticate with a NTAG216.
     *
     * Only for NTAG216. First implemented by Gargantuanman.
     *
     * @param   passWord   password.
     * @param   pACK       result success???.
     * @return STATUS_OK on success, STATUS_??? otherwise.
     */
    StatusCode PCD_NTAG216_AUTH(uint8_t *passWord, uint8_t pACK[]);

    /**
     * Wrapper for MIFARE protocol communication.
     *
     * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
     *     ///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
     *       ///< Number of bytes in sendData.
     *  ///< True => A timeout is also success
     *
     * @return STATUS_OK on success, STATUS_??? otherwise.
     */
    StatusCode PCD_MIFARE_Transceive(uint8_t *sendData, uint8_t sendLen, bool acceptTimeout = false);

    /**
     * Translates the SAK (Select Acknowledge) to a PICC type.
     *
     * @param sak The SAK byte returned from PICC_Select().
     */
    PICC_Type PICC_GetType(uint8_t sak);

    /**
     * Calculate the bit pattern needed for the specified access bits.
     *
     * In the [C1 C2 C3] tuples C1 is MSB (=4) and C3 is LSB (=1).
     * @param accessBitBuffer  Pointer to byte 6, 7 and 8 in the sector trailer. Bytes [0..2] will be set.
     * @param g0               Access bits [C1 C2 C3] for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
     * @param g1               Access bits C1 C2 C3] for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
     * @param g2               Access bits C1 C2 C3] for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
     * @param g3               Access bits C1 C2 C3] for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
     */
    void MIFARE_SetAccessBits(uint8_t *accessBitBuffer, uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3);
protected:
    // Pins
    uint8_t _chipSelectPin;        // Arduino pin connected to MFRC522's SPI slave select input (Pin 24, NSS, active low)
    uint8_t _resetPowerDownPin;    // Arduino pin connected to MFRC522's reset and power down input (Pin 6, NRSTPD, active low)

    // SPI communication
    SPI device;
    const SPISettings _spiSettings; // SPI settings.

    // Functions for communicating with MIFARE PICCs
    StatusCode MIFARE_TwoStepHelper(uint8_t command, uint8_t blockAddr, int32_t data);
    /**
     * Automatically convert the given register address to be recognized as a readable location.
     *
     * @param reg  Register address to eventually be read.
     */
    uint8_t readReg(uint8_t reg);
    /**
     * Automatically convert the given register address to be recognized as a writable location.
     *
     * @param reg  Register address to eventually be written to.
     */
    uint8_t writeReg(uint8_t reg);
};
} /* namespace MFRC */
