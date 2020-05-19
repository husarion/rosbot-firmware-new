#include <mbed.h>

/**
 * @brief  OTP memory start address
 */
#define OTP_START_ADDR		(0x1FFF7800)

/**
 * @brief  OTP memory lock address
 */
#define OTP_LOCK_ADDR		(0x1FFF7A00)
	
/**
 * @brief  Number of OTP blocks 
 */
#define OTP_BLOCKS			16

/**
 * @brief  Number of bytes in one block
 */
#define OTP_BYTES_IN_BLOCK	32

/**
 * @brief  Number of all OTP bytes
 */
#define OTP_SIZE			(OTP_BLOCKS * OTP_BYTES_IN_BLOCK)

/**
 * @brief  Checks if block is locked or not
 * @param  block: OTP block number, 0 to 15 is allowed
 * @retval Block lock status
 *            - 0: Block is not locked
 *            - > 0: Block locked
 */
#define OTP_BlockLocked(block)	((*(__IO uint8_t *) (OTP_LOCK_ADDR + block)) == 0x00 ? 1 : 0)

uint8_t OTP_Read(uint8_t block, uint8_t byte) {
	uint8_t data;
	
	/* Check input parameters */
	if (
		block >= OTP_BLOCKS ||
		byte >= OTP_BYTES_IN_BLOCK
	) {
		/* Invalid parameters */
		return 0;
	}
	
	/* Get value */
	data = *(__IO uint8_t *)(OTP_START_ADDR + block * OTP_BYTES_IN_BLOCK + byte);
	
	/* Return data */
	return data;
}

static char * toHex(char * buffer, uint8_t data)
{
    static const char HEX[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    *(buffer++) = HEX[data >> 4];
    *(buffer++) = HEX[data & 0x0F];
    return buffer;
}

void OTP_Dump()
{
    char buffer_hex[64]; 
    char buffer_char[64]; 
    const uint32_t OTP_ADDR_STOP = OTP_START_ADDR + (OTP_BLOCKS * OTP_BYTES_IN_BLOCK); 
    uint32_t addr = OTP_START_ADDR;
    printf("OTP Data:\r\n");
    while(addr < OTP_ADDR_STOP)
    {
        char * buffer_hex_ptr = buffer_hex;
        char * buffer_char_ptr = buffer_char;
        for(int j=0;j<16;j++)
        {
            uint8_t data = *(__IO uint8_t *)(addr+j);
            buffer_hex_ptr = toHex(buffer_hex_ptr,data); *(buffer_hex_ptr++) = ' ';
            *(buffer_char_ptr++) = isalnum(data) ? data : '.';
        }
        *buffer_hex_ptr = '\0';
        *buffer_char_ptr = '\0';
        printf("%08X: %s - %s\r\n",addr,buffer_hex, buffer_char);
        addr+=0x10;
    }
}

DigitalOut led(LED1);

void test()
{
    OTP_Dump();
    while(1)
    {
        led = !led;
        ThisThread::sleep_for(1000);
    }
}