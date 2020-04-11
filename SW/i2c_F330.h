/***************************************************************************
 
  I2C-DRIVER for F330
  ===================
 
 i2c_F330.h                       
 
 21.06.2010 - Ensimmäinen versio.
 
***************************************************************************/

#define OK    0
#define FAIL  1

extern BIT i2c_busy;

BIT i2c_write( BYTE slv_addr, IDATA BYTE *buf, BYTE cnt );
BIT i2c_read( BYTE slv_addr, IDATA BYTE *buf, BYTE cnt );
BIT i2c_write_sub( BYTE slv_addr, BYTE sub, IDATA BYTE *buf, BYTE cnt );
BIT i2c_read_sub( BYTE slv_addr, BYTE sub, IDATA BYTE *buf, BYTE cnt );

void i2c_isr(void) __interrupt(SMB0_VECTOR) __using (1);

