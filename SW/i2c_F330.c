/***************************************************************************
 
  I2C-DRIVER for F330
  ===================
 
 Keskeytyspohjainen I2C-driveri C8051F330-kontrollerille. Pohjana SiLabsin
 esimerkki F33x_SMBus_Master_Multibyte.c
 
 21.06.2010 - Ensimmäinen versio.

 22.06.2010 - Subaddr + i2c_set.

 24.06.2010 - i2c_set pois, write/read sub/-
 
  
***************************************************************************/

#include <datatypes.h>
#include <C8051F330.h>
#include "i2c_F330.h"

// Status vector - top 4 bits only
#define  SMB_MTSTA      0xE0           // (MT) start transmitted
#define  SMB_MTDB       0xC0           // (MT) data byte transmitted
#define  SMB_MRDB       0x80           // (MR) data byte received

#define  WRITE             0           // SMBus WRITE command
#define  READ              1           // SMBus READ command

BIT i2c_busy = 0;

BYTE slave_addr;                       // Target SMBus slave address
BYTE subaddr;                          // subaddress to be used
BIT i2c_rw;                            // Software flag to indicate the direction of the current transfer
BIT send_subaddr;                      // Are we using subaddr?
BIT randomread;                        // Are reading from subaddr?

BYTE byte_cnt;
IDATA BYTE *i2c_data_buf;

/*=====================================================================
 Writes cnt bytes of data to an I2C slave device.
---------------------------------------------------------------------*/
 
BIT i2c_write( BYTE slv_addr, IDATA BYTE *buf, BYTE cnt )
{
    if (i2c_busy)
        return FAIL;

    slave_addr = slv_addr;
    i2c_data_buf = buf;
    byte_cnt = cnt;

    send_subaddr = 0;
    randomread = 0;
    i2c_rw = WRITE;                     // Mark this transfer as a WRITE
    
    i2c_busy = 1;                       // Claim SMBus (set to busy)
    STA = 1;                            // Start transfer

    return OK;
}


/*=====================================================================
 Reads cnt bytes of data from an I2C slave device.
---------------------------------------------------------------------*/
 
BIT i2c_read( BYTE slv_addr, IDATA BYTE *buf, BYTE cnt )
{
    if (i2c_busy)
        return FAIL;

    slave_addr = slv_addr;
    i2c_data_buf = buf;
    byte_cnt = cnt - 1;

    send_subaddr = 0;
    randomread = 0;
    i2c_rw = READ;                      // Mark this transfer as a READ
    
    i2c_busy = 1;                       // Claim SMBus (set to busy)
    STA = 1;                            // Start transfer
  
    return OK;
}

/*=====================================================================
 Writes cnt bytes of data to an I2C slave device, subaddress sub
---------------------------------------------------------------------*/
 
BIT i2c_write_sub( BYTE slv_addr, BYTE sub, IDATA BYTE *buf, BYTE cnt )
{
    if (i2c_busy)
        return FAIL;

    slave_addr = slv_addr;
    subaddr = sub;
    i2c_data_buf = buf;
    byte_cnt = cnt;

    send_subaddr = 1;
    randomread = 0;
    i2c_rw = WRITE;                     // Mark this transfer as a WRITE
    
    i2c_busy = 1;                       // Claim SMBus (set to busy)
    STA = 1;                            // Start transfer

    return OK;
}


/*=====================================================================
 Reads cnt bytes of data from an I2C slave device, subaddress sub
---------------------------------------------------------------------*/
 
BIT i2c_read_sub( BYTE slv_addr, BYTE sub, IDATA BYTE *buf, BYTE cnt )
{
    if (i2c_busy)
        return FAIL;

    slave_addr = slv_addr;
    subaddr = sub;
    i2c_data_buf = buf;
    byte_cnt = cnt - 1;

    send_subaddr = 1;
    randomread = 1;
    i2c_rw = WRITE;                     // Mark this transfer as a WRITE because we first write subaddress
    
    i2c_busy = 1;                       // Claim SMBus (set to busy)
    STA = 1;                            // Start transfer
  
    return OK;
}

/* =======================================================================
I2C-keskeytys.

// SMBus ISR state machine
// - Master only implementation - no slave or arbitration states defined
// - All incoming data is written to global variable array i2c_data_buf
// - All outgoing data is read from global variable array i2c_data_buf
----------------------------------------------------------------------- */

void i2c_isr(void) __interrupt(SMB0_VECTOR) __using (1) 
{
   static BYTE ctr;
   static BIT send_start = 0;
   BIT fail = 0;                       // Used by the ISR to flag failed transfers

   if (ARBLOST == 0)                   // Check for errors
   {
      // Normal operation
      switch (SMB0CN & 0xF0)           // Status vector
      {
         // Master Transmitter/Receiver: START condition transmitted.
         case SMB_MTSTA:
            SMB0DAT = (slave_addr & 0xFE) | i2c_rw; // Load address of the target slave and R/W-bit
            STA = 0;                                // Manually clear START bit
            ctr = 0;                                // Reset the counter
            break;

         // Master Transmitter: Data byte transmitted
         case SMB_MTDB:
            if (ACK)                   // Slave ACK?
            {
               if (send_start)
               {
                   send_start = 0;
                   STA = 1;
                   break;
               }
               
               if (send_subaddr)
               {
                   send_subaddr = 0;
                   SMB0DAT = subaddr;
                   
                   if (randomread)
                   {
                       send_start = 1;
                       i2c_rw = READ;
                   }
                   break;
               }
               
               if (i2c_rw == WRITE)    // If this transfer is a WRITE,
               {
                  if (ctr < byte_cnt)
                  {
                     // send data byte
                     SMB0DAT = i2c_data_buf[ctr];
                     ctr++;
                  }
                  else
                  {
                     STO = 1;          // Set STO to terminate transfer
                     i2c_busy = 0;     // And free SMBus interface
                  }
               }
               else {}                 // If this transfer is a READ, proceed with transfer 
                                       // without writing to SMB0DAT (switch to receive mode)
            }
            else                       // If slave NACK,
            {
               fail = 1;
               //STO = 1;                // Send STOP condition, followed
               //STA = 1;                // By a START
            }
            break;

         // Master Receiver: byte received
         case SMB_MRDB:
            i2c_data_buf[ctr] = SMB0DAT;   // Store received byte

            if (ctr < byte_cnt)
            {
               ACK = 1;                        // Send ACK to indicate byte received
               ctr++;                        // Increment the byte counter
            }
            else
            {
               i2c_busy = 0;            // Free SMBus interface
               ACK = 0;                 // Send NACK to indicate last byte of this transfer
               STO = 1;                 // Send STOP to terminate transfer
            }
            break;

         default:
            fail = 1;                  // Indicate failed transfer and handle at end of ISR
            break;

      } // end switch
   }
   else // ARBLOST 
   {
      fail = 1;
   } 

   if (fail)                           // If the transfer failed,
   {
      SMB0CF &= ~0x80;                 // Reset communication
      SMB0CF |= 0x80;
      STA = 0;
      STO = 0;
      ACK = 0;

      i2c_busy = 0;                    // Free SMBus
   }

   SI = 0;                             // Clear interrupt flag
}

/* ============================ EOF ====================================== */
