/***************************************************************************
 *
 *            Copyright (c) 2012 by Artafelx INC.
 *
 * This software is copyrighted by and is the sole property of
 * Artaflex INC.  All rights, title, ownership, or other interests
 * in the software remain the property of Artaflex INC.  This
 * software may only be used in accordance with the corresponding
 * license agreement.  Any unauthorized use, duplication, transmission,
 * distribution, or disclosure of this software is expressly forbidden.
 *
 * This Copyright notice may not be removed or modified without prior
 * written consent of Artaflex INC.
 *
 * Artaflex INC reserves the right to modify this software without notice.
 *
 * Artaflex INC.
 * 96 Steelcase Road West,
 * Markham, ON L3R 3T9
 * Canada
 *
 * Tel:   (905) 470-0109
 * http://www.artaflex.com

  
 *
 ***************************************************************************/
#include "stdint.h"
#include "stdbool.h"
#include "global.h"
#include "I2C.h"
#include "MAX14676E.h"


U8 max14676_GetVer(void);


// return true if exists
bool max14676_init(void)
{
   U8 var[1];
 

   if(!max14676_GetVer()) return(false);

   var[0]=0x40;
   I2C_WRITE(MAX14676E_ADR50,MAX_LDOCFG,var,1); // Enable LDO

   var[0] = 0;
   I2C_WRITE(MAX14676E_ADR50,MAX_LED_CFG,var,1);  //0.6mA per LED step
  
   var[0] = 0x80;
   I2C_WRITE(MAX14676E_ADR50, MAX_PWR_CFG_ADDR, var,1); 
   
   var[0] =0x17;
   I2C_WRITE(MAX14676E_ADR50,MAX_ILIMCNTL_ADDR, var,1);
   
   var[0] = 0x14;
   I2C_WRITE(MAX14676E_ADR50,MAX_CHGCNTLA, var, 1);

   var[0] =  0x6E;
   I2C_WRITE(MAX14676E_ADR50,MAX_PCHGCNTL,var,1); 
   
   var[0] =  MAX_CDET_INIT; //Charger Auto restart
   I2C_WRITE(MAX14676E_ADR50,MAX_CDETCNTLB,var,1); 

   var[0] =  MAX_VSET_INIT; //Voltage settings
   I2C_WRITE(MAX14676E_ADR50,MAX_CHGVSET,var,1); 

   var[0] = 0x90;  // Thermistor and charger enabled
   I2C_WRITE(MAX14676E_ADR50,MAX_CHGCNTLB,var,1);
   
//   var[0] =  MAX_BST_ON;
//   I2C_WRITE(MAX14676E_ADR50,MAX_BST_CFG,var,1);   

   return(true);
   
}

void max14676_LED(struct rgbcolor ledset )
{
U8 var[1];

   ledset.red &=0x1F;
   ledset.grn &=0x1F;
   ledset.blu &=0x1F;

   if(ledset.red)
	{
	   ledset.red--;
	   ledset.red |= MAX_LEDN_CFG;
	}
   if(ledset.grn)
   {
      ledset.grn--;
      ledset.grn |= MAX_LEDN_CFG;
   }
   if(ledset.blu) 
   {
      ledset.blu--;
      ledset.blu |= MAX_LEDN_CFG;
   }
   
   
// MAX14676E does not have Auto inc I2C register address mode, write each register individually
   var[0] = ledset.red;
   I2C_WRITE(MAX14676E_ADR50,MAX_LED0_RED,var,1);
   
   var[0] = ledset.grn;
   I2C_WRITE(MAX14676E_ADR50,MAX_LED1_GRN,var,1);
   
   var[0] = ledset.blu;
   I2C_WRITE(MAX14676E_ADR50,MAX_LED2_BLU,var,1);
}


//return the percentage 
U8 max14676_GetBatteryCharge(void) 
{
U8 var[2];
	
    I2C_READ(MAX14676E_ADR6C,MAX_SOC,var,2);
    return (var[0]) ;
 
}

U16 max14676_GetBatteryChargeVoltage(void) 
{
U16 data; 
U8 var[2];
   
   I2C_READ(MAX14676E_ADR6C,MAX_VCELL, var,2);
   
   data = ((U16)var[1]) + ((U16)var[0] <<8);
	
	return data;
}

U16 max14676_GetBatteryCRATE(void) 
{
U16 data; 
U8 var[2];
   
   I2C_READ(MAX14676E_ADR6C,MAX_CRATE, var,2);
   
   data = ((U16)var[1]) + ((U16)var[0] <<8);
	
	return data;
}

U8 max14676_GetStatusA(void)
{
U8 var[1];
	
    I2C_READ(MAX14676E_ADR50,MAX_STATUSA,var,1);
    return (var[0]) ;   
}

U8 max14676_GetStatusB(void)
{
U8 var[1];
	
    I2C_READ(MAX14676E_ADR50,MAX_STATUSB,var,1);
    return (var[0]) ;   
}

U8 max14676_GetVer(void) 
{
  U8 result;
  U8 var[1];
  
  result =  (U8)I2C_READ(MAX14676E_ADR50,MAX_CHIP_ID,var,1);                      

  
  if((result == 0 )&&(var[0] == 0x2E)) return (1);
  else return(0);
}



void max14676_ChargerOn(void)
{
U8 var[1];

   var[0] =0x17;
   I2C_WRITE(MAX14676E_ADR50,MAX_ILIMCNTL_ADDR, var,1);
   
   var[0] = 0x14;
   I2C_WRITE(MAX14676E_ADR50,MAX_CHGCNTLA, var, 1);

   var[0] =  0x6E;
   I2C_WRITE(MAX14676E_ADR50,MAX_PCHGCNTL,var,1);

   var[0] =  MAX_CDET_INIT; //Charger Auto restart
   I2C_WRITE(MAX14676E_ADR50,MAX_CDETCNTLB,var,1);  

   var[0] =  MAX_VSET_INIT; //Voltage settings
   I2C_WRITE(MAX14676E_ADR50,MAX_CHGVSET,var,1);    

   var[0] = 0x90;  // Thermistor and charger enabled
   I2C_WRITE(MAX14676E_ADR50,MAX_CHGCNTLB,var,1);
}


void max14676_Boost(U8 state)
{
U8 var[1];

   if (state) var[0] =  MAX_BST_ON; 
      else var[0] =  MAX_BST_OFF;
   I2C_WRITE(MAX14676E_ADR50,MAX_BST_CFG,var,1);  
}



#if 0
void max14676_process(void)
{
   U8 data;
	U8 status;
	U8 myCharge;
		
      	//DisableInterrupts; 
      	max14676Voltage = max14676_GetBatteryChargeVoltage(); 
    	myCharge = max14676_GetBatteryCharge();
		
		status =  max14676_ReadReg(&data,0x2);
    	//EnableInterrupts; 
		
		max14676Charge += myCharge;
		max14676Charge = max14676Charge/2;
		
		
		if(max14676Charge >100)
			max14676Charge =100;
		
		max14676ChargeStat   = (/*data == 0x5 || */ data == 0x6) ? 1 : 0;   //  //1 charged; 0 uncharged
		if(max14676ChargeStat) 
			max14676Charge =100;
      	if(max14676Voltage >= 9600)    //3V              
      	  max14676Voltage_old = max14676Voltage;    
      	else
          max14676Voltage = max14676Voltage_old;      
      	max14676Voltage = max14676Voltage/625 * 78;
	   
      
	
	
}

#endif
void max14676_poweroff(void)
{
U8 var[1];   
   
   
   var[0] = 0x05;
	
	for(;;)
	{
      I2C_WRITE(MAX14676E_ADR50,MAX_PWR_CFG_ADDR,var,1);
	}

}

