/****************************************************************************
*
*  flash.h
*
******************************************************************************/

#ifndef MSCFLASH_H_
#define MSCFLASH_H_

extern struct statedata progdata;

extern void flash_init(bool wipe);
void flash_write(void);

#define MSC_KEY 0xDEAD

#define MSC_INIT_DEFAULT        \
{                               \
   MSC_KEY,       /* KEY */     \
   {0},           /*serial num */ \
   {0},           /*Unique */   \
   0,             /*HW Major*/  \
   0,             /*HW Minor*/  \
   V3_FW_MAJOR,   /*FW Major*/  \
   V3_FW_MINOR,   /*FW Minor*/  \
   0,							\
   0,							\
   0,							\
   0							\
}

struct statedata  // data to save
{
    U16 key;
    U8 serial[6];
	U8 unique[8];
	U8 hw_major;
    U8 hw_minor;
    U8 fw_major;
    U8 fw_minor;
	U8 sleepmode;
	U8 pad0;
	U8 pad1;
	U8 pad2;
};

#define EM_MSC_RUN_FROM_FLASH

#endif /* MSCFLASH_H_ */
