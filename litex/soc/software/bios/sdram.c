#include <generated/csr.h>
#ifdef CSR_SDRAM_BASE

#include <stdio.h>
#include <stdlib.h>

#include <generated/sdram_phy.h>
#include <generated/mem.h>
#include <hw/flags.h>
#include <system.h>

#include "sdram.h"

static void cdelay(int i)
{
	while(i > 0) {
#if defined (__lm32__)
		__asm__ volatile("nop");
#elif defined (__or1k__)
		__asm__ volatile("l.nop");
#elif defined (__picorv32__)
		__asm__ volatile("nop");
#elif defined (__vexriscv__)
		__asm__ volatile("nop");
#elif defined (__minerva__)
		__asm__ volatile("nop");
#else
#error Unsupported architecture
#endif
		i--;
	}
}

void sdrsw(void)
{
	sdram_dfii_control_write(DFII_CONTROL_CKE|DFII_CONTROL_ODT|DFII_CONTROL_RESET_N);
	printf("SDRAM now under software control\n");
}

void sdrhw(void)
{
	sdram_dfii_control_write(DFII_CONTROL_SEL);
	printf("SDRAM now under hardware control\n");
}

void sdrrow(char *_row)
{
	char *c;
	unsigned int row;

	if(*_row == 0) {
		sdram_dfii_pi0_address_write(0x0000);
		sdram_dfii_pi0_baddress_write(0);
		command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
		cdelay(15);
		printf("Precharged\n");
	} else {
		row = strtoul(_row, &c, 0);
		if(*c != 0) {
			printf("incorrect row\n");
			return;
		}
		sdram_dfii_pi0_address_write(row);
		sdram_dfii_pi0_baddress_write(0);
		command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CS);
		cdelay(15);
		printf("Activated row %d\n", row);
	}
}

void sdrrdbuf(int dq)
{
	int i, p;
	int first_byte, step;

	if(dq < 0) {
		first_byte = 0;
		step = 1;
	} else {
		first_byte = DFII_PIX_DATA_SIZE/2 - 1 - dq;
		step = DFII_PIX_DATA_SIZE/2;
	}

	for(p=0;p<DFII_NPHASES;p++)
		for(i=first_byte;i<DFII_PIX_DATA_SIZE;i+=step)
			printf("%02x", MMPTR(sdram_dfii_pix_rddata_addr[p]+4*i));
	printf("\n");
}

void sdrrd(char *startaddr, char *dq)
{
	char *c;
	unsigned int addr;
	int _dq;

	if(*startaddr == 0) {
		printf("sdrrd <address>\n");
		return;
	}
	addr = strtoul(startaddr, &c, 0);
	if(*c != 0) {
		printf("incorrect address\n");
		return;
	}
	if(*dq == 0)
		_dq = -1;
	else {
		_dq = strtoul(dq, &c, 0);
		if(*c != 0) {
			printf("incorrect DQ\n");
			return;
		}
	}

	sdram_dfii_pird_address_write(addr);
	sdram_dfii_pird_baddress_write(0);
	command_prd(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA);
	cdelay(15);
	sdrrdbuf(_dq);
}

void sdrrderr(char *count)
{
	int addr;
	char *c;
	int _count;
	int i, j, p;
	unsigned char prev_data[DFII_NPHASES*DFII_PIX_DATA_SIZE];
	unsigned char errs[DFII_NPHASES*DFII_PIX_DATA_SIZE];

	if(*count == 0) {
		printf("sdrrderr <count>\n");
		return;
	}
	_count = strtoul(count, &c, 0);
	if(*c != 0) {
		printf("incorrect count\n");
		return;
	}

	for(i=0;i<DFII_NPHASES*DFII_PIX_DATA_SIZE;i++)
			errs[i] = 0;
	for(addr=0;addr<16;addr++) {
		sdram_dfii_pird_address_write(addr*8);
		sdram_dfii_pird_baddress_write(0);
		command_prd(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA);
		cdelay(15);
		for(p=0;p<DFII_NPHASES;p++)
			for(i=0;i<DFII_PIX_DATA_SIZE;i++)
				prev_data[p*DFII_PIX_DATA_SIZE+i] = MMPTR(sdram_dfii_pix_rddata_addr[p]+4*i);

		for(j=0;j<_count;j++) {
			command_prd(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA);
			cdelay(15);
			for(p=0;p<DFII_NPHASES;p++)
				for(i=0;i<DFII_PIX_DATA_SIZE;i++) {
					unsigned char new_data;

					new_data = MMPTR(sdram_dfii_pix_rddata_addr[p]+4*i);
					errs[p*DFII_PIX_DATA_SIZE+i] |= prev_data[p*DFII_PIX_DATA_SIZE+i] ^ new_data;
					prev_data[p*DFII_PIX_DATA_SIZE+i] = new_data;
				}
		}
	}

	for(i=0;i<DFII_NPHASES*DFII_PIX_DATA_SIZE;i++)
		printf("%02x", errs[i]);
	printf("\n");
	for(p=0;p<DFII_NPHASES;p++)
		for(i=0;i<DFII_PIX_DATA_SIZE;i++)
			printf("%2x", DFII_PIX_DATA_SIZE/2 - 1 - (i % (DFII_PIX_DATA_SIZE/2)));
	printf("\n");
}

void sdrwr(char *startaddr)
{
	char *c;
	unsigned int addr;
	int i;
	int p;

	if(*startaddr == 0) {
		printf("sdrrd <address>\n");
		return;
	}
	addr = strtoul(startaddr, &c, 0);
	if(*c != 0) {
		printf("incorrect address\n");
		return;
	}

	for(p=0;p<DFII_NPHASES;p++)
		for(i=0;i<DFII_PIX_DATA_SIZE;i++)
			MMPTR(sdram_dfii_pix_wrdata_addr[p]+4*i) = 0x10*p + i;

	sdram_dfii_piwr_address_write(addr);
	sdram_dfii_piwr_baddress_write(0);
	command_pwr(DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS|DFII_COMMAND_WRDATA);
}

#ifdef CSR_DDRPHY_BASE

#ifdef KUSDDRPHY
#define ERR_DDRPHY_DELAY 512
#else
#define ERR_DDRPHY_DELAY 32
#endif
#define ERR_DDRPHY_BITSLIP 8

#define NBMODULES DFII_PIX_DATA_SIZE/2

#ifdef CSR_DDRPHY_WLEVEL_EN_ADDR

void sdrwlon(void)
{
	sdram_dfii_pi0_address_write(DDRX_MR1 | (1 << 7));
	sdram_dfii_pi0_baddress_write(1);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
	ddrphy_wlevel_en_write(1);
}

void sdrwloff(void)
{
	sdram_dfii_pi0_address_write(DDRX_MR1);
	sdram_dfii_pi0_baddress_write(1);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
	ddrphy_wlevel_en_write(0);
}

int write_level(void)
{
	int i, j, k;

	int dq_address;
	unsigned char dq;

	int err_ddrphy_wdly;

	unsigned char taps_scan[ERR_DDRPHY_DELAY];

	int one_window_active;
	int one_window_start;

	int delays[NBMODULES];

    int ok;

#ifdef KUSDDRPHY
	err_ddrphy_wdly = ERR_DDRPHY_DELAY; /* FIXME */
#else
	err_ddrphy_wdly = ERR_DDRPHY_DELAY - ddrphy_half_sys8x_taps_read() - 1;
#endif

	printf("Write leveling:\n");

	sdrwlon();
	cdelay(100);
	for(i=0;i<NBMODULES;i++) {
		printf("m%d: |", i);
	    dq_address = sdram_dfii_pix_rddata_addr[0]+4*(NBMODULES-1-i);

	    /* reset delay */
		ddrphy_dly_sel_write(1 << i);
		ddrphy_wdly_dq_rst_write(1);
		ddrphy_wdly_dqs_rst_write(1);
#ifdef KUSDDRPHY /* need to init manually on Ultrascale */
		for(j=0; j<ddrphy_wdly_dqs_taps_read(); j++)
			ddrphy_wdly_dqs_inc_write(1);
#endif
		/* scan taps */
		for(j=0;j<err_ddrphy_wdly;j++) {
			int zero_count = 0;
			int one_count = 0;
			for (k=0; k<128; k++) {
				ddrphy_wlevel_strobe_write(1);
				cdelay(10);
				dq = MMPTR(dq_address);
				if (dq != 0)
					one_count++;
				else
					zero_count++;
			}
			if (one_count > zero_count)
				taps_scan[j] = 1;
			else
				taps_scan[j] = 0;
			printf("%d", taps_scan[j]);
			ddrphy_wdly_dq_inc_write(1);
			ddrphy_wdly_dqs_inc_write(1);
			cdelay(10);
		}
		printf("|");

		/* select last 0/1 transition */
		one_window_active = 0;
		one_window_start = 0;
		delays[i] = -1;
		for(j=0;j<err_ddrphy_wdly;j++) {
			if (one_window_active) {
				if (taps_scan[j] == 0)
					one_window_active = 0;
			} else {
				if (taps_scan[j]) {
					one_window_active = 1;
					one_window_start = j;
				}
			}
		}
		delays[i] = one_window_start;

		/* configure delays */
		ddrphy_wdly_dq_rst_write(1);
		ddrphy_wdly_dqs_rst_write(1);
#ifdef KUSDDRPHY /* need to init manually on Ultrascale */
		for(j=0; j<ddrphy_wdly_dqs_taps_read(); j++)
			ddrphy_wdly_dqs_inc_write(1);
#endif
		for(j=0; j<delays[i]; j++) {
			ddrphy_wdly_dq_inc_write(1);
			ddrphy_wdly_dqs_inc_write(1);
		}

		printf(" delay: %02d\n", delays[i]);
	}

	sdrwloff();

	ok = 1;
	for(i=NBMODULES-1;i>=0;i--) {
		if(delays[i] < 0)
			ok = 0;
	}

	return ok;
}

#endif /* CSR_DDRPHY_WLEVEL_EN_ADDR */

static void read_bitslip_inc(char m)
{
		ddrphy_dly_sel_write(1 << m);
		ddrphy_rdly_dq_bitslip_write(1);
}

static int read_level_scan(int module, int bitslip)
{
	unsigned int prv;
	unsigned char prs[DFII_NPHASES*DFII_PIX_DATA_SIZE];
	int p, i, j;
	int score;

	/* Generate pseudo-random sequence */
	prv = 42;
	for(i=0;i<DFII_NPHASES*DFII_PIX_DATA_SIZE;i++) {
		prv = 1664525*prv + 1013904223;
		prs[i] = prv;
	}

	/* Activate */
	sdram_dfii_pi0_address_write(0);
	sdram_dfii_pi0_baddress_write(0);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CS);
	cdelay(15);

	/* Write test pattern */
	for(p=0;p<DFII_NPHASES;p++)
		for(i=0;i<DFII_PIX_DATA_SIZE;i++)
			MMPTR(sdram_dfii_pix_wrdata_addr[p]+4*i) = prs[DFII_PIX_DATA_SIZE*p+i];
	sdram_dfii_piwr_address_write(0);
	sdram_dfii_piwr_baddress_write(0);
	command_pwr(DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS|DFII_COMMAND_WRDATA);

	/* Calibrate each DQ in turn */
	sdram_dfii_pird_address_write(0);
	sdram_dfii_pird_baddress_write(0);
	score = 0;

	printf("m%d, b%d: |", module, bitslip);
	ddrphy_dly_sel_write(1 << module);
	ddrphy_rdly_dq_rst_write(1);
	for(j=0; j<ERR_DDRPHY_DELAY;j++) {
		int working;
		command_prd(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA);
		cdelay(15);
		working = 1;
		for(p=0;p<DFII_NPHASES;p++) {
			if(MMPTR(sdram_dfii_pix_rddata_addr[p]+4*(NBMODULES-module-1)) != prs[DFII_PIX_DATA_SIZE*p+(NBMODULES-module-1)])
				working = 0;
			if(MMPTR(sdram_dfii_pix_rddata_addr[p]+4*(2*NBMODULES-module-1)) != prs[DFII_PIX_DATA_SIZE*p+2*NBMODULES-module-1])
				working = 0;
		}
		printf("%d", working);
		score += working;
		ddrphy_rdly_dq_inc_write(1);
	}
	printf("| ");

	/* Precharge */
	sdram_dfii_pi0_address_write(0);
	sdram_dfii_pi0_baddress_write(0);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
	cdelay(15);

	return score;
}

static void read_level(int module)
{
	unsigned int prv;
	unsigned char prs[DFII_NPHASES*DFII_PIX_DATA_SIZE];
	int p, i, j;
	int working;
	int delay, delay_min, delay_max;

	printf("delays: ");

	/* Generate pseudo-random sequence */
	prv = 42;
	for(i=0;i<DFII_NPHASES*DFII_PIX_DATA_SIZE;i++) {
		prv = 1664525*prv + 1013904223;
		prs[i] = prv;
	}

	/* Activate */
	sdram_dfii_pi0_address_write(0);
	sdram_dfii_pi0_baddress_write(0);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CS);
	cdelay(15);

	/* Write test pattern */
	for(p=0;p<DFII_NPHASES;p++)
		for(i=0;i<DFII_PIX_DATA_SIZE;i++)
			MMPTR(sdram_dfii_pix_wrdata_addr[p]+4*i) = prs[DFII_PIX_DATA_SIZE*p+i];
	sdram_dfii_piwr_address_write(0);
	sdram_dfii_piwr_baddress_write(0);
	command_pwr(DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS|DFII_COMMAND_WRDATA);

	/* Calibrate each DQ in turn */
	sdram_dfii_pird_address_write(0);
	sdram_dfii_pird_baddress_write(0);

	ddrphy_dly_sel_write(1 << module);
	delay = 0;

	/* Find smallest working delay */
	ddrphy_rdly_dq_rst_write(1);
	while(1) {
		command_prd(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA);
		cdelay(15);
		working = 1;
		for(p=0;p<DFII_NPHASES;p++) {
			if(MMPTR(sdram_dfii_pix_rddata_addr[p]+4*(NBMODULES-module-1)) != prs[DFII_PIX_DATA_SIZE*p+(NBMODULES-module-1)])
				working = 0;
			if(MMPTR(sdram_dfii_pix_rddata_addr[p]+4*(2*NBMODULES-module-1)) != prs[DFII_PIX_DATA_SIZE*p+2*NBMODULES-module-1])
				working = 0;
		}
		if(working)
			break;
		delay++;
		if(delay >= ERR_DDRPHY_DELAY)
			break;
		ddrphy_rdly_dq_inc_write(1);
	}
	delay_min = delay;

	/* Get a bit further into the working zone */
#ifdef KUSDDRPHY
	for(j=0;j<16;j++) {
		delay += 1;
		ddrphy_rdly_dq_inc_write(1);
	}
#else
	delay++;
	ddrphy_rdly_dq_inc_write(1);
#endif

	/* Find largest working delay */
	while(1) {
		command_prd(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA);
		cdelay(15);
		working = 1;
		for(p=0;p<DFII_NPHASES;p++) {
			if(MMPTR(sdram_dfii_pix_rddata_addr[p]+4*(NBMODULES-module-1)) != prs[DFII_PIX_DATA_SIZE*p+(NBMODULES-module-1)])
				working = 0;
			if(MMPTR(sdram_dfii_pix_rddata_addr[p]+4*(2*NBMODULES-module-1)) != prs[DFII_PIX_DATA_SIZE*p+2*NBMODULES-module-1])
				working = 0;
		}
		if(!working)
			break;
		delay++;
		if(delay >= ERR_DDRPHY_DELAY)
			break;
		ddrphy_rdly_dq_inc_write(1);
	}
	delay_max = delay;

	printf("%02d+-%02d", (delay_min+delay_max)/2, (delay_max-delay_min)/2);

	/* Set delay to the middle */
	ddrphy_rdly_dq_rst_write(1);
	for(j=0;j<(delay_min+delay_max)/2;j++)
		ddrphy_rdly_dq_inc_write(1);

	/* Precharge */
	sdram_dfii_pi0_address_write(0);
	sdram_dfii_pi0_baddress_write(0);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
	cdelay(15);
}
#endif /* CSR_DDRPHY_BASE */

static unsigned int seed_to_data_32(unsigned int seed, int random)
{
	if (random)
		return 1664525*seed + 1013904223;
	else
		return seed + 1;
}

static unsigned short seed_to_data_16(unsigned short seed, int random)
{
	if (random)
		return 25173*seed + 13849;
	else
		return seed + 1;
}

#define ONEZERO 0xAAAAAAAA
#define ZEROONE 0x55555555

#ifndef MEMTEST_BUS_SIZE
#define MEMTEST_BUS_SIZE (512)
#endif

//#define MEMTEST_BUS_DEBUG

static int memtest_bus(void)
{
	volatile unsigned int *array = (unsigned int *)MAIN_RAM_BASE;
	int i, errors;
	unsigned int rdata;

	errors = 0;

	for(i=0;i<MEMTEST_BUS_SIZE/4;i++) {
		array[i] = ONEZERO;
	}
	flush_cpu_dcache();
	flush_l2_cache();
	for(i=0;i<MEMTEST_BUS_SIZE/4;i++) {
		rdata = array[i];
		if(rdata != ONEZERO) {
			errors++;
#ifdef MEMTEST_BUS_DEBUG
			printf("[bus: 0x%0x]: 0x%08x vs 0x%08x\n", i, rdata, ONEZERO);
#endif
		}
	}

	for(i=0;i<MEMTEST_BUS_SIZE/4;i++) {
		array[i] = ZEROONE;
	}
	flush_cpu_dcache();
	flush_l2_cache();
	for(i=0;i<MEMTEST_BUS_SIZE/4;i++) {
		rdata = array[i];
		if(rdata != ZEROONE) {
			errors++;
#ifdef MEMTEST_BUS_DEBUG
			printf("[bus 0x%0x]: 0x%08x vs 0x%08x\n", i, rdata, ZEROONE);
#endif
		}
	}

	return errors;
}

#ifndef MEMTEST_DATA_SIZE
#define MEMTEST_DATA_SIZE (2*1024*1024)
#endif
#define MEMTEST_DATA_RANDOM 1

//#define MEMTEST_DATA_DEBUG

static int memtest_data(void)
{
	volatile unsigned int *array = (unsigned int *)MAIN_RAM_BASE;
	int i, errors;
	unsigned int seed_32;
	unsigned int rdata;

	errors = 0;
	seed_32 = 0;

	for(i=0;i<MEMTEST_DATA_SIZE/4;i++) {
		seed_32 = seed_to_data_32(seed_32, MEMTEST_DATA_RANDOM);
		array[i] = seed_32;
	}

	seed_32 = 0;
	flush_cpu_dcache();
	flush_l2_cache();
	for(i=0;i<MEMTEST_DATA_SIZE/4;i++) {
		seed_32 = seed_to_data_32(seed_32, MEMTEST_DATA_RANDOM);
		rdata = array[i];
		if(rdata != seed_32) {
			errors++;
#ifdef MEMTEST_DATA_DEBUG
			printf("[data 0x%0x]: 0x%08x vs 0x%08x\n", i, rdata, seed_32);
#endif
		}
	}

	return errors;
}
#ifndef MEMTEST_ADDR_SIZE
#define MEMTEST_ADDR_SIZE (32*1024)
#endif
#define MEMTEST_ADDR_RANDOM 0

//#define MEMTEST_ADDR_DEBUG

static int memtest_addr(void)
{
	volatile unsigned int *array = (unsigned int *)MAIN_RAM_BASE;
	int i, errors;
	unsigned short seed_16;
	unsigned short rdata;

	errors = 0;
	seed_16 = 0;

	for(i=0;i<MEMTEST_ADDR_SIZE/4;i++) {
		seed_16 = seed_to_data_16(seed_16, MEMTEST_ADDR_RANDOM);
		array[(unsigned int) seed_16] = i;
	}

	seed_16 = 0;
	flush_cpu_dcache();
	flush_l2_cache();
	for(i=0;i<MEMTEST_ADDR_SIZE/4;i++) {
		seed_16 = seed_to_data_16(seed_16, MEMTEST_ADDR_RANDOM);
		rdata = array[(unsigned int) seed_16];
		if(rdata != i) {
			errors++;
#ifdef MEMTEST_ADDR_DEBUG
			printf("[addr 0x%0x]: 0x%08x vs 0x%08x\n", i, rdata, i);
#endif
		}
	}

	return errors;
}

#ifdef BOOT_MEMTEST
int verify_memtest(void) {
	volatile unsigned int *array = (unsigned int *)MAIN_RAM_BASE;
	int i, errors;
	unsigned int seed_32;
	unsigned int rdata;

	errors = 0;
	seed_32 = 0;
	flush_cpu_dcache();
	flush_l2_cache();
	for(i=0;i<MEMTEST_DATA_SIZE/4;i++) {
		seed_32 = seed_to_data_32(seed_32, MEMTEST_DATA_RANDOM);
		rdata = array[i];
		if(rdata != seed_32) {
			errors++;
			if( errors < 40 ) {
			  printf("[data 0x%0x]: (got)0x%08x vs (want)0x%08x\n", i, rdata, seed_32);
			}
		}
	}

	return errors;
}
#endif

int memtest(void)
{
	int bus_errors, data_errors, addr_errors;

	bus_errors = memtest_bus();
	if(bus_errors != 0)
		printf("Memtest bus failed: %d/%d errors\n", bus_errors, 2*128);

	addr_errors = memtest_addr();
	if(addr_errors != 0)
		printf("Memtest addr failed: %d/%d errors\n", addr_errors, MEMTEST_ADDR_SIZE/4);

	data_errors = memtest_data();
	if(data_errors != 0)
		printf("Memtest data failed: %d/%d errors\n", data_errors, MEMTEST_DATA_SIZE/4);

	if(bus_errors + data_errors + addr_errors != 0)
		return 0;

	else {
		printf("Memtest OK\n");
		return 1;
	}
}

#ifdef CSR_DDRPHY_BASE
int sdrlevel(void)
{
	int i, j;
	int bitslip;
	int score;
	int best_score;
	int best_bitslip;

	sdrsw();

	for(i=0; i<NBMODULES; i++) {
		ddrphy_dly_sel_write(1<<i);
		ddrphy_rdly_dq_rst_write(1);
		ddrphy_rdly_dq_bitslip_rst_write(1);
	}

#ifdef CSR_DDRPHY_WLEVEL_EN_ADDR
	if(!write_level())
		return 0;
#endif

	printf("Read leveling:\n");
	for(i=0; i<NBMODULES; i++) {
		/* scan possible read windows */
		best_score = 0;
		best_bitslip = 0;
		for(bitslip=0; bitslip<ERR_DDRPHY_BITSLIP; bitslip++) {
			/* compute score */
			score = read_level_scan(i, bitslip);
			read_level(i);
			printf("\n");
			if (score > best_score) {
				best_bitslip = bitslip;
				best_score = score;
			}
			/* exit */
			if (bitslip == ERR_DDRPHY_BITSLIP-1)
				break;
			/* increment bitslip */
			read_bitslip_inc(i);
		}

		/* select best read window */
		printf("best: m%d, b%d ", i, best_bitslip);
		ddrphy_rdly_dq_bitslip_rst_write(1);
		for (j=0; j<best_bitslip; j++)
			read_bitslip_inc(i);

		/* re-do leveling on best read window*/
		read_level(i);
		printf("\n");
	}

	return 1;
}
#endif

int sdrinit(void)
{
	printf("Initializing SDRAM...\n");

	init_sequence();
#ifdef CSR_DDRPHY_BASE
#if CSR_DDRPHY_EN_VTC_ADDR
	ddrphy_en_vtc_write(0);
#endif
	sdrlevel();
#if CSR_DDRPHY_EN_VTC_ADDR
	ddrphy_en_vtc_write(1);
#endif
#endif
	sdrhw();
	if(!memtest()) {
		return 0;
	}

	return 1;
}

#endif

// 0x9041 1001 0000 0100 0001
// CRG MMCM
#define MTE 46  // MCM table entries
static int crg_mmcm[46] = {0x28, 0xffff, 0x9, 0x0, 0x8, 0x1041, 0xa, 0x82, 0xb, 0x0, 0xc, 0x9041, 0xd, 0x0, 0xe, 0x41, 0xf, 0x40, 0x10, 0x1208, 0x11, 0x0, 0x6, 0x1104, 0x7, 0x0, 0x12, 0x41, 0x13, 0x40, 0x16, 0x1041, 0x14, 0x1208, 0x15, 0x0, 0x18, 0x271, 0x19, 0x7c01, 0x1a, 0xffe9, 0x4e, 0x9908, 0x4f, 0x8100};

#define MMCM_TIMEOUT 1000000

#ifdef CSR_CRG_BASE
static void crg_mmcm_write(int adr, int data) {
  int timeout = 0;
	crg_mmcm_adr_write(adr);
	crg_mmcm_dat_w_write(data);
	crg_mmcm_write_write(1);
	while(!crg_mmcm_drdy_read() && timeout < MMCM_TIMEOUT)
	  timeout++;
	if( timeout >= MMCM_TIMEOUT ) {
	  printf("crg_mmcm_write failed with adr %x, data %x\n", adr, data);
	}
}

static int crg_mmcm_read(int adr) {
  int timeout = 0;
	crg_mmcm_adr_write(adr);
	crg_mmcm_read_write(1);
	while(!crg_mmcm_drdy_read() && timeout < MMCM_TIMEOUT)
	  timeout++;
	if( timeout >= MMCM_TIMEOUT ) {
	  printf("crg_mmcm_read failed with adr %x\n", adr);
	}

	return crg_mmcm_dat_r_read();
}
#endif

void config_crg(int phase, int style) {
  int i;
  int val;
  if( style == 0 ) {
    for( i = 0; i < MTE; i += 2 ) {
      if( crg_mmcm[i] == 0xc ) {
	val = (crg_mmcm[i+1] & 0x1FFF) | ((phase & 0x7) << 13);
	printf( " ===========> 0xc entry: %04x\n", val );
	crg_mmcm_write(crg_mmcm[i], val );
      } else {
	crg_mmcm_write(crg_mmcm[i], crg_mmcm[i+1]);
      }
    }
  } else {
    val = 0x1041 | ((phase & 0x7) << 13);
    printf( " ===========> 0xc entry: %04x\n", val );
    crg_mmcm_write(0xc, val );
  }
}

#define S7_MMCM_MAP_LEN  23
// map order comes from xapp888 -- no explanation in docs for why the order is necessary, but it seems important
static int addr_map[S7_MMCM_MAP_LEN] = {0x28, 0x9, 0x8, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0x10, 0x11, 0x6, 0x7,
					0x12, 0x13, 0x16, 0x14, 0x15, 0x18, 0x19, 0x1a, 0x4e, 0x4f};

void mmcm_dump_code(void) {
  int i;

#ifdef CSR_CRG_BASE
  printf( "// CRG MMCM\n" );
  printf( "int crg_mmcm[%d] = {", S7_MMCM_MAP_LEN * 2 );
  for( i = 0; i < S7_MMCM_MAP_LEN; i++ ) {
    if( addr_map[i] == 0x28 )  // this state substitutes to all 1's for DRP to work
      printf( "0x28, 0xffff" );
    else
      printf( "0x%x, 0x%x", addr_map[i], crg_mmcm_read(addr_map[i]) );
    if( i < S7_MMCM_MAP_LEN - 1 )
      printf( ", " );
  }
  printf( "};\n" );
#endif
}

void sdr_scanphase(void) {
  int i;
  
  for( i = 0; i < 8; i++ ) {
    config_crg(i, 0);
    sdrsw();
    sdrinit();
  }

  config_crg(4, 0); // restore default setting
}

#ifdef BOOT_MEMTEST	
static void alt_init_sequence(int mr1, int mr2) {
	printf( "setting mr1: %x, mr2 %x\n", mr1, mr2 );
	
	/* Force reset */
	sdram_dfii_pi0_address_write(0x0);
	sdram_dfii_pi0_baddress_write(0);
	sdram_dfii_control_write(DFII_CONTROL_ODT);
	cdelay(50000);

	/* Release reset */
	sdram_dfii_pi0_address_write(0x0);
	sdram_dfii_pi0_baddress_write(0);
	sdram_dfii_control_write(DFII_CONTROL_ODT|DFII_CONTROL_RESET_N);
	cdelay(50000);

	/* Bring CKE high */
	sdram_dfii_pi0_address_write(0x0);
	sdram_dfii_pi0_baddress_write(0);
	sdram_dfii_control_write(DFII_CONTROL_CKE|DFII_CONTROL_ODT|DFII_CONTROL_RESET_N);
	cdelay(10000);

	/* Load Mode Register 2, CWL=5 */
	sdram_dfii_pi0_address_write(mr2);
	sdram_dfii_pi0_baddress_write(2);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS);

	/* Load Mode Register 3 */
	sdram_dfii_pi0_address_write(0x0);
	sdram_dfii_pi0_baddress_write(3);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS);

	/* Load Mode Register 1 */
	sdram_dfii_pi0_address_write(mr1);
	sdram_dfii_pi0_baddress_write(1);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS);

	/* Load Mode Register 0, CL=6, BL=8 */
	sdram_dfii_pi0_address_write(0xd20);
	sdram_dfii_pi0_baddress_write(0);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
	cdelay(200);

	/* ZQ Calibration */
	sdram_dfii_pi0_address_write(0x400);
	sdram_dfii_pi0_baddress_write(0);
	command_p0(DFII_COMMAND_WE|DFII_COMMAND_CS);
	cdelay(200);
}

static int iter_sdrinit(int rtt_nom, int rtt_wr, int ron) {
	sdrsw();
	
	printf( "\n\n###################################\n  trying rtt_nom %d rtt_wr %d, ron %d\n", rtt_nom, rtt_wr, ron );
	// 1111 1101 1001 1001 = 0xFD99
	int mr1 = 0x44 & 0xFD99;
	// 1111 1001 1111 1111 = 0xF9FF
	int mr2 = 0x0 & 0xF9FF;

	if( ron == 34 ) {
	  mr1 |= (1 << 1);
	}
	// 40 ohm is 00, and others are reserved
	
	if( rtt_nom == 60 ) {
	  mr1 |= (1 << 2);
	} else if( rtt_nom == 120 ) {
	  mr1 |= (1 << 6);
	} else if( rtt_nom == 40 ) {
	  mr1 |= (1 << 2);
	  mr1 |= (1 << 6);
	} else if( rtt_nom == 20 ) {
	  mr1 |= (1 << 9);
	} else if( rtt_nom == 30 ) {
	  mr1 |= (1 << 9);
	  mr1 |= (1 << 2);
	}
	// else disabled

	if( rtt_wr == 60 ) {
	  mr2 |= (1 << 9);
	} else if( rtt_wr == 120 ) {
	  mr2 |= (1 << 10);
	}
	// else off

	alt_init_sequence(mr1, mr2);

#ifdef CSR_DDRPHY_BASE
#if CSR_DDRPHY_EN_VTC_ADDR
	ddrphy_en_vtc_write(0);
#endif
	sdrlevel();
#if CSR_DDRPHY_EN_VTC_ADDR
	ddrphy_en_vtc_write(1);
#endif
#endif
	sdrhw();
	if(!memtest()) {
		return 0;
	}

	return 1;
}

void try_combos(void) {
  int rtt_nom_set[6] = {0, 120, 20, 40, 60, 30};
  int ron_set[2] = {34, 40};
  int rtt_wr_set[3] = {120, 60, 0};

  int i, j, k;
  unsigned long temp;

  while(1) {
    for( k = 0; k < 6; k++ ) {
      //      for( i = 0; i < 2; i++ ) {
	temp = (xadc_temperature_read()) * 50398 / 4096 - 27315;
	printf( "Die temp: %d.%02dC\n", temp / 100, temp - ((temp / 100) * 100));
	iter_sdrinit( rtt_nom_set[k], 0, ron_set[1] );
	//      }
    }
  }
}
#endif
