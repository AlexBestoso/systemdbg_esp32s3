#include <Arduino.h>
#include <cstdint>
#include "./systemReg.class.h"

ES3SystemRegister:: ES3SystemRegister(){

}
ES3SystemRegister::~ES3SystemRegister(){

}

void ES3SystemRegister::regWrite(uint32_t reg, uint32_t val){
        ES3_SYSREG_WRITE(reg, val);
}
uint32_t ES3SystemRegister::regRead(uint32_t reg){
        return ES3_SYSREG_READ(reg);
}

uint32_t ES3SystemRegister::regSet(uint32_t reg, bool val, int pos){
        int v = val ? 1 : 0;
        reg &= ~(1<<pos);
        reg += (v<<pos);
        return reg;
}

uint32_t ES3SystemRegister::regSet(uint32_t reg, int val, int x, int y){
        int s = y - x;
        int g = 0;
        for(int i=0; i<s; i++)
                g = (g << 1) + 1;
        val &= g;
        reg &= ~(1<<x);
        reg += (val << x);
        return reg;
}

uint32_t ES3SystemRegister::getSystem_core_1_control_0(void){
	uint16_t base = this->core_1_control_0.base;
	this->core_1_control_0.val = this->regRead(base);

	this->core_1_control_0.system_control_core_1_reseting = this->regGet(
		this->core_1_control_0.val,
		this->core_1_control_0.system_control_core_1_reseting_pos
	);
	this->core_1_control_0.system_control_core_1_clkgate_en = this->regGet(
		this->core_1_control_0.val,
		this->core_1_control_0.system_control_core_1_clkgate_en_pos
	);
	this->core_1_control_0.system_control_core_1_runstall = this->regGet(
		this->core_1_control_0.val,
		this->core_1_control_0.system_control_core_1_runstall_pos
	);

	return this->core_1_control_0.val;
}
uint32_t ES3SystemRegister::getSystem_core_1_control_1(void){
	uint16_t base = this->core_1_control_1.base;
	this->core_1_control_1.val = this->regRead(base);

	this->core_1_control_1.system_control_core_1_message = this->regGet(
		this->core_1_control_1.val,
		this->core_1_control_1.system_control_core_1_message_x,
		this->core_1_control_1.system_control_core_1_message_y
	);
	
	return this->core_1_control_1.val;
}
uint32_t ES3SystemRegister::getSystem_cpu_per_conf(void){
	uint16_t base = this->cpu_per_conf.base;
	this->cpu_per_conf.val = this->regRead(base);

	this->cpu_per_conf.system_cpu_waiti_delay_num = this->regGet(
		this->cpu_per_conf.val,
		this->cpu_per_conf.system_cpu_waiti_delay_num_x,
		this->cpu_per_conf.system_cpu_waiti_delay_num_y
	);

	this->cpu_per_conf.system_cpu_wait_mode_force_on = this->regGet(
		this->cpu_per_conf.val,
		this->cpu_per_conf.system_cpu_wait_mode_force_on_pos
	);

	this->cpu_per_conf.system_pll_freq_sel = this->regGet(
		this->cpu_per_conf.val,
		this->cpu_per_conf.system_pll_freq_sel_pos
	);

	this->cpu_per_conf.system_cpuperiod_sel = this->regGet(
		this->cpu_per_conf.val,
		this->cpu_per_conf.system_cpuperiod_sel_x,
		this->cpu_per_conf.system_cpuperiod_sel_y
	);

	return this->cpu_per_conf.val;
}
uint32_t ES3SystemRegister::getSystem_perip_clk_en0(void){
	uint16_t base = this->perip_clk_en0.base;
	this->perip_clk_en0.val = this->regRead(base);

	this->perip_clk_en0.system_adc2_arb_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_adc2_arb_clk_en_pos
	);

	this->perip_clk_en0.system_systimer_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_systimer_clk_en_pos
	);

	this->perip_clk_en0.system_apb_saradc_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_apb_saradc_clk_en_pos
	);

	this->perip_clk_en0.system_uart_mem_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_uart_mem_clk_en_pos
	);

	this->perip_clk_en0.system_usb_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_usb_clk_en_pos
	);

	this->perip_clk_en0.system_i2s1_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_i2s1_clk_en_pos
	);
	
	this->perip_clk_en0.system_pwm1_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_pwm1_clk_en_pos
	);

	this->perip_clk_en0.system_can_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_can_clk_en_pos
	);

	this->perip_clk_en0.system_i2c_ext1_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_i2c_ext1_clk_en_pos
	);

	this->perip_clk_en0.system_pwm0_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_pwm0_clk_en_pos
	);

	this->perip_clk_en0.system_spi3_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_spi3_clk_en_pos
	);

	this->perip_clk_en0.system_timergroup1_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_timergroup1_clk_en_pos
	);

	this->perip_clk_en0.system_timergroup_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_timergroup_clk_en_pos
	);

	this->perip_clk_en0.system_ledc_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_ledc_clk_en_pos
	);

	this->perip_clk_en0.system_pcnt_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_pcnt_clk_en_pos
	);

	this->perip_clk_en0.system_rmt_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_rmt_clk_en_pos
	);

	this->perip_clk_en0.system_uhci0_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_uhci0_clk_en_pos
	);

	this->perip_clk_en0.system_i2c_ext0_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_i2c_ext0_clk_en_pos
	);

	this->perip_clk_en0.system_spi2_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_spi2_clk_en_pos
	);

	this->perip_clk_en0.system_uart1_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_uart1_clk_en_pos
	);

	this->perip_clk_en0.system_i2s0_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_i2s0_clk_en_pos
	);

	this->perip_clk_en0.system_uart_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_uart_clk_en_pos
	);

	this->perip_clk_en0.system_spi01_clk_en = this->regGet(
		this->perip_clk_en0.val,
		this->perip_clk_en0.system_spi01_clk_en_pos
	);

		
	return this->perip_clk_en0.val;
}
uint32_t ES3SystemRegister::getSystem_perip_clk_en1(void){
	uint16_t base = this->perip_clk_en1.base;
	this->perip_clk_en1.val = this->regRead(base);

	this->perip_clk_en1.system_usb_device_clk_en = this->regGet(
		this->perip_clk_en1.val,
		this->perip_clk_en1.system_usb_device_clk_en_pos
	);

	this->perip_clk_en1.system_uart2_clk_en = this->regGet(
		this->perip_clk_en1.val,
		this->perip_clk_en1.system_uart2_clk_en_pos
	);

	this->perip_clk_en1.system_lcd_cam_clk_en = this->regGet(
		this->perip_clk_en1.val,
		this->perip_clk_en1.system_lcd_cam_clk_en_pos
	);

	this->perip_clk_en1.system_sdio_host_clk_en = this->regGet(
		this->perip_clk_en1.val,
		this->perip_clk_en1.system_sdio_host_clk_en_pos
	);

	this->perip_clk_en1.system_dma_clk_en = this->regGet(
		this->perip_clk_en1.val,
		this->perip_clk_en1.system_dma_clk_en_pos
	);

	this->perip_clk_en1.system_crypto_hmac_clk_en = this->regGet(
		this->perip_clk_en1.val,
		this->perip_clk_en1.system_crypto_hmac_clk_en_pos
	);

	this->perip_clk_en1.system_crypto_ds_clk_en = this->regGet(
		this->perip_clk_en1.val,
		this->perip_clk_en1.system_crypto_ds_clk_en_pos
	);

	this->perip_clk_en1.system_crypto_rsa_clk_en = this->regGet(
		this->perip_clk_en1.val,
		this->perip_clk_en1.system_crypto_rsa_clk_en_pos
	);

	this->perip_clk_en1.system_crypto_sha_clk_en = this->regGet(
		this->perip_clk_en1.val,
		this->perip_clk_en1.system_crypto_sha_clk_en_pos
	);

	this->perip_clk_en1.system_crypto_aes_clk_en = this->regGet(
		this->perip_clk_en1.val,
		this->perip_clk_en1.system_crypto_aes_clk_en_pos
	);

	this->perip_clk_en1.system_peri_backup_clk_en = this->regGet(
		this->perip_clk_en1.val,
		this->perip_clk_en1.system_peri_backup_clk_en_pos
	);
	
	return this->perip_clk_en1.val;
}
uint32_t ES3SystemRegister::getSystem_perip_rst_en0(void){
	uint16_t base = this->perip_rst_en0.base;
	this->perip_rst_en0.val = this->regRead(base);

	this->perip_rst_en0.system_adc2_arb_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_adc2_arb_rst_pos
	);

	this->perip_rst_en0.system_systimer_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_systimer_rst_pos
	);

	this->perip_rst_en0.system_apb_saradc_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_apb_saradc_rst_pos
	);

	this->perip_rst_en0.system_uart_mem_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_uart_mem_rst_pos
	);

	this->perip_rst_en0.system_usb_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_usb_rst_pos
	);

	this->perip_rst_en0.system_i2s1_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_i2s1_rst_pos
	);
	
	this->perip_rst_en0.system_pwm1_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_pwm1_rst_pos
	);

	this->perip_rst_en0.system_can_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_can_rst_pos
	);

	this->perip_rst_en0.system_i2c_ext1_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_i2c_ext1_rst_pos
	);

	this->perip_rst_en0.system_pwm0_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_pwm0_rst_pos
	);

	this->perip_rst_en0.system_spi3_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_spi3_rst_pos
	);

	this->perip_rst_en0.system_timergroup1_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_timergroup1_rst_pos
	);

	this->perip_rst_en0.system_timergroup_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_timergroup_rst_pos
	);

	this->perip_rst_en0.system_ledc_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_ledc_rst_pos
	);

	this->perip_rst_en0.system_pcnt_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_pcnt_rst_pos
	);

	this->perip_rst_en0.system_rmt_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_rmt_rst_pos
	);

	this->perip_rst_en0.system_uhci0_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_uhci0_rst_pos
	);

	this->perip_rst_en0.system_i2c_ext0_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_i2c_ext0_rst_pos
	);

	this->perip_rst_en0.system_spi2_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_spi2_rst_pos
	);

	this->perip_rst_en0.system_uart1_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_uart1_rst_pos
	);

	this->perip_rst_en0.system_i2s0_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_i2s0_rst_pos
	);

	this->perip_rst_en0.system_uart_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_uart_rst_pos
	);

	this->perip_rst_en0.system_spi01_rst = this->regGet(
		this->perip_rst_en0.val,
		this->perip_rst_en0.system_spi01_rst_pos
	);

		
	return this->perip_rst_en0.val;

}
uint32_t ES3SystemRegister::getSystem_perip_rst_en1(void){
	uint16_t base = this->perip_rst_en1.base;
	this->perip_rst_en1.val = this->regRead(base);

	this->perip_rst_en1.system_usb_device_rst = this->regGet(
		this->perip_rst_en1.val,
		this->perip_rst_en1.system_usb_device_rst_pos
	);

	this->perip_rst_en1.system_uart2_rst = this->regGet(
		this->perip_rst_en1.val,
		this->perip_rst_en1.system_uart2_rst_pos
	);

	this->perip_rst_en1.system_lcd_cam_rst = this->regGet(
		this->perip_rst_en1.val,
		this->perip_rst_en1.system_lcd_cam_rst_pos
	);

	this->perip_rst_en1.system_sdio_host_rst = this->regGet(
		this->perip_rst_en1.val,
		this->perip_rst_en1.system_sdio_host_rst_pos
	);

	this->perip_rst_en1.system_dma_rst = this->regGet(
		this->perip_rst_en1.val,
		this->perip_rst_en1.system_dma_rst_pos
	);

	this->perip_rst_en1.system_crypto_hmac_rst = this->regGet(
		this->perip_rst_en1.val,
		this->perip_rst_en1.system_crypto_hmac_rst_pos
	);

	this->perip_rst_en1.system_crypto_ds_rst = this->regGet(
		this->perip_rst_en1.val,
		this->perip_rst_en1.system_crypto_ds_rst_pos
	);

	this->perip_rst_en1.system_crypto_rsa_rst = this->regGet(
		this->perip_rst_en1.val,
		this->perip_rst_en1.system_crypto_rsa_rst_pos
	);

	this->perip_rst_en1.system_crypto_sha_rst = this->regGet(
		this->perip_rst_en1.val,
		this->perip_rst_en1.system_crypto_sha_rst_pos
	);

	this->perip_rst_en1.system_crypto_aes_rst = this->regGet(
		this->perip_rst_en1.val,
		this->perip_rst_en1.system_crypto_aes_rst_pos
	);

	this->perip_rst_en1.system_peri_backup_rst = this->regGet(
		this->perip_rst_en1.val,
		this->perip_rst_en1.system_peri_backup_rst_pos
	);
	
	return this->perip_rst_en1.val;

}
uint32_t ES3SystemRegister::getSystem_bt_lpck_div_frac(void){
	uint16_t base = this->bt_lpck_div_frac.base;
	this->bt_lpck_div_frac.val = this->regRead(base);

	this->bt_lpck_div_frac.system_lpclk_rtc_en = this->regGet(
		this->bt_lpck_div_frac.val,
		this->bt_lpck_div_frac.system_lpclk_rtc_en_pos
	);

	this->bt_lpck_div_frac.system_lpclk_sel_xtal32k = this->regGet(
		this->bt_lpck_div_frac.val,
		this->bt_lpck_div_frac.system_lpclk_sel_xtal32k_pos
	);

	this->bt_lpck_div_frac.system_lpclk_sel_xtal = this->regGet(
		this->bt_lpck_div_frac.val,
		this->bt_lpck_div_frac.system_lpclk_sel_xtal_pos
	);

	this->bt_lpck_div_frac.system_lpclk_sel_8m = this->regGet(
		this->bt_lpck_div_frac.val,
		this->bt_lpck_div_frac.system_lpclk_sel_8m_pos
	);

	this->bt_lpck_div_frac.system_lpclk_sel_rtc_slow = this->regGet(
		this->bt_lpck_div_frac.val,
		this->bt_lpck_div_frac.system_lpclk_sel_rtc_slow_pos
	);
	
	return this->bt_lpck_div_frac.val;
}
uint32_t ES3SystemRegister::getSystem_cpu_intr_from_cpu_0(void){
	uint16_t base = this->cpu_intr_from_cpu_0.base;
	this->cpu_intr_from_cpu_0.val = this->regRead(base);

	this->cpu_intr_from_cpu_0.system_cpu_intr_from_cpu_0 = this->regGet(
		this->cpu_intr_from_cpu_0.val,
		this->cpu_intr_from_cpu_0.system_cpu_intr_from_cpu_0_pos
	);

	return this->cpu_intr_from_cpu_0.val;
}
uint32_t ES3SystemRegister::getSystem_cpu_intr_from_cpu_1(void){
	uint16_t base = this->cpu_intr_from_cpu_1.base;
	this->cpu_intr_from_cpu_1.val = this->regRead(base);

	this->cpu_intr_from_cpu_1.system_cpu_intr_from_cpu_1 = this->regGet(
		this->cpu_intr_from_cpu_1.val,
		this->cpu_intr_from_cpu_1.system_cpu_intr_from_cpu_1_pos
	);

	return this->cpu_intr_from_cpu_1.val;
}
uint32_t ES3SystemRegister::getSystem_cpu_intr_from_cpu_2(void){
	uint16_t base = this->cpu_intr_from_cpu_2.base;
	this->cpu_intr_from_cpu_2.val = this->regRead(base);

	this->cpu_intr_from_cpu_2.system_cpu_intr_from_cpu_2 = this->regGet(
		this->cpu_intr_from_cpu_2.val,
		this->cpu_intr_from_cpu_2.system_cpu_intr_from_cpu_2_pos
	);

	return this->cpu_intr_from_cpu_2.val;

}
uint32_t ES3SystemRegister::getSystem_cpu_intr_from_cpu_3(void){
	uint16_t base = this->cpu_intr_from_cpu_3.base;
	this->cpu_intr_from_cpu_3.val = this->regRead(base);

	this->cpu_intr_from_cpu_3.system_cpu_intr_from_cpu_3 = this->regGet(
		this->cpu_intr_from_cpu_3.val,
		this->cpu_intr_from_cpu_3.system_cpu_intr_from_cpu_3_pos
	);

	return this->cpu_intr_from_cpu_3.val;

}
uint32_t ES3SystemRegister::getSystem_rsa_pd_ctrl(void){
	uint16_t base = this->rsa_pd_ctrl.base;
	this->rsa_pd_ctrl.val = this->regRead(base);

	this->rsa_pd_ctrl.system_rsa_mem_force_pd = this->regGet(
		this->rsa_pd_ctrl.val,
		this->rsa_pd_ctrl.system_rsa_mem_force_pd_pos
	);

	this->rsa_pd_ctrl.system_rsa_mem_force_pu = this->regGet(
		this->rsa_pd_ctrl.val,
		this->rsa_pd_ctrl.system_rsa_mem_force_pu_pos
	);

	this->rsa_pd_ctrl.system_rsa_mem_pd = this->regGet(
		this->rsa_pd_ctrl.val,
		this->rsa_pd_ctrl.system_rsa_mem_pd_pos
	);

	return this->rsa_pd_ctrl.val;
}
uint32_t ES3SystemRegister::getSystem_edma_ctrl(void){
	uint16_t base = this->edma_ctrl.base;
	this->edma_ctrl.val = this->regRead(base);

	this->edma_ctrl.system_edma_reset = this->regGet(
		this->edma_ctrl.val,
		this->edma_ctrl.system_edma_reset_pos
	);

	this->edma_ctrl.system_edma_clk_on = this->regGet(
		this->edma_ctrl.val,
		this->edma_ctrl.system_edma_clk_on_pos
	);

	return this->edma_ctrl.val;

}
uint32_t ES3SystemRegister::getSystem_cache_control(void){
	uint16_t base = this->cache_control.base;
	this->cache_control.val = this->regRead(base);

	this->cache_control.system_dcache_reset = this->regGet(
		this->cache_control.val,
		this->cache_control.system_dcache_reset_pos
	);

	this->cache_control.system_dcache_clk_on = this->regGet(
		this->cache_control.val,
		this->cache_control.system_dcache_clk_on_pos
	);

	this->cache_control.system_icache_reset = this->regGet(
		this->cache_control.val,
		this->cache_control.system_icache_reset_pos
	);

	this->cache_control.system_icache_clk_on = this->regGet(
		this->cache_control.val,
		this->cache_control.system_icache_clk_on_pos
	);

	return this->cache_control.val;
}
uint32_t ES3SystemRegister::getSystem_external_device_encrypt_decrypt_control(void){
	uint16_t base = this->external_device_encrypt_decrypt_control.base;
	this->external_device_encrypt_decrypt_control.val = regRead(base);
	this->external_device_encrypt_decrypt_control.system_enable_download_manual_encrypt = this->regGet(
		this->external_device_encrypt_decrypt_control.val,
		this->external_device_encrypt_decrypt_control.system_enable_download_manual_encrypt_pos
	);

	this->external_device_encrypt_decrypt_control.system_enable_download_g0cb_decrypt = this->regGet(
		this->external_device_encrypt_decrypt_control.val,
		this->external_device_encrypt_decrypt_control.system_enable_download_g0cb_decrypt_pos
	);

	this->external_device_encrypt_decrypt_control.system_enable_download_db_encrypt = this->regGet(
		this->external_device_encrypt_decrypt_control.val,
		this->external_device_encrypt_decrypt_control.system_enable_download_db_encrypt_pos
	);

	this->external_device_encrypt_decrypt_control.system_enable_spi_manual_encrypt = this->regGet(
		this->external_device_encrypt_decrypt_control.val,
		this->external_device_encrypt_decrypt_control.system_enable_spi_manual_encrypt_pos
	);

	return this->external_device_encrypt_decrypt_control.val;
}
uint32_t ES3SystemRegister::getSystem_rtc_fastmem_config(void){
	uint16_t base = this->rtc_fastmem_config.base;
	this->rtc_fastmem_config.val = this->regRead(base);

	this->rtc_fastmem_config.system_rtc_mem_crc_finish = this->regGet(
		this->rtc_fastmem_config.val,
		this->rtc_fastmem_config.system_rtc_mem_crc_finish_pos
	);

	this->rtc_fastmem_config.system_rtc_mem_crc_len = this->regGet(
		this->rtc_fastmem_config.val,
		this->rtc_fastmem_config.system_rtc_mem_crc_len_x,
		this->rtc_fastmem_config.system_rtc_mem_crc_len_y
	);

	this->rtc_fastmem_config.system_rtc_mem_crc_addr = this->regGet(
		this->rtc_fastmem_config.val,
		this->rtc_fastmem_config.system_rtc_mem_crc_addr_x,
		this->rtc_fastmem_config.system_rtc_mem_crc_addr_y
	);

	this->rtc_fastmem_config.system_rtc_mem_crc_start = this->regGet(
		this->rtc_fastmem_config.val,
		this->rtc_fastmem_config.system_rtc_mem_crc_start_pos
	);

	return this->rtc_fastmem_config.val;
}
uint32_t ES3SystemRegister::getSystem_rtc_fastmem_crc(void){ 
	uint16_t base = this->rtc_fastmem_crc.base;
	this->rtc_fastmem_crc.val = this->regRead(base);

	this->rtc_fastmem_crc.system_rtc_mem_crc_res = this->regGet(
		this->rtc_fastmem_crc.val,
		this->rtc_fastmem_crc.system_rtc_mem_crc_res_x,
		this->rtc_fastmem_crc.system_rtc_mem_crc_res_y
	);
	return this->rtc_fastmem_crc.val;
}
uint32_t ES3SystemRegister::getSystem_clock_gate(void){ 
	uint16_t base = this->clock_gate.base;
	this->clock_gate.val = this->regRead(base);

	this->clock_gate.system_clk_en = this->regGet(
		this->clock_gate.val,
		this->clock_gate.system_clk_en_pos
	);
	
	return this->clock_gate.val;
}
uint32_t ES3SystemRegister::getSystem_sysclk_conf(void){ 
	uint16_t base = this->sysclk_conf.base;
	this->sysclk_conf.val = this->regRead(base);

	this->sysclk_conf.system_clk_xtal_freq = this->regGet(
		this->sysclk_conf.val,
		this->sysclk_conf.system_clk_xtal_freq_x,
		this->sysclk_conf.system_clk_xtal_freq_y
	);
	this->sysclk_conf.system_soc_clk_sel = this->regGet(
		this->sysclk_conf.val,
		this->sysclk_conf.system_soc_clk_sel_x,
		this->sysclk_conf.system_soc_clk_sel_y
	);
	this->sysclk_conf.system_pre_div_cnt = this->regGet(
		this->sysclk_conf.val,
		this->sysclk_conf.system_pre_div_cnt_x,
		this->sysclk_conf.system_pre_div_cnt_y
	);

	return this->sysclk_conf.val;
}
uint32_t ES3SystemRegister::getSystem_date(void){ 
	uint16_t base = this->date.base;
	this->date.val = this->regRead(base);


	this->date.system_date = this->regGet(
		this->date.val,
		this->date.system_date_x,
		this->date.system_date_y
	);

	return this->date.val;
}
uint32_t ES3SystemRegister::getApb_ctrl_clkgate_force_on(void){ 
	uint16_t base = this->apb_ctrl_clkgate_force_on.base;
	this->apb_ctrl_clkgate_force_on.val = this->regRead(base);

	this->apb_ctrl_clkgate_force_on.apb_ctrl_sram_clkgate_force_on = this->regGet(
		this->apb_ctrl_clkgate_force_on.val,
		this->apb_ctrl_clkgate_force_on.apb_ctrl_sram_clkgate_force_on_x,
		this->apb_ctrl_clkgate_force_on.apb_ctrl_sram_clkgate_force_on_y
	);

	this->apb_ctrl_clkgate_force_on.apb_ctrl_rom_clkgate_force_on = this->regGet(
		this->apb_ctrl_clkgate_force_on.val,
		this->apb_ctrl_clkgate_force_on.apb_ctrl_rom_clkgate_force_on_x,
		this->apb_ctrl_clkgate_force_on.apb_ctrl_rom_clkgate_force_on_y
	);
	
	return this->apb_ctrl_clkgate_force_on.val;
}
uint32_t ES3SystemRegister::getApb_ctrl_mem_power_down(void){ 
	uint16_t base = this->apb_ctrl_mem_power_down.base;
	this->apb_ctrl_mem_power_down.val = this->regRead(base);

	this->apb_ctrl_mem_power_down.apb_ctrl_sram_power_down = this->regGet(
		this->apb_ctrl_mem_power_down.val,
		this->apb_ctrl_mem_power_down.apb_ctrl_sram_power_down_x,
		this->apb_ctrl_mem_power_down.apb_ctrl_sram_power_down_y
	);

	this->apb_ctrl_mem_power_down.apb_ctrl_rom_power_down = this->regGet(
		this->apb_ctrl_mem_power_down.val,
		this->apb_ctrl_mem_power_down.apb_ctrl_rom_power_down_x,
		this->apb_ctrl_mem_power_down.apb_ctrl_rom_power_down_y
	);

	return this->apb_ctrl_mem_power_down.val;
}
uint32_t ES3SystemRegister::getApb_ctrl_mem_power_up(void){ 
	uint16_t base = this->apb_ctrl_mem_power_up.base;
	this->apb_ctrl_mem_power_up.val = this->regRead(base);

	this->apb_ctrl_mem_power_up.apb_ctrl_sram_power_up = this->regGet(
		this->apb_ctrl_mem_power_up.val,
		this->apb_ctrl_mem_power_up.apb_ctrl_sram_power_up_x,
		this->apb_ctrl_mem_power_up.apb_ctrl_sram_power_up_y
	);

	this->apb_ctrl_mem_power_up.apb_ctrl_rom_power_up = this->regGet(
		this->apb_ctrl_mem_power_up.val,
		this->apb_ctrl_mem_power_up.apb_ctrl_rom_power_up_x,
		this->apb_ctrl_mem_power_up.apb_ctrl_rom_power_up_y
	);
	
	return this->apb_ctrl_mem_power_up.val;
}

///// SETTERS
void ES3SystemRegister::setSystem_core_1_control_0(void){
	uint32_t r = this->core_1_control_0.val;

	r = this->regSet(r, 	this->core_1_control_0.system_control_core_1_reseting,
		this->core_1_control_0.system_control_core_1_reseting_pos
	);
	r = this->regSet(r, 	this->core_1_control_0.system_control_core_1_clkgate_en,
		this->core_1_control_0.system_control_core_1_clkgate_en_pos
	);
	r = this->regSet(r, 	this->core_1_control_0.system_control_core_1_runstall,
		this->core_1_control_0.system_control_core_1_runstall_pos
	);

	this->regWrite(this->core_1_control_0.base, r);
}
void ES3SystemRegister::setSystem_core_1_control_1(void){
	uint32_t r = this->core_1_control_1.val;

	r = this->regSet(r, 	this->core_1_control_1.system_control_core_1_message,
		this->core_1_control_1.system_control_core_1_message_x,
		this->core_1_control_1.system_control_core_1_message_y
	);
	
	this->regWrite(this->core_1_control_1.base, r);
}
void ES3SystemRegister::setSystem_cpu_per_conf(void){
	uint32_t r = this->cpu_per_conf.val;

	r = this->regSet(r, 	this->cpu_per_conf.system_cpu_waiti_delay_num,
		this->cpu_per_conf.system_cpu_waiti_delay_num_x,
		this->cpu_per_conf.system_cpu_waiti_delay_num_y
	);

	r = this->regSet(r, 	this->cpu_per_conf.system_cpu_wait_mode_force_on,
		this->cpu_per_conf.system_cpu_wait_mode_force_on_pos
	);

	r = this->regSet(r, 	this->cpu_per_conf.system_pll_freq_sel,
		this->cpu_per_conf.system_pll_freq_sel_pos
	);

	r = this->regSet(r, 	this->cpu_per_conf.system_cpuperiod_sel,
		this->cpu_per_conf.system_cpuperiod_sel_x,
		this->cpu_per_conf.system_cpuperiod_sel_y
	);

	this->regWrite(this->cpu_per_conf.base, r);
}
void ES3SystemRegister::setSystem_perip_clk_en0(void){
	uint32_t r = this->perip_clk_en0.val;

	r = this->regSet(r,     this->perip_clk_en0.system_adc2_arb_clk_en,
		this->perip_clk_en0.system_adc2_arb_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_systimer_clk_en,
		this->perip_clk_en0.system_systimer_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_apb_saradc_clk_en,
		this->perip_clk_en0.system_apb_saradc_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_uart_mem_clk_en,
		this->perip_clk_en0.system_uart_mem_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_usb_clk_en,
		this->perip_clk_en0.system_usb_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_i2s1_clk_en,
		this->perip_clk_en0.system_i2s1_clk_en_pos
	);
	
	r = this->regSet(r,     this->perip_clk_en0.system_pwm1_clk_en,
		this->perip_clk_en0.system_pwm1_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_can_clk_en,
		this->perip_clk_en0.system_can_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_i2c_ext1_clk_en,
		this->perip_clk_en0.system_i2c_ext1_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_pwm0_clk_en,
		this->perip_clk_en0.system_pwm0_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_spi3_clk_en,
		this->perip_clk_en0.system_spi3_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_timergroup1_clk_en,
		this->perip_clk_en0.system_timergroup1_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_timergroup_clk_en,
		this->perip_clk_en0.system_timergroup_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_ledc_clk_en,
		this->perip_clk_en0.system_ledc_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_pcnt_clk_en,
		this->perip_clk_en0.system_pcnt_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_rmt_clk_en,
		this->perip_clk_en0.system_rmt_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_uhci0_clk_en,
		this->perip_clk_en0.system_uhci0_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_i2c_ext0_clk_en,
		this->perip_clk_en0.system_i2c_ext0_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_spi2_clk_en,
		this->perip_clk_en0.system_spi2_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_uart1_clk_en,
		this->perip_clk_en0.system_uart1_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_i2s0_clk_en,
		this->perip_clk_en0.system_i2s0_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_uart_clk_en,
		this->perip_clk_en0.system_uart_clk_en_pos
	);

	r = this->regSet(r,     this->perip_clk_en0.system_spi01_clk_en,
		this->perip_clk_en0.system_spi01_clk_en_pos
	);
	
	this->regWrite(this->perip_clk_en0.base, r);
}
void ES3SystemRegister::setSystem_perip_clk_en1(void){
	uint32_t r = this->perip_clk_en1.val;

	r = this->regSet(r, 	this->perip_clk_en1.system_usb_device_clk_en,
		this->perip_clk_en1.system_usb_device_clk_en_pos
	);

	r = this->regSet(r, 	this->perip_clk_en1.system_uart2_clk_en,
		this->perip_clk_en1.system_uart2_clk_en_pos
	);

	r = this->regSet(r, 	this->perip_clk_en1.system_lcd_cam_clk_en,
		this->perip_clk_en1.system_lcd_cam_clk_en_pos
	);

	r = this->regSet(r, 	this->perip_clk_en1.system_sdio_host_clk_en,
		this->perip_clk_en1.system_sdio_host_clk_en_pos
	);

	r = this->regSet(r, 	this->perip_clk_en1.system_dma_clk_en,
		this->perip_clk_en1.system_dma_clk_en_pos
	);

	r = this->regSet(r, 	this->perip_clk_en1.system_crypto_hmac_clk_en,
		this->perip_clk_en1.system_crypto_hmac_clk_en_pos
	);

	r = this->regSet(r, 	this->perip_clk_en1.system_crypto_ds_clk_en,
		this->perip_clk_en1.system_crypto_ds_clk_en_pos
	);

	r = this->regSet(r, 	this->perip_clk_en1.system_crypto_rsa_clk_en,
		this->perip_clk_en1.system_crypto_rsa_clk_en_pos
	);

	r = this->regSet(r, 	this->perip_clk_en1.system_crypto_sha_clk_en,
		this->perip_clk_en1.system_crypto_sha_clk_en_pos
	);

	r = this->regSet(r, 	this->perip_clk_en1.system_crypto_aes_clk_en,
		this->perip_clk_en1.system_crypto_aes_clk_en_pos
	);

	r = this->regSet(r, 	this->perip_clk_en1.system_peri_backup_clk_en,
		this->perip_clk_en1.system_peri_backup_clk_en_pos
	);
	
	this->regWrite(this->perip_clk_en1.base, r);
}
void ES3SystemRegister::setSystem_perip_rst_en0(void){
	uint32_t r = this->perip_rst_en0.val;

	r = this->regSet(r,     this->perip_rst_en0.system_adc2_arb_rst,
		this->perip_rst_en0.system_adc2_arb_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_systimer_rst,
		this->perip_rst_en0.system_systimer_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_apb_saradc_rst,
		this->perip_rst_en0.system_apb_saradc_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_uart_mem_rst,
		this->perip_rst_en0.system_uart_mem_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_usb_rst,
		this->perip_rst_en0.system_usb_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_i2s1_rst,
		this->perip_rst_en0.system_i2s1_rst_pos
	);
	
	r = this->regSet(r,     this->perip_rst_en0.system_pwm1_rst,
		this->perip_rst_en0.system_pwm1_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_can_rst,
		this->perip_rst_en0.system_can_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_i2c_ext1_rst,
		this->perip_rst_en0.system_i2c_ext1_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_pwm0_rst,
		this->perip_rst_en0.system_pwm0_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_spi3_rst,
		this->perip_rst_en0.system_spi3_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_timergroup1_rst,
		this->perip_rst_en0.system_timergroup1_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_timergroup_rst,
		this->perip_rst_en0.system_timergroup_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_ledc_rst,
		this->perip_rst_en0.system_ledc_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_pcnt_rst,
		this->perip_rst_en0.system_pcnt_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_rmt_rst,
		this->perip_rst_en0.system_rmt_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_uhci0_rst,
		this->perip_rst_en0.system_uhci0_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_i2c_ext0_rst,
		this->perip_rst_en0.system_i2c_ext0_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_spi2_rst,
		this->perip_rst_en0.system_spi2_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_uart1_rst,
		this->perip_rst_en0.system_uart1_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_i2s0_rst,
		this->perip_rst_en0.system_i2s0_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_uart_rst,
		this->perip_rst_en0.system_uart_rst_pos
	);

	r = this->regSet(r,     this->perip_rst_en0.system_spi01_rst,
		this->perip_rst_en0.system_spi01_rst_pos
	);
	

	this->regWrite(this->perip_rst_en0.base, r);
}
void ES3SystemRegister::setSystem_perip_rst_en1(void){
	uint32_t r = this->perip_rst_en1.val;

	r = this->regSet(r, 	this->perip_rst_en1.system_usb_device_rst,
		this->perip_rst_en1.system_usb_device_rst_pos
	);

	r = this->regSet(r, 	this->perip_rst_en1.system_uart2_rst,
		this->perip_rst_en1.system_uart2_rst_pos
	);

	r = this->regSet(r, 	this->perip_rst_en1.system_lcd_cam_rst,
		this->perip_rst_en1.system_lcd_cam_rst_pos
	);

	r = this->regSet(r, 	this->perip_rst_en1.system_sdio_host_rst,
		this->perip_rst_en1.system_sdio_host_rst_pos
	);

	r = this->regSet(r, 	this->perip_rst_en1.system_dma_rst,
		this->perip_rst_en1.system_dma_rst_pos
	);

	r = this->regSet(r, 	this->perip_rst_en1.system_crypto_hmac_rst,
		this->perip_rst_en1.system_crypto_hmac_rst_pos
	);

	r = this->regSet(r, 	this->perip_rst_en1.system_crypto_ds_rst,
		this->perip_rst_en1.system_crypto_ds_rst_pos
	);

	r = this->regSet(r, 	this->perip_rst_en1.system_crypto_rsa_rst,
		this->perip_rst_en1.system_crypto_rsa_rst_pos
	);

	r = this->regSet(r, 	this->perip_rst_en1.system_crypto_sha_rst,
		this->perip_rst_en1.system_crypto_sha_rst_pos
	);

	r = this->regSet(r, 	this->perip_rst_en1.system_crypto_aes_rst,
		this->perip_rst_en1.system_crypto_aes_rst_pos
	);

	r = this->regSet(r, 	this->perip_rst_en1.system_peri_backup_rst,
		this->perip_rst_en1.system_peri_backup_rst_pos
	);
	

	this->regWrite(this->perip_rst_en1.base, r);
}
void ES3SystemRegister::setSystem_bt_lpck_div_frac(void){
	uint32_t r = this->bt_lpck_div_frac.val;

	r = this->regSet(r, 	this->bt_lpck_div_frac.system_lpclk_rtc_en,
		this->bt_lpck_div_frac.system_lpclk_rtc_en_pos
	);

	r = this->regSet(r, 	this->bt_lpck_div_frac.system_lpclk_sel_xtal32k,
		this->bt_lpck_div_frac.system_lpclk_sel_xtal32k_pos
	);

	r = this->regSet(r, 	this->bt_lpck_div_frac.system_lpclk_sel_xtal,
		this->bt_lpck_div_frac.system_lpclk_sel_xtal_pos
	);

	r = this->regSet(r, 	this->bt_lpck_div_frac.system_lpclk_sel_8m,
		this->bt_lpck_div_frac.system_lpclk_sel_8m_pos
	);

	r = this->regSet(r, 	this->bt_lpck_div_frac.system_lpclk_sel_rtc_slow,
		this->bt_lpck_div_frac.system_lpclk_sel_rtc_slow_pos
	);
	
	this->regWrite(this->bt_lpck_div_frac.base, r);
}
void ES3SystemRegister::setSystem_cpu_intr_from_cpu_0(void){
	uint32_t r = this->cpu_intr_from_cpu_0.val;

	r = this->regSet(r, 	this->cpu_intr_from_cpu_0.system_cpu_intr_from_cpu_0,
		this->cpu_intr_from_cpu_0.system_cpu_intr_from_cpu_0_pos
	);

	this->regWrite(this->cpu_intr_from_cpu_0.base, r);
}
void ES3SystemRegister::setSystem_cpu_intr_from_cpu_1(void){
	uint32_t r = this->cpu_intr_from_cpu_1.val;

	r = this->regSet(r, 	this->cpu_intr_from_cpu_1.system_cpu_intr_from_cpu_1,
		this->cpu_intr_from_cpu_1.system_cpu_intr_from_cpu_1_pos
	);

	this->regWrite(this->cpu_intr_from_cpu_1.base, r);
}
void ES3SystemRegister::setSystem_cpu_intr_from_cpu_2(void){
	uint32_t r = this->cpu_intr_from_cpu_2.val;

	r = this->regSet(r, 	this->cpu_intr_from_cpu_2.system_cpu_intr_from_cpu_2,
		this->cpu_intr_from_cpu_2.system_cpu_intr_from_cpu_2_pos
	);


	this->regWrite(this->cpu_intr_from_cpu_2.base, r);
}
void ES3SystemRegister::setSystem_cpu_intr_from_cpu_3(void){
	uint32_t r = this->cpu_intr_from_cpu_3.val;

	r = this->regSet(r, 	this->cpu_intr_from_cpu_3.system_cpu_intr_from_cpu_3,
		this->cpu_intr_from_cpu_3.system_cpu_intr_from_cpu_3_pos
	);


	this->regWrite(this->cpu_intr_from_cpu_3.base, r);
}
void ES3SystemRegister::setSystem_rsa_pd_ctrl(void){
	uint32_t r = this->rsa_pd_ctrl.val;

	r = this->regSet(r, 	this->rsa_pd_ctrl.system_rsa_mem_force_pd,
		this->rsa_pd_ctrl.system_rsa_mem_force_pd_pos
	);

	r = this->regSet(r, 	this->rsa_pd_ctrl.system_rsa_mem_force_pu,
		this->rsa_pd_ctrl.system_rsa_mem_force_pu_pos
	);

	r = this->regSet(r, 	this->rsa_pd_ctrl.system_rsa_mem_pd,
		this->rsa_pd_ctrl.system_rsa_mem_pd_pos
	);

	this->regWrite(this->rsa_pd_ctrl.base, r);
}
void ES3SystemRegister::setSystem_edma_ctrl(void){
	uint32_t r = this->edma_ctrl.val;

	r = this->regSet(r, 	this->edma_ctrl.system_edma_reset,
		this->edma_ctrl.system_edma_reset_pos
	);

	r = this->regSet(r, 	this->edma_ctrl.system_edma_clk_on,
		this->edma_ctrl.system_edma_clk_on_pos
	);


	this->regWrite(this->edma_ctrl.base, r);
}
void ES3SystemRegister::setSystem_cache_control(void){
	uint32_t r = this->cache_control.val;

	r = this->regSet(r, 	this->cache_control.system_dcache_reset,
		this->cache_control.system_dcache_reset_pos
	);

	r = this->regSet(r, 	this->cache_control.system_dcache_clk_on,
		this->cache_control.system_dcache_clk_on_pos
	);

	r = this->regSet(r, 	this->cache_control.system_icache_reset,
		this->cache_control.system_icache_reset_pos
	);

	r = this->regSet(r, 	this->cache_control.system_icache_clk_on,
		this->cache_control.system_icache_clk_on_pos
	);

	this->regWrite(this->cache_control.base, r);
}
void ES3SystemRegister::setSystem_external_device_encrypt_decrypt_control(void){
	uint32_t r = this->external_device_encrypt_decrypt_control.val;
	r = this->regSet(r, 	this->external_device_encrypt_decrypt_control.system_enable_download_manual_encrypt,
		this->external_device_encrypt_decrypt_control.system_enable_download_manual_encrypt_pos
	);

	r = this->regSet(r, 	this->external_device_encrypt_decrypt_control.system_enable_download_g0cb_decrypt,
		this->external_device_encrypt_decrypt_control.system_enable_download_g0cb_decrypt_pos
	);

	r = this->regSet(r, 	this->external_device_encrypt_decrypt_control.system_enable_download_db_encrypt,
		this->external_device_encrypt_decrypt_control.system_enable_download_db_encrypt_pos
	);

	r = this->regSet(r, 	this->external_device_encrypt_decrypt_control.system_enable_spi_manual_encrypt,
		this->external_device_encrypt_decrypt_control.system_enable_spi_manual_encrypt_pos
	);

	this->regWrite(this->external_device_encrypt_decrypt_control.base, r);
}
void ES3SystemRegister::setSystem_rtc_fastmem_config(void){
	uint32_t r = this->rtc_fastmem_config.val;

	r = this->regSet(r, 	this->rtc_fastmem_config.system_rtc_mem_crc_finish,
		this->rtc_fastmem_config.system_rtc_mem_crc_finish_pos
	);

	r = this->regSet(r, 	this->rtc_fastmem_config.system_rtc_mem_crc_len,
		this->rtc_fastmem_config.system_rtc_mem_crc_len_x,
		this->rtc_fastmem_config.system_rtc_mem_crc_len_y
	);

	r = this->regSet(r, 	this->rtc_fastmem_config.system_rtc_mem_crc_addr,
		this->rtc_fastmem_config.system_rtc_mem_crc_addr_x,
		this->rtc_fastmem_config.system_rtc_mem_crc_addr_y
	);

	r = this->regSet(r, 	this->rtc_fastmem_config.system_rtc_mem_crc_start,
		this->rtc_fastmem_config.system_rtc_mem_crc_start_pos
	);

	this->regWrite(this->rtc_fastmem_config.base, r);
}
void ES3SystemRegister::setSystem_rtc_fastmem_crc(void){ 
	uint32_t r = this->rtc_fastmem_crc.val;

	r = this->regSet(r, 	this->rtc_fastmem_crc.system_rtc_mem_crc_res,
		this->rtc_fastmem_crc.system_rtc_mem_crc_res_x,
		this->rtc_fastmem_crc.system_rtc_mem_crc_res_y
	);
	this->regWrite(this->rtc_fastmem_crc.base, r);
}
void ES3SystemRegister::setSystem_clock_gate(void){ 
	uint32_t r = this->clock_gate.val;

	r = this->regSet(r, 	this->clock_gate.system_clk_en,
		this->clock_gate.system_clk_en_pos
	);
	
	this->regWrite(this->clock_gate.base, r);
}
void ES3SystemRegister::setSystem_sysclk_conf(void){ 
	uint32_t r = this->sysclk_conf.val;

	r = this->regSet(r, 	this->sysclk_conf.system_clk_xtal_freq,
		this->sysclk_conf.system_clk_xtal_freq_x,
		this->sysclk_conf.system_clk_xtal_freq_y
	);
	r = this->regSet(r, 	this->sysclk_conf.system_soc_clk_sel,
		this->sysclk_conf.system_soc_clk_sel_x,
		this->sysclk_conf.system_soc_clk_sel_y
	);
	r = this->regSet(r, 	this->sysclk_conf.system_pre_div_cnt,
		this->sysclk_conf.system_pre_div_cnt_x,
		this->sysclk_conf.system_pre_div_cnt_y
	);

	this->regWrite(this->sysclk_conf.base, r);
}
void ES3SystemRegister::setSystem_date(void){ 
	uint32_t r = this->date.val;


	r = this->regSet(r, 	this->date.system_date,
		this->date.system_date_x,
		this->date.system_date_y
	);

	this->regWrite(this->date.base, r);
}
void ES3SystemRegister::setApb_ctrl_clkgate_force_on(void){ 
	uint32_t r = this->apb_ctrl_clkgate_force_on.val;

	r = this->regSet(r, 	this->apb_ctrl_clkgate_force_on.apb_ctrl_sram_clkgate_force_on,
		this->apb_ctrl_clkgate_force_on.apb_ctrl_sram_clkgate_force_on_x,
		this->apb_ctrl_clkgate_force_on.apb_ctrl_sram_clkgate_force_on_y
	);

	r = this->regSet(r, 	this->apb_ctrl_clkgate_force_on.apb_ctrl_rom_clkgate_force_on,
		this->apb_ctrl_clkgate_force_on.apb_ctrl_rom_clkgate_force_on_x,
		this->apb_ctrl_clkgate_force_on.apb_ctrl_rom_clkgate_force_on_y
	);
	
	this->regWrite(this->apb_ctrl_clkgate_force_on.base, r);
}
void ES3SystemRegister::setApb_ctrl_mem_power_down(void){ 
	uint32_t r = this->apb_ctrl_mem_power_down.val;

	r = this->regSet(r, 	this->apb_ctrl_mem_power_down.apb_ctrl_sram_power_down,
		this->apb_ctrl_mem_power_down.apb_ctrl_sram_power_down_x,
		this->apb_ctrl_mem_power_down.apb_ctrl_sram_power_down_y
	);

	r = this->regSet(r, 	this->apb_ctrl_mem_power_down.apb_ctrl_rom_power_down,
		this->apb_ctrl_mem_power_down.apb_ctrl_rom_power_down_x,
		this->apb_ctrl_mem_power_down.apb_ctrl_rom_power_down_y
	);
	this->regWrite(this->apb_ctrl_mem_power_down.base, r);
}
void ES3SystemRegister::setApb_ctrl_mem_power_up(void){ 
	uint32_t r = this->apb_ctrl_mem_power_up.val;

	r = this->regSet(r, 	this->apb_ctrl_mem_power_up.apb_ctrl_sram_power_up,
		this->apb_ctrl_mem_power_up.apb_ctrl_sram_power_up_x,
		this->apb_ctrl_mem_power_up.apb_ctrl_sram_power_up_y
	);

	r = this->regSet(r, 	this->apb_ctrl_mem_power_up.apb_ctrl_rom_power_up,
		this->apb_ctrl_mem_power_up.apb_ctrl_rom_power_up_x,
		this->apb_ctrl_mem_power_up.apb_ctrl_rom_power_up_y
	);
	this->regWrite(this->apb_ctrl_mem_power_up.base, r);
}

