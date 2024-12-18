struct system_core_1_control_0_reg{
	uint32_t base = (0x0000);
	uint32_t val = 0;
	
	bool system_control_core_1_reseting = 1;
	int system_control_core_1_reseting_pos =  2;
	
	bool system_control_core_1_clkgate_en =  0;
	int system_control_core_1_clkgate_en_pos = 1;

	bool system_control_core_1_runstall = 0;
	int system_control_core_1_runstall_pos = 0;
};

struct system_core_1_control_1_reg{
	uint32_t base = (0x0004);
	uint32_t val = 0;
	uint32_t system_control_core_1_message = 0;
	int system_control_core_1_message_x = 0;
	int system_control_core_1_message_y = 31;
};

struct system_cpu_per_conf_reg{
	uint32_t base = (0x0010);
	uint32_t val = 0;

	uint32_t system_cpu_waiti_delay_num = 0x0;
	int system_cpu_waiti_delay_num_x = 4;
	int system_cpu_waiti_delay_num_y = 7;

	bool system_cpu_wait_mode_force_on = 1;
	int system_cpu_wait_mode_force_on_pos = 3;

	bool system_pll_freq_sel = 1;
	int system_pll_freq_sel_pos = 2;

	uint32_t system_cpuperiod_sel = 0;
	int system_cpuperiod_sel_x = 0;
	int system_cpuperiod_sel_y = 1;
};

struct system_perip_clk_en0_reg{
	uint32_t base = (0x0018);
	uint32_t val = 0;

	bool system_adc2_arb_clk_en = 1;
	int system_adc2_arb_clk_en_pos = 30;

	bool system_systimer_clk_en = 1;
	int system_systimer_clk_en_pos = 29;

	bool system_apb_saradc_clk_en = 0;
	int system_apb_saradc_clk_en_pos = 28;

	bool system_uart_mem_clk_en = 1;
	int system_uart_mem_clk_en_pos = 24;

	bool system_usb_clk_en = 1;
	int system_usb_clk_en_pos = 23;

	bool system_i2s1_clk_en = 0;
	int system_i2s1_clk_en_pos = 21;

	bool system_pwm1_clk_en = 0;
	int system_pwm1_clk_en_pos = 20;

	bool system_can_clk_en = 0;
	int system_can_clk_en_pos = 19;

	bool system_i2c_ext1_clk_en = 0;
	int system_i2c_ext1_clk_en_pos = 18;

	bool system_pwm0_clk_en = 0;
	int system_pwm0_clk_en_pos = 17;

	bool system_spi3_clk_en = 1;
	int system_spi3_clk_en_pos = 16;

	bool system_timergroup1_clk_en = 1;
	int system_timergroup1_clk_en_pos = 15;

	bool system_timergroup_clk_en = 1;
	int system_timergroup_clk_en_pos = 13;

	bool system_ledc_clk_en = 0;
	int system_ledc_clk_en_pos = 11;

	bool system_pcnt_clk_en = 0;
	int system_pcnt_clk_en_pos = 10;

	bool system_rmt_clk_en = 0;
	int system_rmt_clk_en_pos = 9;

	bool system_uhci0_clk_en = 0;
	int system_uhci0_clk_en_pos = 8;

	bool system_i2c_ext0_clk_en = 0;
	int system_i2c_ext0_clk_en_pos = 7;

	bool system_spi2_clk_en = 1;
	int system_spi2_clk_en_pos = 6;

	bool system_uart1_clk_en = 1;
	int system_uart1_clk_en_pos = 5;

	bool system_i2s0_clk_en = 0;
	int system_i2s0_clk_en_pos = 4;

	bool system_uart_clk_en = 1;
	int system_uart_clk_en_pos = 2;

	bool system_spi01_clk_en = 1;
	int system_spi01_clk_en_pos = 1;
};

struct system_perip_clk_en1_reg{
	uint32_t base = (0x001c);
	uint32_t val = 0;

	bool system_usb_device_clk_en = 1;
	int system_usb_device_clk_en_pos = 10;

	bool system_uart2_clk_en = 1;
	int system_uart2_clk_en_pos = 9;

	bool system_lcd_cam_clk_en = 0;
	int system_lcd_cam_clk_en_pos = 8;

	bool system_sdio_host_clk_en = 0;
	int system_sdio_host_clk_en_pos = 7;

	bool system_dma_clk_en = 0;
	int system_dma_clk_en_pos = 6;

	bool system_crypto_hmac_clk_en = 0;
	int system_crypto_hmac_clk_en_pos = 5;

	bool system_crypto_ds_clk_en = 0;
	int system_crypto_ds_clk_en_pos = 4;

	bool system_crypto_rsa_clk_en = 0;
	int system_crypto_rsa_clk_en_pos = 3;

	bool system_crypto_sha_clk_en = 0;
	int system_crypto_sha_clk_en_pos = 2;

	bool system_crypto_aes_clk_en = 0;
	int system_crypto_aes_clk_en_pos = 1;

	bool system_peri_backup_clk_en = 0;
	int system_peri_backup_clk_en_pos = 0;
};

struct system_perip_rst_en0_reg{
	uint32_t base = (0x0020);
	uint32_t val = 0;

	bool system_adc2_arb_rst = 0;
	int system_adc2_arb_rst_pos = 30;

	bool system_systimer_rst = 0;
	int system_systimer_rst_pos = 29;

	bool system_apb_saradc_rst = 0;
	int system_apb_saradc_rst_pos = 28;

	bool system_uart_mem_rst = 0;
	int system_uart_mem_rst_pos = 24;

	bool system_usb_rst = 0;
	int system_usb_rst_pos = 23;

	bool system_i2s1_rst = 0;
	int system_i2s1_rst_pos = 21;

	bool system_pwm1_rst = 0;
	int system_pwm1_rst_pos = 20;

	bool system_can_rst = 0;
	int system_can_rst_pos = 19;

	bool system_i2c_ext1_rst = 0;
	int system_i2c_ext1_rst_pos = 18;

	bool system_pwm0_rst = 0;
	int system_pwm0_rst_pos = 17;

	bool system_spi3_rst = 0;
	int system_spi3_rst_pos = 16;

	bool system_timergroup1_rst = 0;
	int system_timergroup1_rst_pos = 15;

	bool system_timergroup_rst = 0;
	int system_timergroup_rst_pos = 13;

	bool system_ledc_rst = 0;
	int system_ledc_rst_pos = 11;

	bool system_pcnt_rst = 0;
	int system_pcnt_rst_pos = 10;

	bool system_rmt_rst = 0;
	int system_rmt_rst_pos = 9;

	bool system_uhci0_rst = 0;
	int system_uhci0_rst_pos = 8;

	bool system_i2c_ext0_rst = 0;
	int system_i2c_ext0_rst_pos = 7;

	bool system_spi2_rst = 0;
	int system_spi2_rst_pos = 6;

	bool system_uart1_rst = 0;
	int system_uart1_rst_pos = 5;

	bool system_i2s0_rst = 0;
	int system_i2s0_rst_pos = 4;

	bool system_uart_rst = 0;
	int system_uart_rst_pos = 2;

	bool system_spi01_rst = 0;
	int system_spi01_rst_pos = 1;
};

struct system_perip_rst_en1_reg{
	uint32_t base = (0x0024);
	uint32_t val = 0;

	bool system_usb_device_rst = 0;
	int system_usb_device_rst_pos = 10;

	bool system_uart2_rst = 0;
	int system_uart2_rst_pos = 9;

	bool system_lcd_cam_rst = 1;
	int system_lcd_cam_rst_pos = 8;

	bool system_sdio_host_rst = 1;
	int system_sdio_host_rst_pos = 7;

	bool system_dma_rst = 1;
	int system_dma_rst_pos = 6;

	bool system_crypto_hmac_rst = 1;
	int system_crypto_hmac_rst_pos = 5;

	bool system_crypto_ds_rst = 1;
	int system_crypto_ds_rst_pos = 4;

	bool system_crypto_rsa_rst = 1;
	int system_crypto_rsa_rst_pos = 3;

	bool system_crypto_sha_rst = 1;
	int system_crypto_sha_rst_pos = 2;

	bool system_crypto_aes_rst = 1;
	int system_crypto_aes_rst_pos = 1;

	bool system_peri_backup_rst = 0;
	int system_peri_backup_rst_pos = 0;
};

struct system_bt_lpck_div_frac_reg{
	uint32_t base = (0x002c);
	uint32_t val = 0;

	bool system_lpclk_rtc_en = 0;
	int system_lpclk_rtc_en_pos = 28;

	bool system_lpclk_sel_xtal32k = 0;
	int system_lpclk_sel_xtal32k_pos = 27;

	bool system_lpclk_sel_xtal = 0;
	int system_lpclk_sel_xtal_pos = 26;

	bool system_lpclk_sel_8m = 1;
	int system_lpclk_sel_8m_pos = 25;

	bool system_lpclk_sel_rtc_slow = 0;
	int system_lpclk_sel_rtc_slow_pos = 24;
};

struct system_cpu_intr_from_cpu_0_reg{
	uint32_t base = (0x0030);
	uint32_t val = 0;

	bool system_cpu_intr_from_cpu_0 = 0;
	int system_cpu_intr_from_cpu_0_pos = 0;
};

struct system_cpu_intr_from_cpu_1_reg{
	uint32_t base = (0x0034);
	uint32_t val = 0;

	bool system_cpu_intr_from_cpu_1 = 0;
	int system_cpu_intr_from_cpu_1_pos = 0;
};

struct system_cpu_intr_from_cpu_2_reg{
	uint32_t base = (0x0038);
	uint32_t val = 0;

	bool system_cpu_intr_from_cpu_2 = 0;
	int system_cpu_intr_from_cpu_2_pos = 0;
};

struct system_cpu_intr_from_cpu_3_reg{
	uint32_t base = (0x003c);
	uint32_t val = 0;

	bool system_cpu_intr_from_cpu_3 = 0;
	int system_cpu_intr_from_cpu_3_pos = 0;
};

struct system_rsa_pd_ctrl_reg{
	uint32_t base = (0x0040);
	uint32_t val = 0;

	bool system_rsa_mem_force_pd = 0;
	int system_rsa_mem_force_pd_pos = 2;

	bool system_rsa_mem_force_pu = 0;
	int system_rsa_mem_force_pu_pos = 1;

	bool system_rsa_mem_pd = 1;
	int system_rsa_mem_pd_pos = 0;
};

struct system_edma_ctrl_reg{
	uint32_t base = (0x0044);
	uint32_t val = 0;

	bool system_edma_reset = 0;
	int system_edma_reset_pos = 1;

	bool system_edma_clk_on = 1;
	int system_edma_clk_on_pos = 0;
};

struct system_cache_control_reg{
	uint32_t base = (0x0048);
	uint32_t val = 0;

	bool system_dcache_reset = 0;
	int system_dcache_reset_pos = 3;

	bool system_dcache_clk_on = 1;
	int system_dcache_clk_on_pos = 2;

	bool system_icache_reset = 0;
	int system_icache_reset_pos = 1;

	bool system_icache_clk_on = 1;
	int system_icache_clk_on_pos = 0;
};

struct system_external_device_encrypt_decrypt_control_reg{
	uint32_t base = (0x004c);
	uint32_t val = 0;

	bool system_enable_download_manual_encrypt = 0;
	int system_enable_download_manual_encrypt_pos = 3;

	bool system_enable_download_g0cb_decrypt = 0;
	int system_enable_download_g0cb_decrypt_pos = 2;

	bool system_enable_download_db_encrypt = 0;
	int system_enable_download_db_encrypt_pos = 1;

	bool system_enable_spi_manual_encrypt = 0;
	int system_enable_spi_manual_encrypt_pos = 0;
};

struct system_rtc_fastmem_config_reg{
	uint32_t base = (0x0050);
	uint32_t val = 0;

	bool system_rtc_mem_crc_finish = 0;
	int system_rtc_mem_crc_finish_pos = 31;

	uint32_t system_rtc_mem_crc_len = 0x7ff;
	int system_rtc_mem_crc_len_x = 20;
	int system_rtc_mem_crc_len_y = 30;

	uint32_t system_rtc_mem_crc_addr = 0x0;
	int system_rtc_mem_crc_addr_x = 9;
	int system_rtc_mem_crc_addr_y = 19;

	bool system_rtc_mem_crc_start = 0;
	int system_rtc_mem_crc_start_pos = 8;
};

struct system_rtc_fastmem_crc_reg{
	uint32_t base = (0x0054);
	uint32_t val = 0;

	uint32_t system_rtc_mem_crc_res = 0;
	int system_rtc_mem_crc_res_x = 0;
	int system_rtc_mem_crc_res_y = 31;
};

struct system_clock_gate_reg{
	uint32_t base = (0x005c);
	uint32_t val = 0;

	bool system_clk_en = 1;
	int system_clk_en_pos = 0;

};

struct system_sysclk_conf_reg{
	uint32_t base = (0x0060);
	uint32_t val = 0;

	uint32_t system_clk_xtal_freq = 0;
	int system_clk_xtal_freq_x = 12;
	int system_clk_xtal_freq_y = 18;

	uint32_t system_soc_clk_sel = 0;
	int system_soc_clk_sel_x = 10;
	int system_soc_clk_sel_y = 11;

	uint32_t system_pre_div_cnt = 0x1;
	int system_pre_div_cnt_x = 0;
	int system_pre_div_cnt_y = 9;
};

struct system_date_reg{
	uint32_t base = (0x0ffc);
	uint32_t val = 0;

	uint32_t system_date = 0x2101220;
	int system_date_x = 0;
	int system_date_y = 27;
};

struct apb_ctrl_clkgate_force_on_reg{
	uint32_t base = (0x00a8);
	uint32_t val = 0;

	uint32_t apb_ctrl_sram_clkgate_force_on = 0x7ff;
	int apb_ctrl_sram_clkgate_force_on_x = 3;
	int apb_ctrl_sram_clkgate_force_on_y = 13;

	uint32_t apb_ctrl_rom_clkgate_force_on = 0x7;
	int apb_ctrl_rom_clkgate_force_on_x = 0;
	int apb_ctrl_rom_clkgate_force_on_y = 2;
};

struct apb_ctrl_mem_power_down_reg{
	uint32_t base = (0x00ac);
	uint32_t val = 0;

	uint32_t apb_ctrl_sram_power_down = 0;
	int apb_ctrl_sram_power_down_x = 3;
	int apb_ctrl_sram_power_down_y = 13;

	uint32_t apb_ctrl_rom_power_down = 0;
	int apb_ctrl_rom_power_down_x = 0;
	int apb_ctrl_rom_power_down_y = 2;
};

struct apb_ctrl_mem_power_up_reg{
	uint32_t base = (0x00b0);
	uint32_t val = 0;

	uint32_t apb_ctrl_sram_power_up = 0x7ff;
	int apb_ctrl_sram_power_up_x = 3;
	int apb_ctrl_sram_power_up_y = 13;

	uint32_t apb_ctrl_rom_power_up = 0x7;
	int apb_ctrl_rom_power_up_x = 0;
	int apb_ctrl_rom_power_up_y = 2;

};

