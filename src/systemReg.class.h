#include "./systemReg.structs.h"

class ES3SystemRegister{
	private:

	public:
		ES3SystemRegister();
		~ES3SystemRegister();
		struct system_core_1_control_0_reg system_core_1_control_0;
		struct system_core_1_control_1_reg system_core_1_control_1;
		struct system_cpu_per_conf_reg system_cpu_per_conf;
		struct system_perip_clk_en0_reg system_perip_clk_en0;
		struct system_perip_clk_en1_reg system_perip_clk_en1;
		struct system_perip_rst_en0_reg system_perip_rst_en0;
		struct system_perip_rst_en1_reg system_perip_rst_en1;
		struct system_bt_lpck_div_frac_reg system_bt_lpck_div_frac;
		struct system_cpu_intr_from_cpu_0_reg system_cpu_intr_from_cpu_0;
		struct system_cpu_intr_from_cpu_1_reg system_cpu_intr_from_cpu_1;
		struct system_cpu_intr_from_cpu_2_reg system_cpu_intr_from_cpu_2;
		struct system_cpu_intr_from_cpu_3_reg system_cpu_intr_from_cpu_3;
		struct system_rsa_pd_ctrl_reg system_rsa_pd_ctrl;
		struct system_edma_ctrl_reg system_edma_ctrl;
		struct system_cache_control_reg system_cache_control;
		struct system_external_device_encrypt_decrypt_control_reg system_external_device_encrypt_decrypt_control;
		struct system_rtc_fastmem_config_reg system_rtc_fastmem_config;
		struct system_rtc_fastmem_crc_reg system_rtc_fastmem_crc;
		struct system_clock_gate_reg system_clock_gate;
		struct system_sysclk_conf_reg system_sysclk_conf;
		struct system_date_reg system_date;
		struct apb_ctrl_clkgate_force_on_reg apb_ctrl_clkgate_force_on;
		struct apb_ctrl_mem_power_down_reg apb_ctrl_mem_power_down;
		struct apb_ctrl_mem_power_up_reg apb_ctrl_mem_power_up;


	
};
