#include "./systemReg.structs.h"

#define ES3_SYSREG_BASE 0x600C0000
#define ES3_SYSREG_READ(reg) (*(volatile uint32_t *)(DBG_PERF_SPI_BASE+reg))
#define ES3_SYSREG_WRITE(reg, val) ((*(volatile uint32_t *)(DBG_PERF_SPI_BASE+reg)) = (val))

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

		/*
		 * Useful Functions
		 * */
		void getAll(void);

		/*
		 * Getters 
		 * */
		uint32_t getSystem_core_1_control_0(void);
		uint32_t getSystem_core_1_control_1(void);
		uint32_t getSystem_cpu_per_conf(void);
		uint32_t getSystem_perip_clk_en0(void);
		uint32_t getSystem_perip_clk_en1(void);
		uint32_t getSystem_perip_rst_en0(void);
		uint32_t getSystem_perip_rst_en1(void);
		uint32_t getSystem_bt_lpck_div_frac(void);
		uint32_t getSystem_cpu_intr_from_cpu_0(void);
		uint32_t getSystem_cpu_intr_from_cpu_1(void);
		uint32_t getSystem_cpu_intr_from_cpu_2(void);
		uint32_t getSystem_cpu_intr_from_cpu_3(void);
		uint32_t getSystem_rsa_pd_ctrl(void);
		uint32_t getSystem_edma_ctrl(void);
		uint32_t getSystem_cache_control(void);
		uint32_t getSystem_external_device_encrypt_decrypt_control(void);
		uint32_t getSystem_rtc_fastmem_config(void);
		uint32_t getSystem_rtc_fastmem_crc(void);
		uint32_t getSystem_clock_gate(void);
		uint32_t getSystem_sysclk_conf(void);
		uint32_t getSystem_date(void);
		uint32_t getApb_ctrl_clkgate_force_on(void);
		uint32_t getApb_ctrl_mem_power_down(void);
		uint32_t getApb_ctrl_mem_power_up(void);

		/*
		 * Setters 
		 * */
		void setSystem_core_1_control_0(uint32_t reg);
		void setSystem_core_1_control_1(uint32_t reg);
		void setSystem_cpu_per_conf(uint32_t reg);
		void setSystem_perip_clk_en0(uint32_t reg);
		void setSystem_perip_clk_en1(uint32_t reg);
		void setSystem_perip_rst_en0(uint32_t reg);
		void setSystem_perip_rst_en1(uint32_t reg);
		void setSystem_bt_lpck_div_frac(uint32_t reg);
		void setSystem_cpu_intr_from_cpu_0(uint32_t reg);
		void setSystem_cpu_intr_from_cpu_1(uint32_t reg);
		void setSystem_cpu_intr_from_cpu_2(uint32_t reg);
		void setSystem_cpu_intr_from_cpu_3(uint32_t reg);
		void setSystem_rsa_pd_ctrl(uint32_t reg);
		void setSystem_edma_ctrl(uint32_t reg);
		void setSystem_cache_control(uint32_t reg);
		void setSystem_external_device_encrypt_decrypt_control(uint32_t reg);
		void setSystem_rtc_fastmem_config(uint32_t reg);
		void setSystem_rtc_fastmem_crc(uint32_t reg);
		void setSystem_clock_gate(uint32_t reg);
		void setSystem_sysclk_conf(uint32_t reg);
		void setSystem_date(uint32_t reg);
		void setApb_ctrl_clkgate_force_on(uint32_t reg);
		void setApb_ctrl_mem_power_down(uint32_t reg);
		void setApb_ctrl_mem_power_up(uint32_t reg);

	
};
