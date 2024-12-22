#include "./systemReg.structs.h"

#define ES3_SYSREG_BASE 0x600C0000
#define ES3_SYSREG_READ(reg) (*(volatile uint32_t *)(ES3_SYSREG_BASE+reg))
#define ES3_SYSREG_WRITE(reg, val) ((*(volatile uint32_t *)(ES3_SYSREG_BASE+reg)) = (val))

class ES3SystemRegister{
	private:
		void regWrite(uint32_t reg, uint32_t val);
		uint32_t regRead(uint32_t reg);
		bool regGet(uint32_t val, int pos);
		int regGet(uint32_t val, int x, int y);
		uint32_t regSet(uint32_t reg, bool val, int pos);
		uint32_t regSet(uint32_t reg, int val, int x, int y);
  

	public:
		ES3SystemRegister();
		~ES3SystemRegister();
		struct system_core_1_control_0_reg core_1_control_0;
		struct system_core_1_control_1_reg core_1_control_1;
		struct system_cpu_per_conf_reg cpu_per_conf;
		struct system_perip_clk_en0_reg perip_clk_en0;
		struct system_perip_clk_en1_reg perip_clk_en1;
		struct system_perip_rst_en0_reg perip_rst_en0;
		struct system_perip_rst_en1_reg perip_rst_en1;
		struct system_bt_lpck_div_frac_reg bt_lpck_div_frac;
		struct system_cpu_intr_from_cpu_0_reg cpu_intr_from_cpu_0;
		struct system_cpu_intr_from_cpu_1_reg cpu_intr_from_cpu_1;
		struct system_cpu_intr_from_cpu_2_reg cpu_intr_from_cpu_2;
		struct system_cpu_intr_from_cpu_3_reg cpu_intr_from_cpu_3;
		struct system_rsa_pd_ctrl_reg rsa_pd_ctrl;
		struct system_edma_ctrl_reg edma_ctrl;
		struct system_cache_control_reg cache_control;
		struct system_external_device_encrypt_decrypt_control_reg external_device_encrypt_decrypt_control;
		struct system_rtc_fastmem_config_reg rtc_fastmem_config;
		struct system_rtc_fastmem_crc_reg rtc_fastmem_crc;
		struct system_clock_gate_reg clock_gate;
		struct system_sysclk_conf_reg sysclk_conf;
		struct system_date_reg date;
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
		void setSystem_core_1_control_0(void);
		void setSystem_core_1_control_1(void);
		void setSystem_cpu_per_conf(void);
		void setSystem_perip_clk_en0(void);
		void setSystem_perip_clk_en1(void);
		void setSystem_perip_rst_en0(void);
		void setSystem_perip_rst_en1(void);
		void setSystem_bt_lpck_div_frac(void);
		void setSystem_cpu_intr_from_cpu_0(void);
		void setSystem_cpu_intr_from_cpu_1(void);
		void setSystem_cpu_intr_from_cpu_2(void);
		void setSystem_cpu_intr_from_cpu_3(void);
		void setSystem_rsa_pd_ctrl(void);
		void setSystem_edma_ctrl(void);
		void setSystem_cache_control(void);
		void setSystem_external_device_encrypt_decrypt_control(void);
		void setSystem_rtc_fastmem_config(void);
		void setSystem_rtc_fastmem_crc(void);
		void setSystem_clock_gate(void);
		void setSystem_sysclk_conf(void);
		void setSystem_date(void);
		void setApb_ctrl_clkgate_force_on(void);
		void setApb_ctrl_mem_power_down(void);
		void setApb_ctrl_mem_power_up(void);

	
};
