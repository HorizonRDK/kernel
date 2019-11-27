#ifndef __X2A_SIF_HW_API__
#define __X2A_SIF_HW_API__

void sif_transfer_ddr_owner(u32 __iomem *base_reg, u32 mux_out_index, u32 buf_index);
static u32 sif_get_ddr_addr(u32 __iomem *base_reg, u32 mux_out_index, u32 buf_index);
void sif_set_ddr_output(u32 __iomem *base_reg, sif_output_ddr_t* p_ddr, u32 *enbale);
static void sif_set_isp_output(u32 __iomem *base_reg, sif_output_isp_t* p_isp);

static void sif_set_ipu_output(u32 __iomem *base_reg, sif_output_ipu_t* p_ipu);
void sif_hw_config(u32 __iomem *base_reg, sif_cfg_t* c);

static void sif_disable_input_and_output(u32 __iomem *base_reg);
void sif_hw_disable(u32 __iomem *base_reg);
void sif_hw_enable(u32 __iomem *base_reg);

void sif_get_frameid_timestamps(u32 __iomem *base_reg, u32 mux, struct frame_id *info);
u32 sif_get_current_bufindex(u32 __iomem *base_reg, u32 mux);
bool sif_get_wdma_enable(u32 __iomem *base_reg, u32 mux);
void sif_set_wdma_buf_addr(u32 __iomem *base_reg, u32 mux_index, u32 number, u32 addr);
void sif_config_rdma_fmt(u32 __iomem *base_reg, u32 format, u32 width, u32 height);
void sif_set_rdma_buf_addr(u32 __iomem *base_reg, u32 index, u32 addr);
void sif_set_rdma_buf_stride(u32 __iomem *base_reg, u32 index, u32 stride);

void sif_set_rdma_trigger(u32 __iomem *base_reg, u32 enable);
void sif_set_wdma_enable(u32 __iomem *base_reg, u32 mux_index, bool enable);
void sif_set_rdma_enable(u32 __iomem *base_reg, u32 mux_index, bool enable);
void sif_disable_wdma(u32 __iomem *base_reg);

int sif_get_irq_src(u32 __iomem *base_reg, struct sif_irq_src *src, bool clear);
void sif_hw_dump(u32 __iomem *base_reg);
u32 sif_get_frame_intr(void __iomem *base_reg);
void sif_set_isp_performance(u32 __iomem *base_reg, u8 value);

#endif
