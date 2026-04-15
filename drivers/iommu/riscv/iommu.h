/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright © 2022-2024 Rivos Inc.
 * Copyright © 2023 FORTH-ICS/CARV
 *
 * Authors
 *	Tomasz Jeznach <tjeznach@rivosinc.com>
 *	Nick Kossifidis <mick@ics.forth.gr>
 */

#ifndef _RISCV_IOMMU_H_
#define _RISCV_IOMMU_H_

#include <linux/iommu.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/iopoll.h>
#include <linux/irqdomain.h>
#include <linux/sizes.h>

#include "iommu-bits.h"

struct riscv_iommu_device;
struct riscv_iommu_domain;

struct riscv_iommu_queue {
	atomic_t prod;				/* unbounded producer allocation index */
	atomic_t head;				/* unbounded shadow ring buffer consumer index */
	atomic_t tail;				/* unbounded shadow ring buffer producer index */
	raw_spinlock_t lock;			/* serialize queue publishing */
	unsigned int mask;			/* index mask, queue length - 1 */
	unsigned int irq;			/* allocated interrupt number */
	struct riscv_iommu_device *iommu;	/* iommu device handling the queue when active */
	void *base;				/* ring buffer kernel pointer */
	dma_addr_t phys;			/* ring buffer physical address */
	u16 qbr;				/* base register offset, head and tail reference */
	u16 qcr;				/* control and status register offset */
	u8 qid;					/* queue identifier, same as RISCV_IOMMU_INTR_XX */
};

struct riscv_iommu_device {
	/* iommu core interface */
	struct iommu_device iommu;

	/* iommu hardware */
	struct device *dev;

	/* hardware control register space */
	void __iomem *reg;

	/* supported and enabled hardware capabilities */
	u64 caps;
	u32 fctl;

	/* available interrupt numbers, MSI or WSI */
	unsigned int irqs[RISCV_IOMMU_INTR_COUNT];
	unsigned int irqs_count;
	unsigned int icvec;

	/* hardware queues */
	struct riscv_iommu_queue cmdq;
	struct riscv_iommu_queue fltq;

	/* device directory */
	unsigned int ddt_mode;
	dma_addr_t ddt_phys;
	u64 *ddt_root;

	/* MSI remapping */
	struct irq_domain *irqdomain;
};

struct riscv_iommu_ir_vm;

/*
 * An S2 domain's association with a VM's MSI table. The S2 uses the table
 * once a device with forwarded IRQs is attached to it, until the VM stops
 * forwarding or the S2 is freed. The table is installed in the contexts of
 * the S2's devices whose IOMMU supports MSI_FLAT.
 */
struct riscv_iommu_msi_table {
	/* Protects attachment and MSI updates of the S2's device contexts. */
	raw_spinlock_t lock;
	/* Changed under the VM lock and this lock; read under either. */
	struct riscv_iommu_ir_vm *vm;
	struct list_head vm_node;	/* on vm->s2_domains */
};

/* Private IOMMU data for managed devices, dev_iommu_priv_* */
struct riscv_iommu_info {
	struct riscv_iommu_domain *domain;
	struct riscv_iommu_dc dc;
	struct irq_domain *old_msi_parent;
	/*
	 * The VM the device's forwarded IRQs belong to, and how many there are.
	 * Changed under the VM lock, and the first forward also holds the lock of
	 * the S2 the device is attached to. Read under RCU to find the VM lock.
	 */
	struct riscv_iommu_ir_vm __rcu *vm;
	unsigned int nr_forwarded_irqs;
};

int riscv_iommu_init(struct riscv_iommu_device *iommu);
void riscv_iommu_remove(struct riscv_iommu_device *iommu);
void riscv_iommu_disable(struct riscv_iommu_device *iommu);

void riscv_iommu_msi_table_inval(struct riscv_iommu_msi_table *msi_table, unsigned long addr);
void riscv_iommu_msi_table_join(struct riscv_iommu_msi_table *msi_table,
				struct riscv_iommu_ir_vm *vm);
void riscv_iommu_msi_table_leave(struct riscv_iommu_msi_table *msi_table);

#ifdef CONFIG_RISCV_IMSIC
void riscv_iommu_ir_irq_domain_remove(struct riscv_iommu_device *iommu);
int riscv_iommu_ir_probe_device(struct riscv_iommu_device *iommu, struct device *dev,
				struct riscv_iommu_info *info);
void riscv_iommu_ir_release_device(struct device *dev, struct riscv_iommu_info *info);
#else
static inline void riscv_iommu_ir_irq_domain_remove(struct riscv_iommu_device *iommu) { }
static inline int riscv_iommu_ir_probe_device(struct riscv_iommu_device *iommu,
					      struct device *dev,
					      struct riscv_iommu_info *info)
{
	info->old_msi_parent = NULL;
	return 0;
}
static inline void riscv_iommu_ir_release_device(struct device *dev,
						 struct riscv_iommu_info *info) { }
#endif

#define riscv_iommu_readl(iommu, addr) \
	readl_relaxed((iommu)->reg + (addr))

#define riscv_iommu_readq(iommu, addr) \
	readq_relaxed((iommu)->reg + (addr))

#define riscv_iommu_writel(iommu, addr, val) \
	writel_relaxed((val), (iommu)->reg + (addr))

#define riscv_iommu_writeq(iommu, addr, val) \
	writeq_relaxed((val), (iommu)->reg + (addr))

#define riscv_iommu_readq_timeout(iommu, addr, val, cond, delay_us, timeout_us) \
	readx_poll_timeout(readq_relaxed, (iommu)->reg + (addr), val, cond, \
			   delay_us, timeout_us)

#define riscv_iommu_readl_timeout(iommu, addr, val, cond, delay_us, timeout_us) \
	readx_poll_timeout(readl_relaxed, (iommu)->reg + (addr), val, cond, \
			   delay_us, timeout_us)

#endif
