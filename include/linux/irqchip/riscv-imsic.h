/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2021 Western Digital Corporation or its affiliates.
 * Copyright (C) 2022 Ventana Micro Systems Inc.
 */
#ifndef __LINUX_IRQCHIP_RISCV_IMSIC_H
#define __LINUX_IRQCHIP_RISCV_IMSIC_H

#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/fwnode.h>
#include <linux/list.h>
#include <linux/spinlock.h>

#define IMSIC_MMIO_PAGE_SHIFT		12
#define IMSIC_MMIO_PAGE_SZ		BIT(IMSIC_MMIO_PAGE_SHIFT)
#define IMSIC_MMIO_PAGE_LE		0x00
#define IMSIC_MMIO_PAGE_BE		0x04

#define IMSIC_MIN_ID			63
#define IMSIC_MAX_ID			2048

#define IMSIC_EIDELIVERY		0x70

#define IMSIC_EITHRESHOLD		0x72

#define IMSIC_EIP0			0x80
#define IMSIC_EIP63			0xbf
#define IMSIC_EIPx_BITS			32

#define IMSIC_EIE0			0xc0
#define IMSIC_EIE63			0xff
#define IMSIC_EIEx_BITS			32

#define IMSIC_FIRST			IMSIC_EIDELIVERY
#define IMSIC_LAST			IMSIC_EIE63

#define IMSIC_MMIO_SETIPNUM_LE		0x00
#define IMSIC_MMIO_SETIPNUM_BE		0x04

struct imsic_local_config {
	phys_addr_t				msi_pa;
	void __iomem				*msi_va;

	/* Number of guest interrupt files per-HART */
	u32					nr_guest_files;
};

struct imsic_global_config {
	/* Device details */
	struct fwnode_handle                    *fwnode;

	/*
	 * MSI Target Address Scheme
	 *
	 * XLEN-1                                                12     0
	 * |                                                     |     |
	 * -------------------------------------------------------------
	 * |xxxxxx|Group Index|xxxxxxxxxxx|HART Index|Guest Index|  0  |
	 * -------------------------------------------------------------
	 */

	/* Bits representing Guest index, HART index, and Group index */
	u32					guest_index_bits;
	u32					hart_index_bits;
	u32					group_index_bits;
	u32					group_index_shift;

	/* Global base address matching all target MSI addresses */
	phys_addr_t				base_addr;

	/* Number of interrupt identities */
	u32					nr_ids;

	/* Number of guest interrupt identities */
	u32					nr_guest_ids;

	/* Number of guest interrupt files across all HARTs */
	u32					nr_guest_files;

	/* Per-CPU IMSIC addresses */
	struct imsic_local_config __percpu	*local;
};

#ifdef CONFIG_RISCV_IMSIC

const struct imsic_global_config *imsic_get_global_config(void);

#else

static inline const struct imsic_global_config *imsic_get_global_config(void)
{
	return NULL;
}

#endif

#if IS_ENABLED(CONFIG_ACPI) && IS_ENABLED(CONFIG_RISCV_IMSIC)
int imsic_platform_acpi_probe(struct fwnode_handle *fwnode);
struct fwnode_handle *imsic_acpi_get_fwnode(struct device *dev);
#else
static inline struct fwnode_handle *imsic_acpi_get_fwnode(struct device *dev) { return NULL; }
#endif

enum riscv_iommu_ir_cmd {
	RISCV_IOMMU_IR_INVALID_CMD,
	RISCV_IOMMU_IR_FORWARD,
	RISCV_IOMMU_IR_UPDATE_TARGET,
};

/* Flat MSI tables contain 16-byte entries and require size alignment (at least 4 KiB). */
#define RISCV_IOMMU_IR_MSI_PTE_SIZE	16

enum riscv_iommu_ir_target_type {
	RISCV_IOMMU_IR_TARGET_INVALID,
	RISCV_IOMMU_IR_TARGET_IMSIC,
	RISCV_IOMMU_IR_TARGET_MRIF,
};

struct riscv_iommu_ir_target {
	u64 gpa;
	enum riscv_iommu_ir_target_type type;
	union {
		u64 hpa;
		struct {
			u64 mrif_hpa;
			u64 notice_hpa;
			u32 notice_id;
		};
	};
};

/*
 * VM-wide MSI table state for IRQ bypass, shared by every S2 domain whose
 * devices forward the VM's interrupts. Like GICv4's its_vm, the hypervisor
 * allocates it and the IOMMU driver manages part of it.
 *
 * Before the first FORWARD, the hypervisor initializes it with
 * riscv_iommu_ir_vm_init() and sets the fields in the first group:
 *
 * @msi_addr_mask and @msi_addr_pattern describe the guest interrupt-file
 * addresses (in IMSIC page units), as in the IOMMU's DC.msi_addr_mask and
 * DC.msi_addr_pattern. @targets holds one entry per guest interrupt file, and
 * the hypervisor keeps it current: the IOMMU driver rebuilds the table from it
 * whenever forwarding starts for the VM. @msi_table is physically contiguous,
 * covers the full flat index space selected by @msi_addr_mask, including
 * unused index values, in RISCV_IOMMU_IR_MSI_PTE_SIZE entries, and is aligned
 * to its size or 4 KiB, whichever is larger. Entry encoding belongs to the
 * IOMMU driver.
 *
 * The hypervisor serializes all commands for a VM, and changes to @targets,
 * with each other. It frees the object and @msi_table only after every
 * forward has been undone and an RCU grace period has elapsed, since the
 * IOMMU driver may still look the object up under RCU when attaching a
 * device that had forwarded interrupts.
 */
struct riscv_iommu_ir_vm {
	/* Set by the hypervisor */
	u64 msi_addr_mask;
	u64 msi_addr_pattern;
	const struct riscv_iommu_ir_target *targets;
	unsigned int nr_targets;
	void *msi_table;
	size_t msi_table_size;

	/* Managed by the IOMMU driver */
	raw_spinlock_t lock;
	struct list_head s2_domains;	/* S2 domains using the table */
	struct list_head devices;	/* devices with forwarded IRQs */
	unsigned int nr_forwarded_irqs;
	unsigned int nr_mrif_entries;
	u64 required_caps;		/* IOMMU capabilities the entries need */
};

static inline void riscv_iommu_ir_vm_init(struct riscv_iommu_ir_vm *vm)
{
	raw_spin_lock_init(&vm->lock);
	INIT_LIST_HEAD(&vm->s2_domains);
	INIT_LIST_HEAD(&vm->devices);
	vm->nr_forwarded_irqs = 0;
	vm->nr_mrif_entries = 0;
	vm->required_caps = 0;
}

/*
 * Commands for irq_set_vcpu_affinity() on a device IRQ remapped by the IOMMU.
 *
 * FORWARD starts forwarding the IRQ through @vm's MSI table. The first
 * forwarded IRQ of the VM builds the table from the targets. An S2 domain uses
 * the table once a device with forwarded IRQs is attached to it, until the VM
 * stops forwarding. The device must be attached to an S2 domain, or a nested
 * domain on one, and its IOMMU must support what the table's entries need.
 * Forwarding an already forwarded IRQ again succeeds without changing
 * anything. While the device has forwarded IRQs, it can only be attached to
 * S2 domains which use no table or @vm's table, nested domains on such an S2,
 * or the blocking domain.
 *
 * UPDATE_TARGET replaces the entry for @target.gpa and may be sent through any
 * forwarded IRQ of @vm: the IOMMU driver updates the entry once and
 * invalidates it for every S2 domain using the table. It covers moves between
 * interrupt files and between basic (IMSIC) and MRIF mode. An INVALID target
 * clears the entry. A target needing an IOMMU capability that the IOMMU of a
 * device with forwarded IRQs lacks returns -EOPNOTSUPP. When both words of a
 * live MRIF entry change, the entry is briefly invalid, so the caller must
 * quiesce the MSI producers using it. The caller keeps the old target's
 * backing alive until the command returns.
 *
 * A NULL vcpu_info stops forwarding the IRQ. It also works while the device
 * is attached to a blocking domain.
 *
 * A well-formed request which is incompatible with the producer or the IOMMU
 * configuration returns -EOPNOTSUPP, allowing the caller to fall back to host
 * delivery. Invalid payloads return -EINVAL and operational errors are
 * propagated.
 */
struct riscv_iommu_ir_vcpu_info {
	enum riscv_iommu_ir_cmd cmd;
	struct riscv_iommu_ir_vm *vm;
	struct riscv_iommu_ir_target target;	/* UPDATE_TARGET */
};

#endif
