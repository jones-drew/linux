// SPDX-License-Identifier: GPL-2.0-only
/*
 * IOMMU Interrupt Remapping
 *
 * Copyright (c) 2026 Qualcomm Technologies, Inc.
 */
#include <linux/cleanup.h>
#include <linux/irqchip/riscv-imsic.h>
#include <linux/irqdomain.h>
#include <linux/msi.h>

#include "../iommu-pages.h"
#include "iommu.h"

static size_t riscv_iommu_ir_compute_msipte_idx(struct riscv_iommu_domain *domain,
						phys_addr_t msi_pa)
{
	phys_addr_t mask = domain->msi_addr_mask;
	phys_addr_t addr = msi_pa >> 12;
	size_t idx;

	if (domain->group_index_bits) {
		phys_addr_t group_mask = BIT(domain->group_index_bits) - 1;
		phys_addr_t group_shift = domain->group_index_shift - 12;
		phys_addr_t group = (addr >> group_shift) & group_mask;

		mask &= ~(group_mask << group_shift);
		idx = addr & mask;
		idx |= group << fls64(mask);
	} else {
		idx = addr & mask;
	}

	return idx;
}

static size_t riscv_iommu_ir_nr_msiptes(struct riscv_iommu_domain *domain)
{
	phys_addr_t base = domain->msi_addr_pattern << 12;
	phys_addr_t max_addr = base | (domain->msi_addr_mask << 12);
	size_t max_idx = riscv_iommu_ir_compute_msipte_idx(domain, max_addr);

	return max_idx + 1;
}

static void riscv_iommu_ir_set_pte(struct riscv_iommu_msipte *pte, u64 addr)
{
	pte->pte = FIELD_PREP(RISCV_IOMMU_MSIPTE_M, 3) |
		   riscv_iommu_phys_to_ppn(addr) |
		   FIELD_PREP(RISCV_IOMMU_MSIPTE_V, 1);
	pte->mrif_info = 0;
}

static void riscv_iommu_ir_clear_pte(struct riscv_iommu_msipte *pte)
{
	pte->pte = 0;
	pte->mrif_info = 0;
}

static void __riscv_iommu_ir_msitbl_inval(struct riscv_iommu_domain *domain,
					  bool all, phys_addr_t gpa)
{
	struct riscv_iommu_bond *bond;
	struct riscv_iommu_device *iommu, *prev;
	struct riscv_iommu_command cmd;

	riscv_iommu_cmd_inval_gvma(&cmd);
	riscv_iommu_cmd_inval_set_gscid(&cmd, domain->gscid);

	if (!all)
		riscv_iommu_cmd_inval_set_addr(&cmd, gpa);

	/* Like riscv_iommu_iotlb_inval(), synchronize with riscv_iommu_bond_link() */
	smp_mb();

	rcu_read_lock();

	prev = NULL;
	list_for_each_entry_rcu(bond, &domain->bonds, list) {
		iommu = dev_to_iommu(bond->dev);
		if (iommu == prev)
			continue;

		riscv_iommu_cmd_send(iommu, &cmd);
		prev = iommu;
	}

	prev = NULL;
	list_for_each_entry_rcu(bond, &domain->bonds, list) {
		iommu = dev_to_iommu(bond->dev);
		if (iommu == prev)
			continue;

		riscv_iommu_cmd_sync(iommu, RISCV_IOMMU_IOTINVAL_TIMEOUT);
		prev = iommu;
	}

	rcu_read_unlock();
}

static void riscv_iommu_ir_msitbl_inval(struct riscv_iommu_domain *domain,
					phys_addr_t gpa)
{
	__riscv_iommu_ir_msitbl_inval(domain, false, gpa);
}

static void riscv_iommu_ir_msitbl_inval_all(struct riscv_iommu_domain *domain)
{
	__riscv_iommu_ir_msitbl_inval(domain, true, 0);
}

struct riscv_iommu_ir_chip_data {
	u32 config;
	phys_addr_t gpa;
};

static u32 riscv_iommu_ir_irq_msitbl_config(struct irq_data *data)
{
	struct riscv_iommu_ir_chip_data *chip_data = irq_data_get_irq_chip_data(data);

	return chip_data->config;
}

static phys_addr_t riscv_iommu_ir_irq_msitbl_gpa(struct irq_data *data)
{
	struct riscv_iommu_ir_chip_data *chip_data = irq_data_get_irq_chip_data(data);

	return chip_data->gpa;
}

static void riscv_iommu_ir_irq_set_msitbl_info(struct irq_data *data,
					       u32 config, phys_addr_t gpa)
{
	struct riscv_iommu_ir_chip_data *chip_data = irq_data_get_irq_chip_data(data);

	chip_data->config = config;
	chip_data->gpa = gpa;
}

static void riscv_iommu_ir_msitbl_map(struct riscv_iommu_domain *domain,
				      struct irq_data *data,
				      size_t idx, phys_addr_t addr)
{
	struct riscv_iommu_msipte *pte;

	riscv_iommu_ir_irq_set_msitbl_info(data, domain->msitbl_config, addr);

	if (!domain->msi_root)
		return;

	if (!refcount_inc_not_zero(&domain->msi_pte_counts[idx])) {
		scoped_guard(raw_spinlock_irqsave, &domain->msi_lock) {
			if (refcount_read(&domain->msi_pte_counts[idx]) == 0) {
				pte = &domain->msi_root[idx];
				riscv_iommu_ir_set_pte(pte, addr);
				riscv_iommu_ir_msitbl_inval(domain, addr);
				refcount_set(&domain->msi_pte_counts[idx], 1);
			} else {
				refcount_inc(&domain->msi_pte_counts[idx]);
			}
		}
	}
}

static void riscv_iommu_ir_msitbl_unmap_idx(struct riscv_iommu_domain *domain,
					    size_t idx, phys_addr_t gpa)
{
	struct riscv_iommu_msipte *pte;

	if (!domain->msi_root)
		return;

	scoped_guard(raw_spinlock_irqsave, &domain->msi_lock) {
		if (refcount_dec_and_test(&domain->msi_pte_counts[idx])) {
			pte = &domain->msi_root[idx];
			riscv_iommu_ir_clear_pte(pte);
			riscv_iommu_ir_msitbl_inval(domain, gpa);
		}
	}
}

static void riscv_iommu_ir_msitbl_unmap(struct riscv_iommu_domain *domain,
					struct irq_data *data, size_t idx)
{
	phys_addr_t gpa = riscv_iommu_ir_irq_msitbl_gpa(data);
	u32 config = riscv_iommu_ir_irq_msitbl_config(data);

	riscv_iommu_ir_irq_set_msitbl_info(data, -1, 0);

	if (WARN_ON_ONCE(config != domain->msitbl_config))
		return;

	riscv_iommu_ir_msitbl_unmap_idx(domain, idx, gpa);
}

static size_t riscv_iommu_ir_get_msipte_idx_from_target(struct riscv_iommu_domain *domain,
							struct irq_data *data,
							phys_addr_t *addr)
{
	struct msi_msg msg;

	WARN_ON_ONCE(irq_chip_compose_msi_msg(data, &msg));

	*addr = ((phys_addr_t)msg.address_hi << 32) | msg.address_lo;

	return riscv_iommu_ir_compute_msipte_idx(domain, *addr);
}

static int riscv_iommu_ir_irq_set_affinity(struct irq_data *data,
					   const struct cpumask *dest, bool force)
{
	struct riscv_iommu_info *info = data->domain->host_data;
	struct riscv_iommu_domain *domain = info->domain;
	size_t old_idx, new_idx;
	phys_addr_t new_addr;
	phys_addr_t gpa;
	int ret;

	gpa = riscv_iommu_ir_irq_msitbl_gpa(data);
	old_idx = riscv_iommu_ir_compute_msipte_idx(domain, gpa);

	ret = irq_chip_set_affinity_parent(data, dest, force);
	if (ret < 0)
		return ret;

	new_idx = riscv_iommu_ir_get_msipte_idx_from_target(domain, data, &new_addr);

	if (new_idx == old_idx)
		return ret;

	riscv_iommu_ir_msitbl_map(domain, data, new_idx, new_addr);
	/*
	 * Install the new PTE before removing the old one. After
	 * irq_chip_set_affinity_parent() the device may already be
	 * targeting the new IMSIC address, so the new PTE must be
	 * present before the old slot is torn down.
	 *
	 * msitbl_map() has already overwritten chip_data->gpa with
	 * new_addr, so use the saved old gpa with msitbl_unmap_idx()
	 * directly rather than msitbl_unmap() which reads chip_data.
	 */
	riscv_iommu_ir_msitbl_unmap_idx(domain, old_idx, gpa);

	return ret;
}

static void riscv_iommu_ir_msitbl_clear(struct riscv_iommu_domain *domain)
{
	for (size_t i = 0; i < riscv_iommu_ir_nr_msiptes(domain); i++) {
		riscv_iommu_ir_clear_pte(&domain->msi_root[i]);
		refcount_set(&domain->msi_pte_counts[i], 0);
	}
}

static void riscv_iommu_ir_msiptp_update(struct riscv_iommu_domain *domain)
{
	struct pt_iommu_riscv_64_hw_info pt_info;
	struct riscv_iommu_bond *bond;
	struct riscv_iommu_dc new_dc;

	pt_iommu_riscv_64_hw_info(&domain->riscvpt, &pt_info);

	new_dc = (struct riscv_iommu_dc){
		.ta = RISCV_IOMMU_PC_TA_V,
		.iohgatp = FIELD_PREP(RISCV_IOMMU_DC_IOHGATP_MODE, pt_info.fsc_iosatp_mode) |
			   FIELD_PREP(RISCV_IOMMU_DC_IOHGATP_GSCID, domain->gscid) |
			   FIELD_PREP(RISCV_IOMMU_DC_IOHGATP_PPN, pt_info.ppn),
		.fsc = RISCV_IOMMU_FSC_BARE,
		.msiptp = virt_to_pfn(domain->msi_root) |
			  FIELD_PREP(RISCV_IOMMU_DC_MSIPTP_MODE,
				     RISCV_IOMMU_DC_MSIPTP_MODE_FLAT),
		.msi_addr_mask = domain->msi_addr_mask,
		.msi_addr_pattern = domain->msi_addr_pattern,
	};

	/* Like riscv_iommu_ir_msitbl_inval(), synchronize with riscv_iommu_bond_link() */
	smp_mb();

	rcu_read_lock();

	/*
	 * Unlike IOTINVAL commands (which are IOMMU-wide and correctly
	 * deduplicate by IOMMU), riscv_iommu_iodir_update() operates on a
	 * single device context. Every bonded device must be updated
	 * individually so each DC gets the new msi_addr_pattern/mask.
	 */
	list_for_each_entry_rcu(bond, &domain->bonds, list)
		riscv_iommu_iodir_update(dev_to_iommu(bond->dev), bond->dev, &new_dc);

	rcu_read_unlock();
}

static bool riscv_iommu_ir_vcpu_check_config(struct riscv_iommu_domain *domain,
					     struct riscv_iommu_ir_vcpu_info *vcpu_info)
{
	return domain->msi_addr_mask == vcpu_info->msi_addr_mask &&
	       domain->msi_addr_pattern == vcpu_info->msi_addr_pattern &&
	       domain->group_index_bits == vcpu_info->group_index_bits &&
	       domain->group_index_shift == vcpu_info->group_index_shift;
}

static int riscv_iommu_ir_vcpu_new_config(struct riscv_iommu_domain *domain,
					  struct irq_data *data,
					  struct riscv_iommu_ir_vcpu_info *vcpu_info)
{
	struct riscv_iommu_msipte *pte;
	size_t idx;

	riscv_iommu_ir_msitbl_clear(domain);

	domain->msi_addr_mask = vcpu_info->msi_addr_mask;
	domain->msi_addr_pattern = vcpu_info->msi_addr_pattern;
	domain->group_index_bits = vcpu_info->group_index_bits;
	domain->group_index_shift = vcpu_info->group_index_shift;
	/* Guests don't have guest-index-bits, so their stride is always 4K */
	domain->imsic_stride = SZ_4K;
	domain->msitbl_config += 1;

	idx = riscv_iommu_ir_compute_msipte_idx(domain, vcpu_info->gpa);
	pte = &domain->msi_root[idx];
	riscv_iommu_ir_irq_set_msitbl_info(data, domain->msitbl_config, vcpu_info->gpa);
	riscv_iommu_ir_set_pte(pte, vcpu_info->hpa);
	riscv_iommu_ir_msitbl_inval_all(domain);
	refcount_set(&domain->msi_pte_counts[idx], 1);

	riscv_iommu_ir_msiptp_update(domain);

	return 0;
}

static int riscv_iommu_ir_irq_set_vcpu_affinity(struct irq_data *data, void *arg)
{
	struct riscv_iommu_info *info = data->domain->host_data;
	struct riscv_iommu_domain *domain = info->domain;
	struct riscv_iommu_ir_vcpu_info *vcpu_info = arg;
	struct riscv_iommu_msipte pteval;
	struct riscv_iommu_msipte *pte;
	bool inc = false, dec = false;
	size_t old_idx, new_idx;
	phys_addr_t old_gpa;
	u32 old_config;

	if (!domain->msi_root)
		return -EOPNOTSUPP;

	old_config = riscv_iommu_ir_irq_msitbl_config(data);
	old_gpa = riscv_iommu_ir_irq_msitbl_gpa(data);
	old_idx = riscv_iommu_ir_compute_msipte_idx(domain, old_gpa);

	/* NULL vcpu_info means remove the mapping and revert to host delivery. */
	if (!vcpu_info) {
		riscv_iommu_ir_msitbl_unmap(domain, data, old_idx);
		return 0;
	}

	guard(raw_spinlock_irqsave)(&domain->msi_lock);

	if (!riscv_iommu_ir_vcpu_check_config(domain, vcpu_info))
		return riscv_iommu_ir_vcpu_new_config(domain, data, vcpu_info);

	new_idx = riscv_iommu_ir_compute_msipte_idx(domain, vcpu_info->gpa);
	riscv_iommu_ir_irq_set_msitbl_info(data, domain->msitbl_config, vcpu_info->gpa);

	pte = &domain->msi_root[new_idx];
	riscv_iommu_ir_set_pte(&pteval, vcpu_info->hpa);

	if (pteval.pte != pte->pte) {
		*pte = pteval;
		riscv_iommu_ir_msitbl_inval(domain, vcpu_info->gpa);
	}

	if (old_config != domain->msitbl_config)
		inc = true;
	else if (new_idx != old_idx)
		inc = dec = true;

	if (dec && refcount_dec_and_test(&domain->msi_pte_counts[old_idx])) {
		pte = &domain->msi_root[old_idx];
		riscv_iommu_ir_clear_pte(pte);
		riscv_iommu_ir_msitbl_inval(domain, old_gpa);
	}

	if (inc && !refcount_inc_not_zero(&domain->msi_pte_counts[new_idx]))
		refcount_set(&domain->msi_pte_counts[new_idx], 1);

	return 0;
}

static struct irq_chip riscv_iommu_ir_irq_chip = {
	.name			= "IOMMU-IR",
	.irq_ack		= irq_chip_ack_parent,
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_set_affinity	= riscv_iommu_ir_irq_set_affinity,
	.irq_set_vcpu_affinity	= riscv_iommu_ir_irq_set_vcpu_affinity,
};

static int riscv_iommu_ir_irq_domain_alloc_irqs(struct irq_domain *irqdomain,
						unsigned int irq_base, unsigned int nr_irqs,
						void *arg)
{
	struct riscv_iommu_info *info = irqdomain->host_data;
	struct riscv_iommu_domain *domain = info->domain;
	struct riscv_iommu_ir_chip_data *chip_data;
	struct irq_data *data;
	phys_addr_t addr;
	size_t idx;
	int i, ret;

	ret = irq_domain_alloc_irqs_parent(irqdomain, irq_base, nr_irqs, arg);
	if (ret)
		return ret;

	for (i = 0; i < nr_irqs; i++) {
		chip_data = kzalloc_obj(*chip_data, GFP_KERNEL_ACCOUNT);
		if (!chip_data) {
			while (--i >= 0) {
				data = irq_domain_get_irq_data(irqdomain, irq_base + i);
				kfree(data->chip_data);
			}
			irq_domain_free_irqs_parent(irqdomain, irq_base, nr_irqs);
			return -ENOMEM;
		}
		data = irq_domain_get_irq_data(irqdomain, irq_base + i);
		data->chip = &riscv_iommu_ir_irq_chip;
		data->chip_data = chip_data;
		idx = riscv_iommu_ir_get_msipte_idx_from_target(domain, data, &addr);
		riscv_iommu_ir_msitbl_map(domain, data, idx, addr);
	}

	return 0;
}

static void riscv_iommu_ir_irq_domain_free_irqs(struct irq_domain *irqdomain,
						unsigned int irq_base, unsigned int nr_irqs)
{
	struct riscv_iommu_info *info = irqdomain->host_data;
	struct riscv_iommu_domain *domain = info->domain;
	struct irq_data *data;
	phys_addr_t gpa;
	u32 config;
	size_t idx;
	int i;

	for (i = 0; i < nr_irqs; i++) {
		data = irq_domain_get_irq_data(irqdomain, irq_base + i);
		config = riscv_iommu_ir_irq_msitbl_config(data);
		/*
		 * Only irqs with matching config versions need to be unmapped here
		 * since config changes will unmap everything and irq-set-vcpu-affinity
		 * irq deletions unmap at deletion time. An example of stale indices that
		 * don't need to be unmapped are those of irqs allocated by VFIO that a
		 * guest driver never used. The config change made for the guest will have
		 * already unmapped those, though, so there's no need to unmap them here.
		 */
		if (config == domain->msitbl_config) {
			gpa = riscv_iommu_ir_irq_msitbl_gpa(data);
			idx = riscv_iommu_ir_compute_msipte_idx(domain, gpa);
			riscv_iommu_ir_msitbl_unmap(domain, data, idx);
		}
		kfree(data->chip_data);
	}

	irq_domain_free_irqs_parent(irqdomain, irq_base, nr_irqs);
}

static const struct irq_domain_ops riscv_iommu_ir_irq_domain_ops = {
	.alloc = riscv_iommu_ir_irq_domain_alloc_irqs,
	.free = riscv_iommu_ir_irq_domain_free_irqs,
};

static const struct msi_parent_ops riscv_iommu_ir_msi_parent_ops = {
	.prefix			= "IR-",
	.supported_flags	= MSI_GENERIC_FLAGS_MASK |
				  MSI_FLAG_PCI_MSIX,
	.required_flags		= MSI_FLAG_USE_DEF_DOM_OPS |
				  MSI_FLAG_USE_DEF_CHIP_OPS |
				  MSI_FLAG_PCI_MSI_MASK_PARENT,
	.chip_flags		= MSI_CHIP_FLAG_SET_ACK,
	.init_dev_msi_info	= msi_parent_init_dev_msi_info,
};

struct irq_domain *riscv_iommu_ir_irq_domain_create(struct riscv_iommu_device *iommu,
						    struct device *dev,
						    struct riscv_iommu_info *info)
{
	struct irq_domain *irqparent = dev_get_msi_domain(dev);
	struct irq_domain *irqdomain;
	struct fwnode_handle *fn;
	char *fwname __free(kfree) = NULL;

	fwname = kasprintf(GFP_KERNEL, "IOMMU-IR-%s", dev_name(dev));
	if (!fwname)
		return NULL;

	fn = irq_domain_alloc_named_fwnode(fwname);
	if (!fn) {
		dev_err(dev, "Couldn't allocate fwnode\n");
		return NULL;
	}

	irqdomain = irq_domain_create_hierarchy(irqparent, 0, 0, fn,
						&riscv_iommu_ir_irq_domain_ops,
						info);
	if (!irqdomain) {
		dev_err(dev, "Failed to create IOMMU irq domain\n");
		irq_domain_free_fwnode(fn);
		return NULL;
	}

	/*
	 * The RISC-V IOMMU doesn't validate MSI data, so
	 * IRQ_DOMAIN_FLAG_ISOLATED_MSI cannot be set unconditionally.
	 * IRQ_DOMAIN_FLAG_ISOLATED_MSI is only set for KVM device
	 * assignment via the iommufd cdev path where it can be known that
	 * the assignment is for a guest. When assigning devices to guests
	 * the MSIs are isolated, since MSI remapping guarantees MSIs are
	 * only sent to guest-exclusive interrupt files. This is why the
	 * RISC-V IOMMU claims the IOMMU_CAP_VIRT_MSI_ISOLATION capability.
	 * For KVM device assignment via the legacy VFIO container path,
	 * allow_unsafe_interrupts is required, but safe to use. Bare
	 * userspace VFIO is not safe.
	 */
	irqdomain->flags |= IRQ_DOMAIN_FLAG_MSI_PARENT;
	irqdomain->msi_parent_ops = &riscv_iommu_ir_msi_parent_ops;
	irq_domain_update_bus_token(irqdomain, DOMAIN_BUS_MSI_REMAP);

	dev_set_msi_domain(dev, irqdomain);

	return irqdomain;
}

void riscv_iommu_ir_irq_domain_remove(struct device *dev, struct riscv_iommu_info *info)
{
	struct irq_domain *parent;
	struct fwnode_handle *fn;

	if (!info->irqdomain)
		return;

	parent = info->irqdomain->parent;
	fn = info->irqdomain->fwnode;
	irq_domain_remove(info->irqdomain);
	info->irqdomain = NULL;
	irq_domain_free_fwnode(fn);
	dev_set_msi_domain(dev, parent);
}

static void riscv_iommu_ir_free_msi_table(struct riscv_iommu_domain *domain)
{
	iommu_free_pages(domain->msi_root);
	domain->msi_root = NULL;
	kfree(domain->msi_pte_counts);
}

static int riscv_ir_set_imsic_global_config(struct riscv_iommu_device *iommu,
					    struct riscv_iommu_domain *domain)
{
	const struct imsic_global_config *imsic_global;
	size_t nr_ptes;
	u64 mask = 0;

	imsic_global = imsic_get_global_config();

	mask |= (BIT(imsic_global->group_index_bits) - 1) << (imsic_global->group_index_shift - 12);
	mask |= BIT(imsic_global->hart_index_bits + imsic_global->guest_index_bits) - 1;
	domain->msi_addr_mask = mask;
	domain->msi_addr_pattern = imsic_global->base_addr >> 12;
	domain->group_index_bits = imsic_global->group_index_bits;
	domain->group_index_shift = imsic_global->group_index_shift;
	domain->imsic_stride = BIT(imsic_global->guest_index_bits + 12);

	nr_ptes = riscv_iommu_ir_nr_msiptes(domain);
	domain->msi_root = iommu_alloc_pages_node_sz(NUMA_NO_NODE, GFP_KERNEL_ACCOUNT,
						     nr_ptes * sizeof(*domain->msi_root));
	if (!domain->msi_root) {
		domain->msi_addr_mask = 0;
		return -ENOMEM;
	}

	domain->msi_pte_counts = kcalloc(nr_ptes, sizeof(refcount_t), GFP_KERNEL_ACCOUNT);
	if (!domain->msi_pte_counts) {
		iommu_free_pages(domain->msi_root);
		domain->msi_root = NULL;
		domain->msi_addr_mask = 0;
		return -ENOMEM;
	}

	raw_spin_lock_init(&domain->msi_lock);
	domain->msitbl_config = 1;

	return 0;
}

int riscv_iommu_ir_attach_paging_domain(struct riscv_iommu_domain *domain,
					struct device *dev)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_info *info = dev_iommu_priv_get(dev);

	if (!info->irqdomain)
		return 0;

	if (domain->msi_addr_mask == 0)
		return riscv_ir_set_imsic_global_config(iommu, domain);

	return 0;
}

void riscv_iommu_ir_free_paging_domain(struct riscv_iommu_domain *domain)
{
	riscv_iommu_ir_free_msi_table(domain);
}
