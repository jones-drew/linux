// SPDX-License-Identifier: GPL-2.0-only
/*
 * IOMMU Interrupt Remapping
 *
 * Copyright (c) 2026 Qualcomm Technologies, Inc.
 */
#include <linux/cleanup.h>
#include <linux/irqchip/riscv-imsic.h>
#include <linux/msi.h>
#include <linux/slab.h>

#include "iommu.h"

static int riscv_iommu_ir_irq_set_affinity(struct irq_data *data,
					   const struct cpumask *mask, bool force)
{
	if (irqd_is_forwarded_to_vcpu(data))
		return -EINVAL;

	return irq_chip_set_affinity_parent(data, mask, force);
}

/* Forwarding starts for the VM: build its table. */
static int riscv_iommu_ir_vm_activate(struct riscv_iommu_ir_vm *vm)
{
	lockdep_assert_held(&vm->lock);

	return -EOPNOTSUPP;
}

static int riscv_iommu_ir_update_target(struct riscv_iommu_info *info,
					struct riscv_iommu_ir_vm *vm,
					const struct riscv_iommu_ir_target *target)
{
	lockdep_assert_held(&vm->lock);

	/* The command must come through a forwarded IRQ of the VM. */
	if (rcu_access_pointer(info->vm) != vm || WARN_ON_ONCE(!vm->nr_forwarded_irqs))
		return -EINVAL;

	return -EOPNOTSUPP;
}

/* @msi_table is the association of the S2 the device is attached to. */
static int riscv_iommu_ir_forward(struct irq_data *data, struct riscv_iommu_info *info,
				  struct riscv_iommu_ir_vm *vm,
				  struct riscv_iommu_msi_table *msi_table)
{
	int ret;

	lockdep_assert_held(&vm->lock);
	lockdep_assert_held(&msi_table->lock);

	/* A device's forwarded IRQs belong to one VM, and an S2 serves one VM. */
	if ((info->nr_forwarded_irqs && rcu_access_pointer(info->vm) != vm) ||
	    (msi_table->vm && msi_table->vm != vm))
		return -EOPNOTSUPP;

	if (!vm->nr_forwarded_irqs) {
		ret = riscv_iommu_ir_vm_activate(vm);
		if (ret)
			return ret;
	}

	if (!msi_table->vm)
		riscv_iommu_msi_table_join(msi_table, vm);

	if (!info->nr_forwarded_irqs++) {
		rcu_assign_pointer(info->vm, vm);
		list_add(&info->vm_node, &vm->devices);
	}
	vm->nr_forwarded_irqs++;
	irqd_set_forwarded_to_vcpu(data);

	return 0;
}

/* The VM stopped forwarding: every S2 stops using its table. */
static void riscv_iommu_ir_vm_deactivate(struct riscv_iommu_ir_vm *vm)
{
	struct riscv_iommu_msi_table *msi_table, *tmp;

	lockdep_assert_held(&vm->lock);

	list_for_each_entry_safe(msi_table, tmp, &vm->s2_domains, vm_node) {
		raw_spin_lock(&msi_table->lock);
		riscv_iommu_msi_table_leave(msi_table);
		raw_spin_unlock(&msi_table->lock);
	}
	vm->required_caps = 0;
	vm->nr_mrif_entries = 0;
}

/* Works whether or not the device is attached to an S2. */
static int riscv_iommu_ir_unforward(struct irq_data *data, struct riscv_iommu_info *info,
				    struct riscv_iommu_ir_vm *vm)
{
	lockdep_assert_held(&vm->lock);

	if (WARN_ON_ONCE(!info->nr_forwarded_irqs || !vm->nr_forwarded_irqs))
		return -EINVAL;

	if (!--info->nr_forwarded_irqs) {
		list_del(&info->vm_node);
		rcu_assign_pointer(info->vm, NULL);
	}
	if (!--vm->nr_forwarded_irqs)
		riscv_iommu_ir_vm_deactivate(vm);
	irqd_clr_forwarded_to_vcpu(data);

	return 0;
}

/* Lock the S2 association of the device's current domain, if it's an S2. */
static struct riscv_iommu_msi_table *riscv_iommu_ir_lock_msi_table(struct riscv_iommu_info *info)
{
	struct riscv_iommu_msi_table *msi_table;

	guard(rcu)();
	/*
	 * RCU keeps the S2 association alive, but the device may switch
	 * domains before it is locked. Recheck under the association lock.
	 */
	for (;;) {
		msi_table = riscv_iommu_msi_table_rcu(info);
		if (!msi_table)
			return NULL;

		raw_spin_lock(&msi_table->lock);
		if (msi_table == riscv_iommu_msi_table_rcu(info))
			return msi_table;
		raw_spin_unlock(&msi_table->lock);
	}
}

static int riscv_iommu_ir_irq_set_vcpu_affinity(struct irq_data *data, void *arg)
{
	struct riscv_iommu_ir_vcpu_info *vcpu_info = arg;
	struct riscv_iommu_msi_table *msi_table;
	struct riscv_iommu_ir_vm *vm;
	struct riscv_iommu_info *info;
	struct msi_desc *desc;
	struct device *dev;
	int ret;

	if (!vcpu_info && !irqd_is_forwarded_to_vcpu(data))
		return 0;

	if (vcpu_info &&
	    (!vcpu_info->vm ||
	     (vcpu_info->cmd != RISCV_IOMMU_IR_FORWARD &&
	      (vcpu_info->cmd != RISCV_IOMMU_IR_UPDATE_TARGET || !irqd_is_forwarded_to_vcpu(data)))))
		return -EINVAL;

	desc = irq_data_get_msi_desc(data);
	if (WARN_ON_ONCE(!desc))
		return -EINVAL;

	dev = msi_desc_to_dev(desc);
	info = dev_iommu_priv_get(dev);
	if (WARN_ON_ONCE(!info))
		return -EINVAL;

	if (vcpu_info) {
		vm = vcpu_info->vm;
	} else {
		/* The VM outlives the IRQ's forwarding. */
		scoped_guard(rcu)
			vm = rcu_dereference(info->vm);
		if (WARN_ON_ONCE(!vm))
			return -EINVAL;
	}

	raw_spin_lock(&vm->lock);

	if (!vcpu_info) {
		ret = riscv_iommu_ir_unforward(data, info, vm);
	} else if (vcpu_info->cmd == RISCV_IOMMU_IR_UPDATE_TARGET) {
		ret = riscv_iommu_ir_update_target(info, vm, &vcpu_info->target);
	} else if (irqd_is_forwarded_to_vcpu(data)) {
		/* Forwarding again changes nothing, even while the device is blocked. */
		ret = rcu_access_pointer(info->vm) == vm ? 0 : -EINVAL;
	} else {
		msi_table = riscv_iommu_ir_lock_msi_table(info);
		ret = msi_table ? riscv_iommu_ir_forward(data, info, vm, msi_table) : -EOPNOTSUPP;
		if (msi_table)
			raw_spin_unlock(&msi_table->lock);
	}

	raw_spin_unlock(&vm->lock);
	return ret;
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
	int ret;

	ret = irq_domain_alloc_irqs_parent(irqdomain, irq_base, nr_irqs, arg);
	if (ret)
		return ret;

	for (unsigned int i = 0; i < nr_irqs; i++) {
		ret = irq_domain_set_hwirq_and_chip(irqdomain, irq_base + i,
						    irq_base + i,
						    &riscv_iommu_ir_irq_chip, NULL);
		if (ret) {
			irq_domain_free_irqs_parent(irqdomain, irq_base, nr_irqs);
			return ret;
		}
	}

	return 0;
}

static const struct irq_domain_ops riscv_iommu_ir_irq_domain_ops = {
	.alloc			= riscv_iommu_ir_irq_domain_alloc_irqs,
	.free			= irq_domain_free_irqs_parent,
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

static struct irq_domain *riscv_iommu_ir_irq_domain_create(struct riscv_iommu_device *iommu,
							   struct irq_domain *irqparent)
{
	char *fwname __free(kfree) = NULL;
	struct irq_domain *irqdomain;
	struct fwnode_handle *fn;

	fwname = kasprintf(GFP_KERNEL, "IOMMU-IR-%s", dev_name(iommu->dev));
	if (!fwname)
		return ERR_PTR(-ENOMEM);

	fn = irq_domain_alloc_named_fwnode(fwname);
	if (!fn)
		return ERR_PTR(-ENOMEM);

	irqdomain = irq_domain_create_hierarchy(irqparent, 0, 0, fn,
						&riscv_iommu_ir_irq_domain_ops, iommu);
	if (!irqdomain) {
		irq_domain_free_fwnode(fn);
		return ERR_PTR(-ENOMEM);
	}

	/*
	 * The RISC-V IOMMU doesn't validate MSI data, so we can't set
	 * IRQ_DOMAIN_FLAG_ISOLATED_MSI. The means VFIO requires its
	 * allow_unsafe_interrupts module parameter.
	 */
	irqdomain->flags |= IRQ_DOMAIN_FLAG_MSI_PARENT;
	irqdomain->msi_parent_ops = &riscv_iommu_ir_msi_parent_ops;
	irq_domain_update_bus_token(irqdomain, DOMAIN_BUS_MSI_REMAP);

	return irqdomain;
}

void riscv_iommu_ir_irq_domain_remove(struct riscv_iommu_device *iommu)
{
	struct irq_domain *irqdomain = iommu->irqdomain;
	struct fwnode_handle *fn;

	if (!irqdomain)
		return;

	fn = irqdomain->fwnode;
	irq_domain_remove(irqdomain);
	iommu->irqdomain = NULL;
	irq_domain_free_fwnode(fn);
}

int riscv_iommu_ir_probe_device(struct riscv_iommu_device *iommu, struct device *dev,
				struct riscv_iommu_info *info)
{
	struct irq_domain *msi_parent = dev_get_msi_domain(iommu->dev);
	struct irq_domain *irqdomain;

	info->old_msi_parent = NULL;

	if (iommu->fctl & RISCV_IOMMU_FCTL_WSI)
		msi_parent = imsic_get_base_domain();
	else if (!imsic_dev_has_imsic_msi_parent(iommu->dev))
		return 0;

	if (!msi_parent || dev_get_msi_domain(dev) != msi_parent)
		return 0;

	irqdomain = iommu->irqdomain;
	if (!irqdomain) {
		irqdomain = riscv_iommu_ir_irq_domain_create(iommu, msi_parent);
		if (IS_ERR(irqdomain))
			return PTR_ERR(irqdomain);
		iommu->irqdomain = irqdomain;
	} else if (irqdomain->parent != msi_parent) {
		return 0;
	}

	if (!device_link_add(dev, iommu->dev, DL_FLAG_AUTOREMOVE_SUPPLIER))
		return -ENODEV;

	info->old_msi_parent = msi_parent;
	dev_set_msi_domain(dev, irqdomain);

	return 0;
}

void riscv_iommu_ir_release_device(struct device *dev, struct riscv_iommu_info *info)
{
	struct riscv_iommu_ir_vm *vm;
	unsigned long flags;

	/* Revoking forwarding failed; at least don't leave the VM pointing at freed info. */
	scoped_guard(rcu) {
		vm = rcu_dereference(info->vm);
		if (WARN_ON_ONCE(vm)) {
			raw_spin_lock_irqsave(&vm->lock, flags);
			if (rcu_access_pointer(info->vm) == vm)
				list_del(&info->vm_node);
			raw_spin_unlock_irqrestore(&vm->lock, flags);
		}
	}

	if (info->old_msi_parent)
		dev_set_msi_domain(dev, info->old_msi_parent);
}
