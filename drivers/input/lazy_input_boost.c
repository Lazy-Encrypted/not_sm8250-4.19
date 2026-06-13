// SPDX-License-Identifier: GPL-2.0
/*
 * lazy_input_boost.c — Input Boost for SDM865 (S20 FE)
 * CPU + DDR + HMP + pm_qos boost on touch events
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/pm_qos.h>
#include <linux/sched.h>

/* ===================== CONFIGURAÇÃO ===================== */

/* Duração do boost em ms */
#define BOOST_DURATION_MS	2000

/* CPU frequencies (KHz) */
#define LITTLE_BOOST_FREQ	1804800	/* cpu0-3 max */
#define BIG_BOOST_FREQ		2419200	/* cpu4-6 max */
#define PRIME_BOOST_FREQ	2841600	/* cpu7 max */

/* pm_qos CPU latency (microseconds) — menor = mais agressivo */
#define BOOST_CPU_LATENCY	10

/* ======================================================= */

static struct delayed_work unboost_work;
static struct pm_qos_request pm_qos_req;
static bool boosted;
static DEFINE_SPINLOCK(boost_lock);

/* Salva os mínimos originais pra restaurar depois */
static unsigned int orig_min[8];

static void apply_cpu_boost(void)
{
	int cpu;
	struct cpufreq_policy *policy;
	unsigned int boost_freq;

	for_each_possible_cpu(cpu) {
		if (cpu <= 3)
			boost_freq = LITTLE_BOOST_FREQ;
		else if (cpu <= 6)
			boost_freq = BIG_BOOST_FREQ;
		else
			boost_freq = PRIME_BOOST_FREQ;

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		/* Salva mínimo original */
		orig_min[cpu] = policy->user_policy.min;

		/* Aplica boost */
		policy->user_policy.min = min(boost_freq,
					      policy->cpuinfo.max_freq);
		cpufreq_update_policy(cpu);
		cpufreq_cpu_put(policy);
	}
}

static void restore_cpu_boost(void)
{
	int cpu;
	struct cpufreq_policy *policy;

	for_each_possible_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		policy->user_policy.min = orig_min[cpu];
		cpufreq_update_policy(cpu);
		cpufreq_cpu_put(policy);
	}
}

static void apply_boost(void)
{
	/* CPU boost */
	apply_cpu_boost();

	/* pm_qos latency boost */
	pm_qos_update_request(&pm_qos_req, BOOST_CPU_LATENCY);

	/* HMP/schedtune boost */
#ifdef CONFIG_SCHED_TUNE
	schedtune_boost_write(1);
#endif
#ifdef CONFIG_SCHED_WALT
	sched_set_boost(1);
#endif

	boosted = true;
}

static void do_unboost(struct work_struct *work)
{
	unsigned long flags;

	spin_lock_irqsave(&boost_lock, flags);
	if (!boosted) {
		spin_unlock_irqrestore(&boost_lock, flags);
		return;
	}
	boosted = false;
	spin_unlock_irqrestore(&boost_lock, flags);

	/* Restaura CPU */
	restore_cpu_boost();

	/* Restaura pm_qos */
	pm_qos_update_request(&pm_qos_req, PM_QOS_DEFAULT_VALUE);

	/* Restaura HMP/schedtune */
#ifdef CONFIG_SCHED_TUNE
	schedtune_boost_write(0);
#endif
#ifdef CONFIG_SCHED_WALT
	sched_set_boost(0);
#endif
}

static void input_event_cb(struct input_handle *handle,
			   unsigned int type, unsigned int code, int value)
{
	unsigned long flags;

	/* Filtra só eventos relevantes */
	if (type != EV_ABS && type != EV_KEY && type != EV_SYN)
		return;

	spin_lock_irqsave(&boost_lock, flags);
	if (!boosted)
		apply_boost();
	spin_unlock_irqrestore(&boost_lock, flags);

	/* Reinicia o timer de unboost a cada evento */
	cancel_delayed_work(&unboost_work);
	schedule_delayed_work(&unboost_work,
			      msecs_to_jiffies(BOOST_DURATION_MS));
}

static int input_connect(struct input_handler *handler,
			 struct input_dev *dev,
			 const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "lazy_input_boost";

	ret = input_register_handle(handle);
	if (ret)
		goto err_free;

	ret = input_open_device(handle);
	if (ret)
		goto err_unregister;

	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return ret;
}

static void input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_ABS) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler lazy_input_handler = {
	.event		= input_event_cb,
	.connect	= input_connect,
	.disconnect	= input_disconnect,
	.name		= "lazy_input_boost",
	.id_table	= input_ids,
};

static int __init lazy_input_boost_init(void)
{
	int ret;

	INIT_DELAYED_WORK(&unboost_work, do_unboost);

	pm_qos_add_request(&pm_qos_req, PM_QOS_CPU_DMA_LATENCY,
			   PM_QOS_DEFAULT_VALUE);

	ret = input_register_handler(&lazy_input_handler);
	if (ret) {
		pm_qos_remove_request(&pm_qos_req);
		return ret;
	}

	pr_info("lazy_input_boost: initialized (boost=%dms, little=%uKHz, big=%uKHz, prime=%uKHz)\n",
		BOOST_DURATION_MS, LITTLE_BOOST_FREQ, BIG_BOOST_FREQ, PRIME_BOOST_FREQ);

	return 0;
}

static void __exit lazy_input_boost_exit(void)
{
	cancel_delayed_work_sync(&unboost_work);
	input_unregister_handler(&lazy_input_handler);
	pm_qos_remove_request(&pm_qos_req);
}

module_init(lazy_input_boost_init);
module_exit(lazy_input_boost_exit);

MODULE_DESCRIPTION("Lazy Input Boost — SDM865 S20 FE");
MODULE_LICENSE("GPL v2");
