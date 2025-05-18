#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(.gnu.linkonce.this_module) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section(__versions) = {
	{ 0xb7b3abdc, "module_layout" },
	{ 0xc0494d82, "usb_deregister" },
	{ 0xc5850110, "printk" },
	{ 0x1fcad1a4, "tty_driver_kref_put" },
	{ 0xda703233, "tty_unregister_driver" },
	{ 0x1c66d8e0, "usb_register_driver" },
	{ 0xbb1c3f0a, "tty_register_driver" },
	{ 0x88dc1de8, "tty_set_operations" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0x7a1a7a37, "__tty_alloc_driver" },
	{ 0x2c45dfd3, "tty_port_register_device" },
	{ 0x9383281, "usb_get_intf" },
	{ 0x522f24ad, "usb_driver_claim_interface" },
	{ 0x9a3515a1, "_dev_info" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x831699a1, "_dev_warn" },
	{ 0xda4e153b, "device_create_file" },
	{ 0xfdbf07, "usb_alloc_urb" },
	{ 0xfa7d91a2, "usb_alloc_coherent" },
	{ 0xee43e753, "tty_port_init" },
	{ 0x977f511b, "__mutex_init" },
	{ 0xb26033d6, "usb_ifnum_to_if" },
	{ 0x6cbbfc54, "__arch_copy_to_user" },
	{ 0x5ec8c7c, "kmem_cache_alloc_trace" },
	{ 0x73dc14a, "kmalloc_caches" },
	{ 0xc6cbbc89, "capable" },
	{ 0x12a4e128, "__arch_copy_from_user" },
	{ 0x409873e3, "tty_termios_baud_rate" },
	{ 0xb4b8fad6, "tty_port_open" },
	{ 0xb5a9532d, "usb_autopm_put_interface" },
	{ 0xbffc1103, "usb_autopm_get_interface" },
	{ 0xd697e69a, "trace_hardirqs_on" },
	{ 0x712f55fe, "cpu_hwcaps" },
	{ 0xec3d2e1b, "trace_hardirqs_off" },
	{ 0xb38ad4b, "cpu_hwcap_keys" },
	{ 0x14b89635, "arm64_const_caps_ready" },
	{ 0x296695f, "refcount_warn_saturate" },
	{ 0x20744091, "tty_standard_install" },
	{ 0x4f78739, "tty_insert_flip_string_fixed_flag" },
	{ 0xe32b0594, "tty_flip_buffer_push" },
	{ 0xb8e5c06e, "__tty_insert_flip_char" },
	{ 0xffb8609f, "usb_driver_release_interface" },
	{ 0x2a982d32, "usb_free_urb" },
	{ 0x7a282974, "tty_unregister_device" },
	{ 0x8d9f3210, "tty_kref_put" },
	{ 0x12312b70, "tty_vhangup" },
	{ 0x410425e3, "tty_port_tty_get" },
	{ 0xd9e0aef8, "device_remove_file" },
	{ 0x301fa007, "_raw_spin_unlock" },
	{ 0xdbf17652, "_raw_spin_lock" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0x97919d18, "usb_kill_urb" },
	{ 0xf80be42b, "tty_port_close" },
	{ 0xa8a12b6b, "usb_autopm_get_interface_async" },
	{ 0xfda49de1, "tty_port_hangup" },
	{ 0xcdd5e916, "tty_port_tty_wakeup" },
	{ 0x37a0cba, "kfree" },
	{ 0xed387c8e, "usb_put_intf" },
	{ 0x409bcb62, "mutex_unlock" },
	{ 0x2ab7989d, "mutex_lock" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x6ebe366f, "ktime_get_mono_fast_ns" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0xdecd0b29, "__stack_chk_fail" },
	{ 0x8c8569cb, "kstrtoint" },
	{ 0x3201ba22, "tty_port_put" },
	{ 0xd8cb1993, "usb_free_coherent" },
	{ 0xe8a8a6d0, "tty_port_tty_hangup" },
	{ 0x46703e42, "usb_autopm_put_interface_async" },
	{ 0x93ad8658, "_dev_err" },
	{ 0x5ab97bd3, "usb_submit_urb" },
	{ 0x4829a47e, "memcpy" },
	{ 0x5d5c016a, "__dynamic_dev_dbg" },
	{ 0xdc018b09, "usb_control_msg" },
	{ 0xc5664491, "_raw_spin_unlock_irq" },
	{ 0x47941711, "_raw_spin_lock_irq" },
	{ 0x3812050a, "_raw_spin_unlock_irqrestore" },
	{ 0x51760917, "_raw_spin_lock_irqsave" },
	{ 0x1fdc7df2, "_mcount" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v04E2p1410d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1411d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1412d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1414d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1420d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1421d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1422d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1424d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1400d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1401d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1402d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1403d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "C57937D25E24140900CBC6E");
