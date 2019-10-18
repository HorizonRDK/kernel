#ifndef PKA_CLASS_H_
#define PKA_CLASS_H_

#include <linux/device.h>
#include "pka_ioctl.h"

/**
 * struct pka_class_ops - core operations for PKA drivers.
 * @pka_setparam: Loads a named parameter into the engine.
 * @pka_getparam: Reads a named parameter from the engine.
 * @pka_testf:    Test a named flag from the previous operation.
 * @pka_setf:     Manipulate flags for the next operation.
 * @pka_call:     Begin executing an operation by name.
 * @pka_abort:    Abort a currently running operation.
 * @pka_wait:     Wait for an operation to finish and return its result.
 * @pka_ioctl:    Handle an unknown ioctl to allow extensions.
 */
struct pka_class_ops {
   long (*pka_setparam)(struct device *dev, struct pka_param *param);
   long (*pka_getparam)(struct device *dev, struct pka_param *param);
   long (*pka_testf)(struct device *dev, struct pka_flag *flag);
   long (*pka_setf)(struct device *dev, struct pka_flag *flag);
   long (*pka_call)(struct device *dev, struct pka_param *param);
   long (*pka_abort)(struct device *dev);
   long (*pka_wait)(struct device *dev);

   long (*pka_ioctl)(struct device *dev, unsigned int cmd, unsigned long arg);
};

struct device *pka_chrdev_register(struct device *dev, const struct pka_class_ops *ops);
void pka_chrdev_unregister(struct device *dev);

int pka_class_init(void);
void pka_class_exit(void);

#endif
