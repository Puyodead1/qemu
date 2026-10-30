/* IGS E2100 arcade platform machine. */

#include "qemu/osdep.h"
#include "hw/core/boards.h"
#include "hw/i386/pc.h"
#include "hw/pci/pci.h"
#include "qemu/module.h"

#define TYPE_IGS_E2100_MACHINE MACHINE_TYPE_NAME("igs-e2100")
#define TYPE_IGS_E2100_FPGA "igs-e2100-fpga"

static void (*igs_e2100_parent_init)(MachineState *machine);

static void igs_e2100_machine_init(MachineState *machine)
{
    PCMachineState *pcms;

    igs_e2100_parent_init(machine);

    pcms = PC_MACHINE(machine);
    pci_create_simple(pcms->pcibus, PCI_DEVFN(5, 0),
                      TYPE_IGS_E2100_FPGA);
}

static void igs_e2100_machine_class_init(ObjectClass *oc,
                                         const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    igs_e2100_parent_init = mc->init;
    mc->init = igs_e2100_machine_init;
    mc->desc = "IGS E2100 arcade platform (Harpoon/Fish Hunter)";
    mc->default_display = "std";
}

static const TypeInfo igs_e2100_machine_info = {
    .name = TYPE_IGS_E2100_MACHINE,
    .parent = MACHINE_TYPE_NAME("pc-i440fx-5.2"),
    .class_init = igs_e2100_machine_class_init,
};

static void igs_e2100_machine_register_types(void)
{
    type_register_static(&igs_e2100_machine_info);
}

type_init(igs_e2100_machine_register_types)
