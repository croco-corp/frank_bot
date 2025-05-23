import numpy as np
import hailo_platform as hpf

# 1. Load the compiled HEF file
hef = hpf.HEF("/usr/share/hailo-models/yolov8s_h8.hef")

# 2. Open a virtual device (VDevice) and configure the network
with hpf.VDevice() as vdevice:
    # On RPi 5 AI HAT, the Hailo‐8 is exposed over PCIe, so use HailoStreamInterface.PCIe
    cfg_params = hpf.ConfigureParams.create_from_hef(
        hef,
        interface=hpf.HailoStreamInterface.PCIe
    )
    network_groups = vdevice.configure(hef, cfg_params)
    # network_groups is a tuple/list; usually you take the first group:
    network_group = network_groups[0]

    # 3. Create input/output parameters for that network
    ng_params = network_group.create_params()

    # From here you would allocate I/O buffers (using ng_params), populate them,
    # and call network_group.run(), etc.
    # (See HailoRT examples for full I/O–inference loop.)
