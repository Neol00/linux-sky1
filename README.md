# Linux 7.0.0-sky1 — Custom Kernel for Radxa Orion O6

A custom Linux kernel for the **Radxa Orion O6** board, based on the CIX Sky1 SoC (ARM64).

This kernel is designed to work with [edk2-cix-unlocked](https://github.com/Neol00/edk2-cix-unlocked), an unlocked UEFI firmware for the Orion O6. 
This kernel boots with ACPI-only currently (no Device Tree support).

## Building

### Apply the default config

```bash
make defconfig sky1_defconfig
```

The defconfig is located at `arch/arm64/configs/sky1_defconfig`.

### Build the kernel

```bash
make -j$(nproc) all
```

### Build the DKMS drivers

The mali_kbase GPU, aipu NPU, VPU kernel modules is packaged separately as a DKMS package. See the releases for pre-built packages
The installed packages will place dkms src code under /usr/src/ and can be built with for example:

```bash
sudo dkms build -m cix-gpu-kmd/1.0.0 -k 7.0.0-sky1
```
```bash
sudo dkms install -m cix-gpu-kmd/1.0.0 -k 7.0.0-sky1
```

## Pre-built Packages

Pre-built packages are available under the [Releases](../../releases) tab:

- **`.pkg.tar.zst` packages** — for Arch Linux installations with `pacman`
- **Arch Linux image** — a ready-to-use Arch Linux installation with the kernel and packages already installed, running xfce4 on Wayland can be found here: https://pixeldrain.com/u/2MLsD8ND

## Hardware Support

### Working

- GPU (Mali, integrated) — works under Wayland
- USB
- NVMe / Storage
- Networking (Ethernet)
- PCIe
- Display output (DisplayPort)
- CPU frequency scaling
- Audio (HDA)

### Not Working

- **Hardware cursor / overlay planes** — causes GPU firmware crash; use `WLR_NO_HARDWARE_CURSORS=1` and `WLR_SCENE_DISABLE_DIRECT_SCANOUT=1` under wlroots-based compositors

### Notes

- If you want to use a NVIDIA discrete GPU then first start up the installation without the GPU installed, install lightdm and a greeter, configure lightdm how you want it, then disable SDDM and enable lightdm with systemctl. This is the easiest way to get it working without issues. SDDM is configured to use wayland and will default to using the inegrated GPU which will crash nvidia-drm if a GPU is installed.
