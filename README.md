# Linux 7.0.0-sky1 — Custom Kernel for Radxa Orion O6

A custom Linux kernel for the **Radxa Orion O6** board, based on the CIX Sky1 SoC (ARM64).

This kernel is designed to work with [edk2-cix-unlocked](https://github.com/Neol00/edk2-cix-unlocked), an unlocked UEFI firmware for the Orion O6. The system boots ACPI-only (no Device Tree).

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

### Build the DKMS GPU driver (mali_kbase)

The mali_kbase GPU kernel module is packaged separately as a DKMS package. See the releases for pre-built packages or build it from the `cix-gpu-kmd` source.

## Pre-built Packages

Pre-built packages are available under the [Releases](../../releases) tab:

- **`.deb` packages** — for Debian-based installations
- **`.pkg.tar.zst` packages** — for Arch Linux installations with `pacman`
- **Arch Linux image** — a ready-to-use Arch Linux installation with the kernel and packages already installed, running xfce4 on Wayland

## Hardware Support

### Working

- GPU (Mali, integrated) — works under **Wayland only** (see notes below)
- USB
- NVMe / Storage
- Networking (Ethernet)
- PCIe
- Display output (DisplayPort)
- CPU frequency scaling
- Audio (HDA)

### Not Working

- **VPU** — video processing unit support is not available yet
- **Hardware cursor / overlay planes** — causes GPU firmware crash; use `WLR_NO_HARDWARE_CURSORS=1` and `WLR_SCENE_DISABLE_DIRECT_SCANOUT=1` under wlroots-based compositors

### Notes

- The integrated GPU does **not** support GLX or Vulkan. The CIX proprietary userspace blob provides EGL/GLES and OpenCL only. GLX falls back to software rendering (llvmpipe), meaning all X11 applications running under XWayland run **without hardware acceleration**. Native Wayland applications have full GPU acceleration.
- If you want to use a NVIDIA discrete GPU then first start up the installation without the GPU installed, install lightdm and a greeter, configure lightdm how you want it, then disable SDDM and enable lightdm with systemctl. This is the easiest way to get it working without issues. SDDM is configured to use wayland and will default to using the inegrated GPU which will crash nvidia-drm if a GPU is installed.

## License

This kernel is licensed under **GPLv2**. The mali_kbase driver is licensed under GPLv2 by ARM. See `license.txt` in the DKMS source for details.
