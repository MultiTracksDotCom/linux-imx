# Provenance

Vendored verbatim from `MultiTracksDotCom/firmware` @
`9923f343acb6abbdc1a34e1f95803d313c52741c`, path
`firmware-common/spi-transport/{src,inc/spi_transport}`.

Files vendored (portable core only — the `platform/host` and `platform/stm32`
adapter subdirectories from the source tree are NOT vendored; this port
provides its own `platform/linux-kernel`-equivalent adapter as sibling files
one directory up, see `spi_transport_os_linux.c`/`spi_transport_hw_linux.c`):

- `spi_transport.c`
- `spi_transport_channel.c`
- `spi_transport_crc16.c`
- `spi_transport_frame.c`
- `spi_transport_hw.c`
- `include/spi_transport/spi_transport.h`
- `include/spi_transport/spi_transport_channel.h`
- `include/spi_transport/spi_transport_frame.h`
- `include/spi_transport/spi_transport_hw.h`
- `include/spi_transport/spi_transport_os.h`
- `include/spi_transport/spi_transport_types.h`

Do not hand-edit these files. If a change is needed, make it in the firmware
repo's copy first, then re-vendor by re-copying and updating the commit SHA
above.

The one exception: the vendored `.c` files' `#include <string.h>` (hosted
libc, used by their STM32/host-native builds) doesn't resolve under the
kernel's freestanding build -- `<stdint.h>`/`<stdbool.h>`/`<stdarg.h>`
already work directly (GCC provides these regardless of `-nostdinc`, and
other in-tree drivers in this repo already rely on that), but `<string.h>`
is hosted-only. Rather than hand-edit the vendored files, `../Makefile`
adds `core/kernel-compat/` to the include path ahead of the vendored
headers -- it contains only a `string.h` shim that `#include
<linux/string.h>`. This is a build-time-only addition, not a change to any
vendored file; keep it that way on re-vendor.

To check for drift against the firmware repo:

```
diff -rq linux-imx/drivers/spi/spi-mt-transport/core/ \
  <(cd firmware && git show 9923f343acb6abbdc1a34e1f95803d313c52741c:firmware-common/spi-transport)
```

(or, more practically, re-clone the firmware repo at the pinned SHA and diff
directory-to-directory, matching `src/*.c` -> `core/*.c` and
`inc/spi_transport/*.h` -> `core/include/spi_transport/*.h`.)
