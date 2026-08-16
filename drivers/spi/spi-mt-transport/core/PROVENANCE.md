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

The exception: the vendored files include standard hosted-C11 headers
(`<string.h>`, `<stdbool.h>`, `<stdint.h>`, `<stdarg.h>`) for their
STM32/host-native builds. None of these resolve under the kernel's
`-nostdinc` build -- this cross-compiler's own freestanding headers aren't
on the search path either, and the kernel provides its own equivalents
instead (`linux/string.h`, `linux/types.h` + `linux/stddef.h`,
`linux/types.h`, `linux/stdarg.h` respectively). Rather than hand-edit the
vendored files, `../Makefile` adds `core/kernel-compat/` to the include path
ahead of the vendored headers (via `-I$(srctree)/$(src)/core/kernel-compat`
-- the `$(srctree)/` prefix is required since `$(src)` alone resolves
against `$(objtree)` under Yocto's out-of-tree kernel builds) -- it contains
one shim per standard header, each `#include`-ing the kernel equivalent.
This is a build-time-only addition, not a change to any vendored file; keep
it that way on re-vendor.

To check for drift against the firmware repo:

```
diff -rq linux-imx/drivers/spi/spi-mt-transport/core/ \
  <(cd firmware && git show 9923f343acb6abbdc1a34e1f95803d313c52741c:firmware-common/spi-transport)
```

(or, more practically, re-clone the firmware repo at the pinned SHA and diff
directory-to-directory, matching `src/*.c` -> `core/*.c` and
`inc/spi_transport/*.h` -> `core/include/spi_transport/*.h`.)
