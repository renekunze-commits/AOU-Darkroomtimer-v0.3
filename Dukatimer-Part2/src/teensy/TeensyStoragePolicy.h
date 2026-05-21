#pragma once

#include <SdFat.h>

#if !defined(SDFAT_FILE_TYPE) || SDFAT_FILE_TYPE != 3
#error Dukatimer-Part2 teensy storage policy expects SdFat with SDFAT_FILE_TYPE == 3.
#endif

namespace dukatimer {

using TeensyStorageVolume = SdFs;
using TeensyStorageFile = FsFile;

inline bool beginTeensyStorageVolume(TeensyStorageVolume& volume) {
	// Der Teensy-Storagepfad bleibt bewusst auf dem nativen SdFat-SDIO-Zugang.
	// So vermeiden wir gemischte Arduino-FS-/SD-Wrapper im Safety- und VFS-Pfad.
	return volume.begin(SdioConfig(FIFO_SDIO));
}

}  // namespace dukatimer