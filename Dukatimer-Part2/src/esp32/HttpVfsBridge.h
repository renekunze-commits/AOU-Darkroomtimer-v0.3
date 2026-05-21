#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <WebServer.h>

#include <DukatimerProtocol.h>

namespace dukatimer {

class TeensyLinkService;

/*
 * HttpVfsBridge
 *
 * Zweck:
 * - stellt eine kleine HTTP-Oberflaeche fuer Dateiuploads zum Teensy-VFS bereit
 * - entkoppelt WLAN/AP-Management und HTTP-Requesthandling vom seriellen Linkdienst
 * - uebersetzt Link-/VFS-Zustaende in nachvollziehbare HTTP-Statuscodes und JSON-
 *   Antworten
 */
class HttpVfsBridge {
public:
	using CooperativeTickHook = void (*)(uint32_t nowMs);

	enum class NetworkMode : uint8_t {
		Offline,
		Station,
		AccessPoint,
	};

	explicit HttpVfsBridge(TeensyLinkService& teensyLink);

	void begin(uint32_t nowMs = 0);
	void tick(uint32_t nowMs);
	void setCooperativeTickHook(CooperativeTickHook hook);

private:
	TeensyLinkService& teensyLink_;
	WebServer server_;
	CooperativeTickHook cooperativeTickHook_ = nullptr;
	bool initialized_ = false;
	bool networkReady_ = false;
	NetworkMode networkMode_ = NetworkMode::Offline;
	char networkAddress_[20] = "0.0.0.0";
	// Der gemeinsame HTTP/VFS-Stagingpuffer sammelt Upload- und Exportdaten zuerst
	// vollstaendig im ESP-RAM/PSRAM. Erst danach wird zum Teensy geschrieben oder
	// an den HTTP-Client ausgeliefert. So bleiben Produktdatenpfade transaktional
	// und auch fuer Diagnose und spaetere Validierung nachvollziehbar.
	uint8_t* uploadStagingBuffer_ = nullptr;
	size_t uploadStagingCapacity_ = 0u;
	size_t uploadStagingFill_ = 0u;
	bool uploadStagingUsesPsram_ = false;
	uint32_t lastPaperSlotRecoveryRefreshMs_ = 0u;
	// Sichtbarer Zustandscontainer des aktuell verarbeiteten HTTP-Uploads.
	struct UploadRequestState {
		bool active = false;
		bool started = false;
		bool failed = false;
		bool forwarding = false;
		int httpStatusCode = 500;
		const char* statusText = "not_ready";
		const char* detail = "request not handled";
		dukatimer::protocol::VfsStatusCode vfsStatus = dukatimer::protocol::VfsStatusCode::None;
		char path[dukatimer::protocol::kVfsMaxPathLength + 1] = {};
		uint32_t bytesWritten = 0u;
		uint32_t stagedBytes = 0u;
		uint32_t expectedLength = 0u;
		bool stagingUsesPsram = false;
	} uploadRequest_;
	struct DownloadRequestState {
		bool active = false;
		bool started = false;
		bool failed = false;
		bool serving = false;
		int httpStatusCode = 500;
		const char* statusText = "not_ready";
		const char* detail = "request not handled";
		dukatimer::protocol::VfsStatusCode vfsStatus = dukatimer::protocol::VfsStatusCode::None;
		char path[dukatimer::protocol::kVfsMaxPathLength + 1] = {};
		uint32_t bytesRead = 0u;
		uint32_t stagedBytes = 0u;
		uint32_t expectedLength = 0u;
		bool stagingUsesPsram = false;
	} downloadRequest_;
	// Die letzte Upload-Zusammenfassung bleibt sichtbar, damit ein spaeterer
	// Health-Check nicht auf einen gerade aktiven Request angewiesen ist.
	struct LastUploadSnapshot {
		bool valid = false;
		bool failed = false;
		char statusText[24] = "none";
		char detail[96] = "no upload observed yet";
		char path[dukatimer::protocol::kVfsMaxPathLength + 1] = {};
		uint32_t bytesWritten = 0u;
		uint32_t stagedBytes = 0u;
		uint32_t expectedLength = 0u;
		uint16_t vfsStatusCode = 0u;
		bool stagingUsesPsram = false;
	} lastUpload_;
	struct LastDownloadSnapshot {
		bool valid = false;
		bool failed = false;
		char statusText[24] = "none";
		char detail[96] = "no export observed yet";
		char path[dukatimer::protocol::kVfsMaxPathLength + 1] = {};
		uint32_t bytesRead = 0u;
		uint32_t stagedBytes = 0u;
		uint32_t expectedLength = 0u;
		uint16_t vfsStatusCode = 0u;
		bool stagingUsesPsram = false;
	} lastDownload_;

	void configureNetwork();
	bool tryStartStation();
	bool tryStartAccessPoint();
	void startServer();
	void registerRoutes();
	void resetUploadRequest();
	void resetDownloadRequest();
	void releaseUploadStaging();
	bool allocateUploadStaging(size_t byteCount);
	bool flushStagedUploadToTeensy();
	bool stageDownloadToBuffer(const char* path);
	void captureCompletedUploadSnapshot();
	void captureCompletedDownloadSnapshot();
	const char* currentUploadPhaseName() const;
	const char* currentDownloadPhaseName() const;
	static const char* stagingStorageName(bool stagingUsesPsram, uint32_t stagedBytes);
	void markUploadSuccess(const char* detail);
	void markUploadFailure(int httpStatusCode, dukatimer::protocol::VfsStatusCode vfsStatus,
	                 const char* statusText, const char* detail);
	void markUploadFailureFromLink(int fallbackStatusCode, const char* fallbackStatusText,
	                         const char* fallbackDetail);
	void markDownloadSuccess(const char* detail);
	void markDownloadFailure(int httpStatusCode, dukatimer::protocol::VfsStatusCode vfsStatus,
	                   const char* statusText, const char* detail);
	void markDownloadFailureFromLink(int fallbackStatusCode, const char* fallbackStatusText,
	                           const char* fallbackDetail);
	void handleRoot();
	void handleHealth();
	void handlePaperSlotsContract();
	void handlePaperSlotsRecoveryState();
	void handlePaperSlotsInspect();
	void handlePaperSlotsBackupInspect();
	void handlePaperSlotsExport();
	void handlePaperSlotsBackupExport();
	void handlePaperSlotsRestoreBackup();
	void handlePaperSlotsRecover();
	void handleUpload();
	void handleUploadRaw(const char* fixedPath = nullptr);
	void handleNotFound();
	void runCooperativeWaitWork(uint32_t nowMs);
	bool waitForDownloadChunk(uint32_t timeoutMs);
	bool waitForUploadReady(uint32_t timeoutMs);
	bool waitForUploadCompletion(uint32_t timeoutMs);
	void sendJsonResponse(int statusCode, const char* statusText, const char* detail,
	                  const char* path = nullptr, uint32_t bytesWritten = 0u,
	                  uint16_t vfsStatusCode = 0u, uint32_t stagedBytes = 0u,
	                  bool stagingUsesPsram = false);
};

}  // namespace dukatimer