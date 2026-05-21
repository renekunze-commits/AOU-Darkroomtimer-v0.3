[CmdletBinding()]
param(
	[string]$PioArgumentsBase64,

	[string]$PioArgumentsJson,

	[Parameter(ValueFromRemainingArguments = $true)]
	[string[]]$PioArguments
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$isWindowsHost = $env:OS -eq "Windows_NT"
$projectRootForChildProcesses = $projectRoot
$projectRootSubstDrive = $null
$isolatedCoreDir = $null
$isolatedPioExe = $null
$isolatedPythonExe = $null
$penvDependencyHelper = Join-Path $projectRoot "tools\ensure_pioarduino_penv_deps.py"
$installerUrl = "https://raw.githubusercontent.com/pioarduino/pioarduino-core-installer/pioarduino/get-platformio.py"

function Update-IsolatedCoreBindings {
	param(
		[string]$BaseProjectRoot
	)

	$script:isolatedCoreDir = Join-Path $BaseProjectRoot ".pioarduino-core"
	$script:isolatedPioExe = Join-Path $script:isolatedCoreDir "penv\Scripts\platformio.exe"
	$script:isolatedPythonExe = Join-Path $script:isolatedCoreDir "penv\Scripts\python.exe"
}

function Enable-ShortProjectRootAlias {
	if (-not $isWindowsHost) {
		return
	}

	$candidateLetters = @("Z", "Y", "X", "W", "V", "U", "T", "S", "R", "Q", "P")
	foreach ($driveLetter in $candidateLetters) {
		$existingDrive = Get-PSDrive -Name $driveLetter -PSProvider FileSystem -ErrorAction SilentlyContinue
		if ($existingDrive) {
			continue
		}

		$substDrive = "${driveLetter}:"
		$substPath = "${substDrive}\"
		& subst $substDrive $projectRoot | Out-Null
		if ($LASTEXITCODE -eq 0 -and (Test-Path $substPath)) {
			$script:projectRootSubstDrive = $substDrive
			$script:projectRootForChildProcesses = $substPath
			Update-IsolatedCoreBindings -BaseProjectRoot $script:projectRootForChildProcesses
			return
		}
	}

	Update-IsolatedCoreBindings -BaseProjectRoot $script:projectRootForChildProcesses
}

function Disable-ShortProjectRootAlias {
	if ($script:projectRootSubstDrive) {
		& subst $script:projectRootSubstDrive /d | Out-Null
		$script:projectRootSubstDrive = $null
		$script:projectRootForChildProcesses = $script:projectRoot
	}

	Update-IsolatedCoreBindings -BaseProjectRoot $script:projectRootForChildProcesses
}

function Clear-StalePackageInstallTemp {
	$tmpRoot = Join-Path $isolatedCoreDir ".cache\tmp"
	if (-not (Test-Path $tmpRoot)) {
		return
	}

	Get-ChildItem -Path $tmpRoot -Directory -Filter "pkg-installing-*" -ErrorAction SilentlyContinue |
		ForEach-Object {
			Remove-Item $_.FullName -Recurse -Force -ErrorAction SilentlyContinue
		}
}

function Get-ArduinoLibsPackageVersion {
	$frameworkManifestPath = Join-Path $isolatedCoreDir "packages\framework-arduinoespressif32\package.json"
	if (Test-Path $frameworkManifestPath) {
		$frameworkManifest = Get-Content $frameworkManifestPath -Raw | ConvertFrom-Json
		if ($frameworkManifest.version) {
			return [string]$frameworkManifest.version
		}
	}

	$platformManifestPath = Join-Path $isolatedCoreDir "platforms\espressif32\platform.json"
	if (Test-Path $platformManifestPath) {
		$platformManifest = Get-Content $platformManifestPath -Raw | ConvertFrom-Json
		$libsVersionUrl = $platformManifest.packages."framework-arduinoespressif32-libs".version
		if ($libsVersionUrl -match "esp32-core-([0-9.]+)-libs\.tar\.xz") {
			return $Matches[1]
		}
	}

	return "3.3.8"
}

function Ensure-ArduinoLibsPackageManifest {
	$libsPackageDir = Join-Path $isolatedCoreDir "packages\framework-arduinoespressif32-libs"
	if (-not (Test-Path $libsPackageDir)) {
		return $false
	}

	$libsManifestPath = Join-Path $libsPackageDir "package.json"
	$needsRewrite = -not (Test-Path $libsManifestPath)
	if (-not $needsRewrite) {
		try {
			$manifestBytes = [System.IO.File]::ReadAllBytes($libsManifestPath)
			$hasUtf8Bom = $manifestBytes.Length -ge 3 -and $manifestBytes[0] -eq 0xEF -and $manifestBytes[1] -eq 0xBB -and $manifestBytes[2] -eq 0xBF
			$existingManifest = Get-Content $libsManifestPath -Raw | ConvertFrom-Json
			if ($hasUtf8Bom -or $existingManifest.name -ne "framework-arduinoespressif32-libs") {
				$needsRewrite = $true
			}
		}
		catch {
			$needsRewrite = $true
		}
	}

	if (-not $needsRewrite) {
		return $false
	}

	$libsManifest = [ordered]@{
		name = "framework-arduinoespressif32-libs"
		version = Get-ArduinoLibsPackageVersion
		description = "Precompiled Arduino core libraries for Espressif ESP32 targets"
		keywords = @("framework", "arduino", "espressif", "esp32", "libs")
		license = "LGPL-2.1-or-later"
		repository = [ordered]@{
			type = "git"
			url = "https://github.com/espressif/arduino-esp32"
		}
	}

	$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
	[System.IO.File]::WriteAllText($libsManifestPath, ($libsManifest | ConvertTo-Json -Depth 4), $utf8NoBom)
	Write-Host "Erzeuge oder repariere package.json fuer framework-arduinoespressif32-libs unter $libsManifestPath"
	return $true
}

function Resolve-BootstrapPython {
	$platformioPython = Join-Path $HOME ".platformio\penv\Scripts\python.exe"
	if (Test-Path $platformioPython) {
		return $platformioPython
	}

	$pythonCommand = Get-Command python -ErrorAction SilentlyContinue
	if ($pythonCommand -and $pythonCommand.Source -and $pythonCommand.Source -notlike "*WindowsApps*") {
		return $pythonCommand.Source
	}

	throw "Kein verlaesslicher Python-Interpreter fuer den isolierten pioarduino-Bootstrap gefunden. Erwartet wurde '$platformioPython' oder ein echtes python.exe ausserhalb von WindowsApps."
}

function Install-IsolatedCore {
	$bootstrapPython = Resolve-BootstrapPython
	$null = New-Item -ItemType Directory -Force -Path $isolatedCoreDir
	$installerFile = Join-Path ([System.IO.Path]::GetTempPath()) ("pioarduino-bootstrap-" + [guid]::NewGuid().ToString("N") + ".py")

	try {
		# Der Bootstrap bleibt projektlokal, damit das C6-Setup weder den globalen
		# PlatformIO-Core noch die Dukatimer-Part2-Umgebung umbiegt.
		Write-Host "Initialisiere isolierten pioarduino-Core unter $isolatedCoreDir"
		Invoke-WebRequest -Uri $installerUrl -OutFile $installerFile

		$previousCoreDir = $env:PLATFORMIO_CORE_DIR
		$env:PLATFORMIO_CORE_DIR = $isolatedCoreDir
		try {
			& $bootstrapPython $installerFile
			if ($LASTEXITCODE -ne 0) {
				throw "Der pioarduino-Bootstrap ist mit Exit-Code $LASTEXITCODE fehlgeschlagen."
			}
		}
		finally {
			if ($null -ne $previousCoreDir -and $previousCoreDir -ne "") {
				$env:PLATFORMIO_CORE_DIR = $previousCoreDir
			}
			else {
				Remove-Item Env:PLATFORMIO_CORE_DIR -ErrorAction SilentlyContinue
			}
		}

		if (-not (Test-Path $isolatedPioExe)) {
			throw "Der isolierte PlatformIO-Launcher wurde nicht unter '$isolatedPioExe' angelegt."
		}
	}
	finally {
		Remove-Item $installerFile -Force -ErrorAction SilentlyContinue
	}
}

function Test-HasEsp32Builder {
	$builderScript = Join-Path $isolatedCoreDir "platforms\espressif32\builder\penv_setup.py"
	return Test-Path $builderScript
}

function Ensure-IsolatedPenvDependencies {
	if (-not (Test-HasEsp32Builder)) {
		return
	}

	if (-not (Test-Path $isolatedPythonExe)) {
		throw "Der isolierte Python-Interpreter fehlt unter '$isolatedPythonExe'."
	}

	if (-not (Test-Path $penvDependencyHelper)) {
		throw "Der Dependency-Helper fehlt unter '$penvDependencyHelper'."
	}

	# Der pioarduino-Builder scheitert in dieser Lane unzuverlaessig beim eigenen
	# `uv pip install`. Der projektlokale Helper liest denselben Python-Contract
	# aus `penv_setup.py` und fuellt die isolierte penv vorab stabil auf.
	& $isolatedPythonExe $penvDependencyHelper --core-dir $isolatedCoreDir
	if ($LASTEXITCODE -ne 0) {
		throw "Der pioarduino-Dependency-Helper ist mit Exit-Code $LASTEXITCODE fehlgeschlagen."
	}
}

if ($PioArgumentsBase64) {
	# Fuer VS-Code-Tasks ist Base64 der robusteste Transportpfad, weil PowerShell
	# dabei keine eingebetteten Quotes oder PlatformIO-Flags vorab umdeutet.
	$decodedJson = [Text.Encoding]::UTF8.GetString([Convert]::FromBase64String($PioArgumentsBase64))
	$PioArguments = @([string[]](ConvertFrom-Json -InputObject $decodedJson))
}
elseif ($PioArgumentsJson) {
	# Die Task-Datei uebergibt PlatformIO-Argumente als JSON-Array, damit Flags
	# wie `-e` oder `-t` nicht schon vom PowerShell-Parser abgefangen werden.
	$PioArguments = @([string[]](ConvertFrom-Json -InputObject $PioArgumentsJson))
}

if (-not $PioArguments -or $PioArguments.Count -eq 0) {
	$PioArguments = @("--version")
}

Update-IsolatedCoreBindings -BaseProjectRoot $projectRootForChildProcesses
Enable-ShortProjectRootAlias

if (-not (Test-Path $isolatedPioExe)) {
	Install-IsolatedCore
}

# Jeder Aufruf dieses Wrappers bleibt an denselben projektlokalen Core gebunden.
# Dadurch kann Wireless TSL2591 einen eigenen pioarduino-Stack nutzen, waehrend
# andere Workspace-Projekte ihren bestehenden PlatformIO-Core unveraendert lassen.
$env:PLATFORMIO_CORE_DIR = $isolatedCoreDir

Push-Location $projectRoot
try {
	Clear-StalePackageInstallTemp
	$builderPresentBeforeInvoke = Test-HasEsp32Builder
	if ($builderPresentBeforeInvoke) {
		Ensure-IsolatedPenvDependencies
	}
	$repairedArduinoLibsManifest = Ensure-ArduinoLibsPackageManifest

	& $isolatedPioExe @PioArguments
	$exitCode = $LASTEXITCODE

	if ($exitCode -ne 0 -and -not $builderPresentBeforeInvoke -and (Test-HasEsp32Builder)) {
		Write-Host "Heile isolierte pioarduino-penv nach erstem Builder-Bootstrap und wiederhole den Aufruf einmal."
		Ensure-IsolatedPenvDependencies
		& $isolatedPioExe @PioArguments
		$exitCode = $LASTEXITCODE
	}

	if ($exitCode -ne 0 -and -not $repairedArduinoLibsManifest -and (Ensure-ArduinoLibsPackageManifest)) {
		Write-Host "Wiederhole den Aufruf einmal nach dem Nachziehen des fehlenden Arduino-Libs-Manifests."
		& $isolatedPioExe @PioArguments
		$exitCode = $LASTEXITCODE
	}
}
finally {
	Pop-Location
	Disable-ShortProjectRootAlias
}

exit $exitCode