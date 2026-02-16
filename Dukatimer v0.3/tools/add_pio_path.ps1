# add_pio_path.ps1
# Adds PlatformIO's venv Scripts folder to the user PATH (persistent)
# Usage: run in PowerShell (no admin required)

$pathEntry = "$env:USERPROFILE\.platformio\penv\Scripts"
$current = [Environment]::GetEnvironmentVariable('PATH','User')
if ($current -notlike "*$pathEntry*") {
  try {
    setx PATH "$current;$pathEntry" | Out-Null
    Write-Output "Added $pathEntry to user PATH. Close and reopen your terminals to take effect."
  } catch {
    Write-Error "Failed to set PATH: $_"
  }
} else {
  Write-Output "$pathEntry already present in user PATH."
}
