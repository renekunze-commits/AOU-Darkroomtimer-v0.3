# install_powershell_profile.ps1
# Adds a small helper function `pio-mon` to the current user's PowerShell profile
# Usage: run in PowerShell (will create profile file if absent)

$profileFile = $PROFILE.CurrentUserAllHosts
if (-not (Test-Path $profileFile)) { New-Item -ItemType File -Path $profileFile -Force | Out-Null }

$functionBlock = @'
function pio-mon {
  param([string]$port='COM12',[int]$baud=115200)
  pio device monitor -p $port -b $baud -f send_on_enter,time
}
'@

$content = Get-Content $profileFile -Raw -ErrorAction SilentlyContinue
if ($content -notmatch 'function pio-mon') {
  Add-Content -Path $profileFile -Value "`n# Added by Dukatimer helper`n$functionBlock"
  Write-Output "Added pio-mon to $profileFile. Restart PowerShell to use it (or run: . $profileFile)."
} else {
  Write-Output "pio-mon already present in profile."
}
