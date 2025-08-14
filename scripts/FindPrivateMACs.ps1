# Discover active IPv4 + mask (prefer interface with a default gateway)
$iface = Get-NetIPConfiguration |
  Where-Object { $_.IPv4DefaultGateway -and $_.IPv4Address } |
  Select-Object -First 1

$ipStr   = $iface.IPv4Address.IPAddress
$prefix  = [int]$iface.IPv4Address.PrefixLength  # e.g., 22

Write-Host "Scanning $ipStr/$prefix for locally-administered (randomized) MACs...`n"

function IPToUint([string]$ip) {
  $b = [System.Net.IPAddress]::Parse($ip).GetAddressBytes()
  [Array]::Reverse($b)
  [BitConverter]::ToUInt32($b,0)
}
function UintToIP([uint32]$u) {
  $b = [BitConverter]::GetBytes([UInt32]$u)
  [Array]::Reverse($b)
  ([System.Net.IPAddress]::new($b)).ToString()
}

# Build mask safely as uint32
$allOnes = [uint32]::MaxValue
$hostBits = 32 - $prefix
$mask = [uint32]($allOnes -shl $hostBits)
$ipU  = [uint32](IPToUint $ipStr)
$net  = [uint32]($ipU -band $mask)
$hostMask = [uint32]($allOnes - $mask)
$bcast = [uint32]($net -bor $hostMask)

# ICMP sweep to populate ARP (lightweight; stop early if you like)
$start = $net + 1
$end   = $bcast - 1
$jobs = @()
for ($u=$start; $u -le $end; $u++) {
  $t = UintToIP $u
  if ($t -ne $ipStr) {
    $jobs += Start-Job -ScriptBlock { param($x) ping -n 1 -w 150 $x | Out-Null } -ArgumentList $t
  }
}
Start-Sleep -Seconds 5
Get-Job | Receive-Job -ErrorAction SilentlyContinue | Out-Null
Get-Job | Remove-Job -Force

# Parse ARP and flag locally-administered MACs (bit 0x02 set in first byte)
$entries = arp -a | ForEach-Object {
  if ($_ -match '(\d+\.\d+\.\d+\.\d+)\s+([0-9a-fA-F-]{17})\s+\w+') {
    [PSCustomObject]@{ IP=$matches[1]; MAC=$matches[2].ToLower() }
  }
} | Where-Object {
  $_.MAC -ne 'ff-ff-ff-ff-ff-ff' -and $_.IP -notmatch '^(224\.|239\.|255\.)'
}

$randomized = $entries | Where-Object {
  $fb = [Convert]::ToInt32($_.MAC.Substring(0,2),16)
  ($fb -band 0x02) -ne 0
} | Sort-Object IP

if ($randomized) {
  Write-Host "Likely randomized MACs:" -ForegroundColor Yellow
  $randomized | ForEach-Object { "{0,-15}  {1}" -f $_.IP, $_.MAC }
} else {
  Write-Host "No randomized MACs currently in ARP. Re-run when devices are online." -ForegroundColor Green
}
