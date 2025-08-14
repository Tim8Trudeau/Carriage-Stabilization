# --- Config ---
$ip  = "192.168.5.54"           # the unknown device IP
$mac = "f6-8e-53-e2-34-ef"      # its MAC (for display)

Write-Host "Watching $ip ($mac). Ctrl+C to stop.`n"

while ($true) {
    try {
        # 1) Clear any stale neighbor entry so the next probe is fresh
        arp -d $ip *>$null 2>&1

        # 2) Send a single ping to trigger ARP resolution (ICMP may be blocked; that's ok)
        ping -n 1 -w 800 $ip *>$null 2>&1

        # 3) Ask Windows for the neighbor table entry and state
        $nbr = Get-NetNeighbor -IPAddress $ip -ErrorAction SilentlyContinue

        if ($nbr -and $nbr.State -in @('Reachable','Stale','Delay','Probe')) {
            # We have a Layer-2 mapping → device is on the LAN (online locally)
            Write-Host "$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')  LAN: ONLINE  (State=$($nbr.State), MAC=$($nbr.LinkLayerAddress))" -ForegroundColor Green
        } else {
            # No neighbor entry → likely not present on the LAN right now
            Write-Host "$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')  LAN: OFFLINE (no ARP/neighbor entry)" -ForegroundColor Yellow
        }
    } catch {
        Write-Host "$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')  ERROR: $_" -ForegroundColor Red
    }
    Start-Sleep 5
}
