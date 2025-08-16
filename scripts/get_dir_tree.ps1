<#
Usage:
  .\scripts\get_dir_tree.ps1
  .\scripts\get_dir_tree.ps1 "." "tree_filtered.txt"
#>

$base = Get-Location

# Args with safe defaults (no param{} block)
$Path    = if ($args.Count -ge 1 -and $args[0]) { $args[0] } else { "." }
$OutFile = if ($args.Count -ge 2 -and $args[1]) { $args[1] } else { "tree_filtered.txt" }

# Resolve absolute root
$root = (Resolve-Path -Path $Path).Path

# Hard-coded directories to exclude (folders and everything under them)
$ignoreDirs = @(
  '.venv', 'venv', '.git', '__pycache__', '.pytest_cache', '.vscode',
  '.mypy_cache', '.ruff_cache'
)

function Show-Tree {
  param(
    [string]$Path,
    [string]$Prefix = ""
  )

  # Children: skip ignored folders; sort dirs first, then files by name
  $children = Get-ChildItem -Force -LiteralPath $Path | Where-Object {
    if ($_.PSIsContainer) {
      $ignoreDirs -notcontains $_.Name
    } else {
      $true
    }
  }

  $dirs  = $children | Where-Object { $_.PSIsContainer } | Sort-Object Name
  $files = $children | Where-Object { -not $_.PSIsContainer } | Sort-Object Name
  $ordered = @()
  if ($dirs)  { $ordered += $dirs }
  if ($files) { $ordered += $files }

  for ($i = 0; $i -lt $ordered.Count; $i++) {
    $child  = $ordered[$i]
    $isLast = ($i -eq $ordered.Count - 1)

    # branch token
    $branch = '+-- '
    if ($isLast) { $branch = '\-- ' }

    # display name
    $name = $child.Name
    if ($child.PSIsContainer) { $name = "[$name]" }

    ($Prefix + $branch + $name) | Out-File -Append -Encoding utf8 $OutFile

    if ($child.PSIsContainer) {
      $nextPrefix = $Prefix
      if ($isLast) { $nextPrefix += '    ' } else { $nextPrefix += '|   ' }
      Show-Tree -Path $child.FullName -Prefix $nextPrefix
    }
  }
}

# Start fresh output with the root line
"[$((Split-Path -Leaf $root))]" | Out-File -Encoding utf8 $OutFile
Show-Tree -Path $root -Prefix ""
