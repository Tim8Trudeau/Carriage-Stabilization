"""
Run this script in powershell while at the project root (directory above the script's location)
> .\scripts\get_dir_tree.ps1
"""

$base = Get-Location

function Show-PyTree {
    param (
        [string]$Path,
        [int]$Level = 0
    )

    $indent = '    ' * $Level
    $folderName = Split-Path $Path -Leaf

    # Skip any folder named venv
    if ($folderName -ieq 'venv') {
        return
    }

    Write-Output "$indent$folderName/"

    # List only .py files (not .pyc) directly in this folder
    Get-ChildItem -Path $Path -File -Filter *.py | Where-Object {
        $_.Extension -eq '.py'
    } | ForEach-Object {
        Write-Output "$indent    $($_.Name)"
    }

    # Recurse into subdirectories
    Get-ChildItem -Path $Path -Directory | ForEach-Object {
        Show-PyTree -Path $_.FullName -Level ($Level + 1)
    }
}

Show-PyTree -Path $base
